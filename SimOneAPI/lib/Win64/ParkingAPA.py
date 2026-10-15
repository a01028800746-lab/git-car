# -*- coding: utf-8 -*-
"""07.垂直泊车驶出-测试：真实油门/刹车/转向控制，不修改场景。

PyCharm: Python311，直接运行本文件；先关闭其它车辆控制算法。
离线检查: python ParkingAPA.py --self-test （不连接或启动仿真）。
本程序针对 cc20b542-b32e-47e4-aed9-ce089d134faa.sim，不是所有 APA 场景的通用程序。
几何检查、简化动力学检查不等于平台满分，首次运行须核对车辆参考点/转向标定。
"""

import argparse
import contextlib
import csv
import heapq
import json
import math
import os
from pathlib import Path
import sys
import time
import traceback
from uuid import UUID

PI = math.pi
START_GPS = (-40.64979996416881, -4.571679491258358, 0.0001041720923865343)
FINISH = (82.0, -4.558903)
CASE_NAME = "07.垂直泊车驶出-测试"
# Native HDMapModule.LoadDataFromContent/GetParkingSpaceList output, space id 10.
# The exporter's cornerLocal orientation must NOT be interpreted as ordinary hdg rotation.
BAY = ((-2.534078486986664, -10.447266915358352),
       (-2.5418909869925854, -4.443360665360068),
       (-4.995015986991407, -4.439454415361581),
       (-4.987203486985474, -10.451173165359862))
BAY_X = sum(p[0] for p in BAY) / 4
BAY_Y = sum(p[1] for p in BAY) / 4
ROAD_HEADING = START_GPS[2]
ROAD_Y_AT_ZERO = 1.4003906270276183 + 308.76301079251795 * math.tan(ROAD_HEADING)
# World position is obstacle centroid (SDK header); all obstacle dimensions from export.
OBSTACLES = ((-6.250702523017571, -7.819460316050709, PI / 2, 4.8, 2.23),
             (-1.2500000000000684, -7.860581962356957, PI / 2, 4.8, 2.23))
CAR_LENGTH, CAR_WIDTH, BOX_CENTER = 4.57, 2.21, -1.4
# Keep the body geometry in the rear-axle frame, independently of the SDK
# GPS origin. The exported front-reference box gives 3.1 - 1.4 = 1.7m.
BODY_CENTER_FROM_REAR = 1.7
# Fallback only; calibrated from world-coordinate wheel states when available.
WHEELBASE = 3.1
GPS_TO_REAR = 3.1
# Official AVP UtilDriver.hpp maps road wheel radians to percent by 36/(7*pi).
FULL_STEER_RAD = 7 * PI / 36
PLAN_STEER_RAD = 0.50
PARK_SECONDS = 12.0  # 10s rule + 2s margin, measured in simulation time.
EVALUATION_END_GRACE_SECONDS = 2.0  # Wall time for the SDK's asynchronous end event.
EVALUATION_VERIFY_SECONDS = 0.75


def clip(x, lo, hi):
    return max(lo, min(hi, x))


def wrap(a):
    return (a + PI) % (2 * PI) - PI


def rectangle(x, y, yaw, length, width, center=0.0):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + u*c - v*s, y + u*s + v*c)
            for u, v in ((center+length/2, width/2),
                         (center-length/2, width/2),
                         (center-length/2, -width/2),
                         (center+length/2, -width/2))]


def footprint(pose, margin=0.0):
    # pose = rear axle; GPS calibration must not move the physical body box.
    return rectangle(*pose, CAR_LENGTH+2*margin, CAR_WIDTH+2*margin,
                     BODY_CENTER_FROM_REAR)


def overlap(a, b):
    """Separating-axis test, including edge/edge collisions."""
    for poly in (a, b):
        for i, p in enumerate(poly):
            q = poly[(i+1) % len(poly)]
            nx, ny = p[1]-q[1], q[0]-p[0]
            aa = [x*nx+y*ny for x, y in a]
            bb = [x*nx+y*ny for x, y in b]
            if max(aa) < min(bb) or max(bb) < min(aa):
                return False
    return True


OBSTACLE_POLYS = [rectangle(*o) for o in OBSTACLES]


def safe(pose, margin=0.07):
    poly = footprint(pose, margin)
    for x, y in poly:
        top = ROAD_Y_AT_ZERO + x * math.tan(ROAD_HEADING)
        if not top-12 < y < top:
            return False
    return not any(overlap(poly, obs) for obs in OBSTACLE_POLYS)


def in_bay(pose, margin=0.015):
    # Convex quadrilateral containment with distance to each painted edge.
    for x, y in footprint(pose):
        signs = []
        for i, (ax, ay) in enumerate(BAY):
            bx, by = BAY[(i+1) % 4]
            cross = (bx-ax)*(y-ay) - (by-ay)*(x-ax)
            signs.append(cross / math.hypot(bx-ax, by-ay))
        if not (min(signs) >= margin or max(signs) <= -margin):
            return False
    return True


def integrate(pose, signed_distance, curvature):
    x, y, yaw = pose
    dyaw = signed_distance*curvature
    if abs(curvature) < 1e-10:
        return x+signed_distance*math.cos(yaw), y+signed_distance*math.sin(yaw), yaw
    return (x+(math.sin(yaw+dyaw)-math.sin(yaw))/curvature,
            y-(math.cos(yaw+dyaw)-math.cos(yaw))/curvature, wrap(yaw+dyaw))


def primitive(pose, direction, curvature, length):
    count = max(1, math.ceil(length/0.08))
    points = [integrate(pose, direction*length*i/count, curvature)
              for i in range(1, count+1)]
    return points if all(safe(p) for p in points) else None


def plan_maneuver(max_nodes=140000):
    """Hybrid A*: plan leaving the bay; parking is its time-reversed path.

    Search only near the bay. Straight final reverse is kept separate so yaw
    settles before the very narrow slot. No teleporting / score manipulation.
    """
    parked = (BAY_X, BAY_Y-BODY_CENTER_FROM_REAR, PI/2)
    mouth = (BAY_X, -4.78, PI/2)
    if not in_bay(parked) or not safe(parked):
        raise RuntimeError("Vehicle geometry cannot fit the target bay")
    first = primitive(parked, 1, 0, mouth[1]-parked[1])
    if first is None:
        raise RuntimeError("Straight bay exit is obstructed")
    kmax = math.tan(PLAN_STEER_RAD)/WHEELBASE
    turn_length = math.radians(10)/kmax
    actions = [(d, k, turn_length if k else 0.70)
               for d in (1, -1) for k in (0, -kmax, kmax)]

    def key(p, d):
        return (round(p[0]/0.22), round(p[1]/0.22),
                round(wrap(p[2])/math.radians(10)), d)

    def heuristic(p):
        # A helpful (not strictly admissible) weighted search heuristic.
        return 1.2*max(0, 4-p[0]) + 4*abs(wrap(p[2])) + abs(p[1]+2.4)

    # Each node keeps its actual continuous pose, parent, and incoming action.
    nodes = [(mouth, 1, 0.0, -1, None)]
    queue = [(heuristic(mouth), 0)]
    best = {key(mouth, 1): 0.0}
    goal = None
    began = time.monotonic()
    while queue and len(nodes) < max_nodes:
        _, ni = heapq.heappop(queue)
        p, old_d, cost, _, _ = nodes[ni]
        if cost > best.get(key(p, old_d), float('inf'))+1e-8:
            continue
        if 3 < p[0] < 16 and -3.0 < p[1] < -1.8 and abs(wrap(p[2])) < 0.001:
            goal = ni
            break
        for d, k, length in actions:
            end = integrate(p, d*length, k)
            if not (-15 < end[0] < 17 and -8 < end[1] < 0.3):
                continue
            new_cost = cost+length*(1.0 if d == 1 else 1.15)+(3.0 if old_d != d else 0)
            ek = key(end, d)
            if new_cost >= best.get(ek, float('inf')):
                continue
            samples = primitive(p, d, k, length)
            if samples is None:
                continue
            best[ek] = new_cost
            nodes.append((end, d, new_cost, ni, (d, k, length)))
            heapq.heappush(queue, (new_cost+1.6*heuristic(end), len(nodes)-1))
    if goal is None:
        raise RuntimeError(f"No collision-free parking path ({len(nodes)} nodes). Do not drive blind.")
    chain = []
    i = goal
    while nodes[i][3] != -1:
        chain.append((nodes[nodes[i][3]][0], nodes[i][4]))
        i = nodes[i][3]
    chain.reverse()
    # point fields = (rear_x, rear_y, car_yaw, physical_curvature).
    moves = [(1, [(*parked, 0.0)]+[(*p, 0.0) for p in first])]
    for p, (d, k, length) in chain:
        pts = [(*p, k)] + [(*q, k) for q in primitive(p, d, k, length)]
        if moves[-1][0] == d:
            moves[-1][1].extend(pts[1:])
        else:
            moves.append((d, pts))
    print(f"Plan: {len(nodes)} nodes, {time.monotonic()-began:.1f}s, "
          f"{len(moves)-1} gear changes per maneuver; staging={nodes[goal][0]}", flush=True)
    return parked, moves


def reverse_moves(moves):
    # Reversing traversal reverses gear, but physical curvature stays unchanged.
    result = []
    for direction, points in reversed(moves):
        # Curvature stored at the arriving sample. Reverse each edge's association.
        rev = list(reversed(points))
        result.append((-direction, [(*p[:3], rev[min(i+1, len(rev)-1)][3])
                                    for i, p in enumerate(rev)]))
    return result


def road_path(start, end, spacing=0.12):
    """Quintic lane change: zero yaw/curvature at both ends."""
    x0, y0 = start[:2]
    x1, y1 = end[:2]
    if x1 <= x0+5:
        raise RuntimeError("Insufficient distance for road approach")
    # Complete lane shift before reaching the two parked vehicles.
    change_len = min(22.0, x1-x0)
    n = math.ceil((x1-x0)/spacing)
    points = []
    for i in range(n+1):
        x = x0+(x1-x0)*i/n
        u = clip((x-x0)/change_len, 0, 1)
        f = 10*u**3-15*u**4+6*u**5
        fp = (30*u**2-60*u**3+30*u**4)/change_len
        fpp = (60*u-180*u**2+120*u**3)/change_len**2
        slope = (y1-y0)*fp
        points.append((x, y0+(y1-y0)*f, math.atan(slope),
                       (y1-y0)*fpp/(1+slope*slope)**1.5))
    if not all(safe(p[:3]) for p in points):
        raise RuntimeError("Road path intersects vehicle or boundary")
    return points


class Tracker:
    def __init__(self, points, direction):
        self.points, self.direction, self.index = points, direction, 0
        self.distances = [0.0]
        for a, b in zip(points, points[1:]):
            self.distances.append(self.distances[-1]+math.hypot(b[0]-a[0], b[1]-a[1]))

    def update(self, pose, speed, road=False):
        pts = self.points
        lo = max(0, self.index-4)
        hi = min(len(pts), self.index+45)
        self.index = min(range(lo, hi), key=lambda i: math.hypot(pose[0]-pts[i][0], pose[1]-pts[i][1]))
        a = pts[self.index]
        b = pts[min(self.index+1, len(pts)-1)]
        dx, dy = b[0]-a[0], b[1]-a[1]
        t = clip(((pose[0]-a[0])*dx+(pose[1]-a[1])*dy)/max(1e-12, dx*dx+dy*dy), 0, 1)
        refx, refy = a[0]+t*dx, a[1]+t*dy
        yaw = a[2]+t*wrap(b[2]-a[2])
        ey = -(pose[0]-refx)*math.sin(yaw)+(pose[1]-refy)*math.cos(yaw)
        eh = wrap(pose[2]-yaw)
        curvature = b[3] - (0.14 if road else 0.50)*ey - self.direction*(0.8 if road else 1.5)*eh
        steering = clip(-math.atan(WHEELBASE*curvature)/FULL_STEER_RAD, -1, 1)
        remaining = self.distances[-1]-self.distances[self.index]-t*math.hypot(dx, dy)
        end = pts[-1]
        dist = math.hypot(pose[0]-end[0], pose[1]-end[1])
        # Signed distance past end avoids trying to reverse direction with the wrong gear.
        along = self.direction*((end[0]-pose[0])*math.cos(end[2])+(end[1]-pose[1])*math.sin(end[2]))
        limit = 5.5 if road else (0.80 if abs(curvature)>0.08 else 1.0)
        target = min(limit, math.sqrt(max(0, 1.0*remaining)), max(0.0, along)*1.4+0.06)
        if along <= 0.045 or (dist < 0.075 and remaining < 0.12):
            target = 0.0
        done = target == 0 and speed < 0.025
        return steering, target, done, ey, eh, dist


class SpeedController:
    def __init__(self):
        self.integral = 0.0

    def update(self, target, speed, dt):
        if target <= 0:
            self.integral = 0.0
            return 0.0, 0.55
        error = target-speed
        # Anti-windup while braking; no previous segment's acceleration demand.
        self.integral = clip(self.integral+error*dt, -0.6, 0.9)
        command = 0.12+0.32*error+0.12*self.integral
        if speed > target+0.12:
            return 0.0, clip(0.25*(speed-target), 0.04, 0.7)
        return clip(command, 0.0, 0.65), 0.0


class ParkingController:
    def __init__(self, start_pose, moves):
        parking = reverse_moves(moves)
        staging = moves[-1][1][-1][:3]
        self.segments = [('APPROACH', 1, road_path(start_pose, staging))]
        self.segments += [(f'PARK_{i+1}', d, p) for i, (d, p) in enumerate(parking)]
        self.segments += [('HOLD_12S', -1, None)]
        self.segments += [(f'EXIT_{i+1}', d, p) for i, (d, p) in enumerate(moves)]
        # Give the lane transition enough distance; goal trigger fires within 5m of82.
        final_rear = (FINISH[0]-GPS_TO_REAR, FINISH[1], ROAD_HEADING)
        self.segments += [('TO_FINISH', 1, road_path(staging, final_rear))]
        self.segment = 0
        self.tracker = Tracker(self.segments[0][2], 1)
        self.speed_ctrl = SpeedController()
        self.shift_until = None
        self.hold_since = None
        self.last_time = None
        self.previous_gear = 1
        self.complete = False

    @property
    def name(self):
        return 'COMPLETE' if self.complete else self.segments[self.segment][0]

    def advance(self, sim_time, pose):
        self.segment += 1
        self.hold_since = None
        self.shift_until = None
        self.speed_ctrl = SpeedController()
        if self.segment >= len(self.segments):
            self.complete = True
            return
        name, direction, points = self.segments[self.segment]
        if points:
            # Brake before shifting; allow steering/gear to settle at each cusp.
            self.tracker = Tracker(points, direction)
            self.shift_until = sim_time+0.8
        print(f'STATE -> {name} at sim={sim_time:.2f}s, rear={pose}', flush=True)

    def update(self, pose, speed, sim_time):
        dt = 0.02 if self.last_time is None else clip(sim_time-self.last_time, 0.005, 0.2)
        self.last_time = sim_time
        if not safe(pose, 0.0):
            raise RuntimeError('Observed vehicle footprint intersects parked car / road boundary')
        if self.complete:
            return 0.0, 0.0, 0.65, 0, (0, 0, 0)
        name, direction, points = self.segments[self.segment]
        if points is None:
            if not in_bay(pose):
                raise RuntimeError('Not fully inside bay at parking stop; refusing to count it as success')
            if speed < 0.025:
                if self.hold_since is None:
                    self.hold_since = sim_time
                if sim_time-self.hold_since >= PARK_SECONDS:
                    self.advance(sim_time, pose)
            else:
                self.hold_since = None
            return 0.0, 0.0, 0.65, 0, (0, 0, 0)
        steer, target, done, ey, eh, dist = self.tracker.update(pose, speed, name in ('APPROACH', 'TO_FINISH'))
        if abs(ey) > 0.8 or abs(eh) > math.radians(35):
            raise RuntimeError(f'Path tracking diverged: lateral={ey:.2f}m yaw={math.degrees(eh):.1f}deg')
        if self.shift_until is not None:
            if speed >= 0.025:
                self.shift_until = sim_time+0.8
                return steer, 0.0, 0.65, self.previous_gear, (ey, eh, dist)
            self.previous_gear = direction
            if sim_time < self.shift_until:
                return steer, 0.0, 0.65, direction, (ey, eh, dist)
            self.shift_until = None
        if done:
            if dist > 0.22 or abs(eh) > math.radians(2):
                raise RuntimeError(f'Segment ended inaccurately: {dist:.3f}m / {math.degrees(eh):.2f}deg')
            self.advance(sim_time, pose)
            return steer, 0.0, 0.65, direction, (ey, eh, dist)
        throttle, brake = self.speed_ctrl.update(target, speed, dt)
        self.previous_gear = direction
        return steer, throttle, brake, direction, (ey, eh, dist)


def pid_alive(pid):
    if os.name == 'nt':
        import ctypes
        kernel = ctypes.WinDLL('kernel32', use_last_error=True)
        kernel.OpenProcess.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_uint32]
        kernel.OpenProcess.restype = ctypes.c_void_p
        kernel.CloseHandle.argtypes = [ctypes.c_void_p]
        handle = kernel.OpenProcess(0x1000, False, pid)
        if handle:
            kernel.CloseHandle(handle)
            return True
        # Access denied is not proof that a process has exited.
        return ctypes.get_last_error() == 5
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False


def measured_geometry(gps, values):
    """Wheel positions must be world coordinates and sampled while stationary."""
    if len(values) != 8 or not all(math.isfinite(v) for v in values):
        print('Wheel calibration unavailable: incomplete/non-finite coordinates.', flush=True)
        return None
    fx, fy = (values[0]+values[2])/2, (values[1]+values[3])/2
    rx, ry = (values[4]+values[6])/2, (values[5]+values[7])/2
    c, s = math.cos(gps.oriZ), math.sin(gps.oriZ)
    wheelbase = (fx-rx)*c+(fy-ry)*s
    offset = (gps.posX-rx)*c+(gps.posY-ry)*s
    lateral = -(gps.posX-rx)*s+(gps.posY-ry)*c
    track = math.hypot(values[0]-values[2], values[1]-values[3])
    rear_track = math.hypot(values[4]-values[6], values[5]-values[7])
    axle_lateral = -(fx-rx)*s+(fy-ry)*c
    print(f'Wheel calibration: xy={list(values)}; wheelbase={wheelbase:.4f}m; '
          f'GPS_to_rear={offset:.4f}m; GPS_lateral={lateral:.4f}m; '
          f'tracks=({track:.4f}, {rear_track:.4f})m', flush=True)
    if not (2.4 < wheelbase < 3.5 and -0.2 < offset < 3.8
            and abs(lateral) < 0.12 and abs(axle_lateral) < 0.12
            and 1.3 < track < 2.2 and 1.3 < rear_track < 2.2):
        print('Wheel calibration rejected: unsupported coordinates or inconsistent axles.', flush=True)
        return None  # Unsupported/local-coordinate/empty state data.
    # Compare axles and body in ONE frame, not exported box vs SDK GPS frame.
    rear_in_box = BODY_CENTER_FROM_REAR-CAR_LENGTH/2-0.15 < 0
    front_in_box = wheelbase < BODY_CENTER_FROM_REAR+CAR_LENGTH/2+0.15
    if not rear_in_box or not front_in_box:
        raise RuntimeError('Measured axles cannot fit rear-frame body geometry')
    return wheelbase, offset


def stop_requested(path, pid):
    try:
        target = path.read_text(encoding='ascii').strip()
    except FileNotFoundError:
        return False
    # Empty requests from the previous version still mean stop, never ignore them.
    if not target or target == str(pid):
        return True
    if not target.isdecimal():
        raise RuntimeError(f'Invalid APA stop request: {target!r}')
    print(f'Ignoring stop request for previous PID {target}; current PID={pid}', flush=True)
    path.unlink()
    return False


def evaluation_record_path(sdk_directory, task_id):
    """Locate this task's SDK JSON using existing platform config, without edits."""
    # A task ID is a UUID, never a user-supplied relative filename.
    task_id = str(UUID(task_id))
    install_root = Path(sdk_directory).resolve().parents[2]
    server_directory = install_root/'Web'/'TaskServer'
    root = Path.home()/'.simone'
    config_path = install_root/'Common'/'Conf'/'simone.cfg'
    if config_path.is_file():
        config = json.loads(config_path.read_text(encoding='utf-8-sig'))
        if config.get('user_data'):
            root = Path(config['user_data'])
    if not root.is_absolute():
        root = server_directory/root
    directory = root/'evaluation'
    app_config_path = server_directory/'config'/'app_config.json'
    if app_config_path.is_file():
        app_config = json.loads(app_config_path.read_text(encoding='utf-8-sig'))
        override = (app_config.get('dir') or {}).get('evaluation')
        if override:
            directory = Path(override)
            if not directory.is_absolute():
                directory = server_directory/directory
    return directory/(task_id+'.json')


def finalize_evaluation(save_record, get_status, record_path, *, stopped=False,
                        clock=time.monotonic, sleep=time.sleep, report=None):
    """Drain end events, save ONCE, and inspect the SDK's own record read-only.

    No driving commands, invented success flags, uploads, or evaluation-file edits.
    The waits are bounded; a blocking native SDK call cannot be time-bounded here.
    Repeated saves are avoided because the SDK's buffer-retention contract is unknown.
    """
    if report is None:
        report = lambda message: print(message, flush=True)
    result = {'saved': False, 'end_reason': None, 'verified': False}
    if stopped:
        report(f'Evaluation finalize: keeping SDK connected for '
               f'{EVALUATION_END_GRACE_SECONDS:.2f} wall seconds; no control commands.')
        deadline = clock()+EVALUATION_END_GRACE_SECONDS
        while True:
            # Do not save another case's data if a new task starts during cleanup.
            if get_status() != 1:
                report('EVALUATION WARNING: case is no longer Stop; save skipped '
                       'to avoid mixing tasks. Evaluation end is UNCONFIRMED.')
                return result
            remaining = deadline-clock()
            if remaining <= 0:
                break
            sleep(min(0.05, remaining))
    try:
        result['saved'] = bool(save_record())
    except Exception as exc:
        report(f'Evaluation save error: {exc}')
        return result
    report(f'SaveEvaluationRecord={result["saved"]}')
    if not result['saved']:
        report('EVALUATION WARNING: save failed; no successful evaluation claimed.')
        return result
    if record_path is None:
        report('EVALUATION WARNING: task record path unavailable; '
               'save succeeded but evaluation end is UNCONFIRMED.')
        return result
    report(f'Evaluation record (read-only check): {record_path}')
    deadline = clock()+EVALUATION_VERIFY_SECONDS
    last_error = None
    record = None
    while True:
        try:
            record = json.loads(record_path.read_text(encoding='utf-8-sig'))
            if not isinstance(record, dict):
                raise ValueError('Evaluation JSON is not an object')
            reason = record.get('endreason')
            # Protocol ETaskEndReason: 0=None, 1=Goal; other positive codes
            # include collisions, interruptions and judge failures, NOT success.
            if type(reason) is int and reason >= 0:
                result['end_reason'] = reason
                if reason > 0:
                    result['verified'] = True
                    break
            last_error = None
        except (OSError, ValueError, UnicodeError) as exc:
            last_error = str(exc)
        remaining = deadline-clock()
        if remaining <= 0:
            break
        sleep(min(0.05, remaining))
    reason = result['end_reason']
    label = {0: 'NONE', 1: 'GOAL', 2: 'COLLISION', 3: 'USER_INTERRUPT',
             6: 'TIMEOUT', 8: 'TRAFFIC_EVENT', 9: 'PARKING'}.get(reason, 'OTHER/UNKNOWN')
    records = record.get('records') if isinstance(record, dict) else None
    record_count = len(records) if isinstance(records, list) else 'not supplied'
    report(f'Evaluation endreason={reason} ({label}); judge_records={record_count}')
    if not result['verified']:
        if last_error:
            report(f'Evaluation record check: {last_error}')
        report('EVALUATION WARNING: end reason is missing/unspecified. '
               'Save=True is not a complete evaluation or a passing score.')
    elif reason != 1:
        report('EVALUATION WARNING: SDK recorded a non-Goal end reason; '
               'preserved unchanged. Check the platform failure report.')
    return result


class DiagnosticStream:
    def __init__(self, console, log):
        self.console, self.log = console, log

    def write(self, text):
        self.log.write(text)
        self.log.flush()
        if self.console is not None:
            try:
                self.console.write(text)
            except (OSError, UnicodeError):
                pass  # A closed launcher console must not stop vehicle control.
        return len(text)

    def flush(self):
        self.log.flush()
        if self.console is not None:
            try:
                self.console.flush()
            except OSError:
                pass


def run_live(moves, log_path=None):
    global WHEELBASE, GPS_TO_REAR
    # SDK has relative DLL lookup; set it only inside this process, no global edits.
    directory = Path(__file__).resolve().parent
    original_directory = Path.cwd()
    os.chdir(directory)
    dll_scope = os.add_dll_directory(str(directory)) if hasattr(os, 'add_dll_directory') else None
    from SimOneIOStruct import (SimOne_Data_Control, SimOne_Data_Gps,
                                SimOne_Data_CaseInfo, ESimOne_Drive_Mode,
                                ESimOne_Throttle_Mode, ESimOne_Brake_Mode,
                                ESimOne_Steering_Mode, SimOne_Data_Obstacle,
                                ESimOne_Data_Vehicle_State, SimOne_Data_Vehicle_Extra)
    from SimOneServiceAPI import (SoInitSimOneAPI, SoTerminateSimOneAPI,
                                  SoGetCaseRunStatus, SoAPIGetCaseInfo)
    from SimOnePNCAPI import (SoSetDrive, SoSetDriveMode, SoSetDriverName,
                             SoRegisterVehicleState, SoGetVehicleState)
    from SimOneSensorAPI import SoGetGps, SoGetGroundTruth
    from SimOneEvaluationAPI import SoInitEvaluationServiceWithLocalData, SoSaveEvaluationRecord

    # Refuse known active launcher/LKA/APA processes instead of killing user programs.
    for other in (directory/'.TestDrive.lock', directory/'.simone_python_algorithm.pid'):
        if other.exists():
            try:
                pid = int(other.read_text(encoding='ascii').strip())
            except ValueError:
                raise RuntimeError(f'Unrecognized lock: {other}')
            # run_algorithm.py may be the parent launching THIS file.
            if pid not in (os.getpid(), os.getppid()) and pid_alive(pid):
                raise RuntimeError(f'Another controller is active (PID {pid}); close it first')
    lock = directory/'.ParkingAPA.lock'
    if lock.exists():
        pid = int(lock.read_text(encoding='ascii').strip())
        if pid_alive(pid):
            raise RuntimeError(f'ParkingAPA already active: PID {pid}')
        lock.unlink()  # A checked stale lock only; never remove another active lock.
    lock_fd = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
    os.write(lock_fd, str(os.getpid()).encode('ascii'))
    os.close(lock_fd)
    stop_request = directory/'.ParkingAPA.stop'
    # Targeted old requests are filtered in the loop; never erase a concurrent stop.
    if log_path is None:
        log_path = directory/(time.strftime('ParkingAPA-%Y%m%d-%H%M%S')+f'-{os.getpid()}.csv')
    seen_running = connected = evaluation = False
    controller = None
    last_stamp = first_stamp = None
    last_fresh = waiting_since = time.monotonic()
    last_print = last_eval = 0.0
    last_control = None
    finish_distance = float('inf')
    failure = None
    configured = False
    takeover_requested = False
    phase = 'CONNECTING'
    previous_status = None
    state_registered = False
    stationary_since = None
    platform_stopped = False
    evaluation_path = None
    evaluation_result = {'saved': False, 'end_reason': None, 'verified': False}
    # measured_geometry expects front L/R x/y, then rear L/R x/y.
    state_ids = (ESimOne_Data_Vehicle_State*8)(
        ESimOne_Data_Vehicle_State.ESimOne_Data_Vehicle_State_SO_X_L1,
        ESimOne_Data_Vehicle_State.ESimOne_Data_Vehicle_State_SO_Y_L1,
        ESimOne_Data_Vehicle_State.ESimOne_Data_Vehicle_State_SO_X_R1,
        ESimOne_Data_Vehicle_State.ESimOne_Data_Vehicle_State_SO_Y_R1,
        ESimOne_Data_Vehicle_State.ESimOne_Data_Vehicle_State_SO_X_L2,
        ESimOne_Data_Vehicle_State.ESimOne_Data_Vehicle_State_SO_Y_L2,
        ESimOne_Data_Vehicle_State.ESimOne_Data_Vehicle_State_SO_X_R2,
        ESimOne_Data_Vehicle_State.ESimOne_Data_Vehicle_State_SO_Y_R2,
    )

    def command(steer=0.0, throttle=0.0, brake=0.65, direction=1, stamp=0, frame=0):
        c = SimOne_Data_Control()
        c.timestamp, c.frame = stamp, frame
        c.EThrottleMode = ESimOne_Throttle_Mode.ESimOne_Throttle_Mode_Percent
        c.EBrakeMode = ESimOne_Brake_Mode.ESimOne_Brake_Mode_Percent
        c.ESteeringMode = ESimOne_Steering_Mode.ESimOne_Steering_Mode_Percent
        c.steering, c.throttle, c.brake = steer, throttle, brake
        c.handbrake, c.isManualGear = False, False
        # SimOne gear values: 0 Neutral, 1 Drive, 2 Reverse.
        c.gear = 0 if direction == 0 else (1 if direction > 0 else 2)
        return c

    try:
        with log_path.open('x', encoding='utf-8-sig', newline='') as log:
            writer = csv.writer(log)
            writer.writerow(('sim_s', 'timestamp_ms', 'frame', 'state', 'gps_x', 'gps_y',
                             'yaw_deg', 'speed_mps', 'gear_cmd', 'gps_gear', 'steer_cmd',
                             'gps_steer_deg', 'throttle', 'brake', 'lateral_m', 'yaw_error_deg',
                             'end_distance_m', 'drive_ok', 'in_bay', 'yaw_rate', 'vel_x', 'vel_y'))
            print(f'CSV log: {log_path}\nWaiting for APA case / Ego...', flush=True)
            # Current wrapper may return 0 after successful connection; verify GPS/status instead.
            SoInitSimOneAPI('Ego', 0, '127.0.0.1')
            connected = True
            last_fresh = waiting_since = time.monotonic()
            phase = 'WAIT_RUNNING'
            print('SDK initialization returned; waiting for Running status and fresh GPS.', flush=True)
            while True:
                status = SoGetCaseRunStatus()
                now = time.monotonic()
                if status != previous_status:
                    print(f'Case status={status}; phase={phase}', flush=True)
                    previous_status = status
                if status == 1 and seen_running:
                    platform_stopped = True
                    print('Platform case stopped.', flush=True)
                    break
                if stop_requested(stop_request, os.getpid()):
                    raise RuntimeError(f'Graceful stop requested by APA end script; phase={phase}, '
                                       f'API takeover attempted={takeover_requested}')
                if status != 2:
                    last_fresh = now  # GPS timeout starts when the case is Running.
                    if not seen_running and now-waiting_since > 180:
                        raise RuntimeError('No Running case after 180 wall seconds')
                    time.sleep(0.01)
                    continue
                seen_running = True
                if not configured:
                    phase = 'WAIT_GPS'
                gps = SimOne_Data_Gps()
                valid = SoGetGps('Ego', gps) and not gps.isGPSLost
                fresh = valid and gps.timestamp != last_stamp
                if not fresh:
                    if configured and now-last_fresh > 0.35:
                        SoSetDrive('Ego', command(brake=1.0, direction=0,
                                                 stamp=last_stamp or 0))
                    if now-last_fresh > 5:
                        raise RuntimeError('GPS absent/stale for 5 wall seconds; braking')
                    time.sleep(0.005)
                    continue
                values = (gps.posX, gps.posY, gps.oriZ, gps.velX, gps.velY)
                if not all(math.isfinite(v) for v in values):
                    raise RuntimeError('Non-finite GPS')
                if last_stamp is not None and gps.timestamp < last_stamp:
                    raise RuntimeError('Simulation timestamp reset; restart algorithm for next case')
                last_fresh, last_stamp = now, gps.timestamp
                speed = math.hypot(gps.velX, gps.velY)
                finish_distance = math.hypot(gps.posX-FINISH[0], gps.posY-FINISH[1])
                yaw = gps.oriZ
                pose = (gps.posX-GPS_TO_REAR*math.cos(yaw),
                        gps.posY-GPS_TO_REAR*math.sin(yaw), yaw)
                if not configured:
                    phase = 'VALIDATE_CASE'
                    info = SimOne_Data_CaseInfo()
                    if not SoAPIGetCaseInfo(info):
                        if now-waiting_since > 5:
                            raise RuntimeError('Cannot read case info after 5 wall seconds; no control sent')
                        time.sleep(0.005)
                        continue
                    live_name = bytes(info.caseName).split(b'\x00')[0].decode('utf-8', errors='replace')
                    print(f'Case={live_name}; GPS=({gps.posX:.3f}, {gps.posY:.3f}); '
                          f'yaw={math.degrees(yaw):.2f}deg; speed={speed:.3f}m/s', flush=True)
                    if CASE_NAME not in live_name:
                        raise RuntimeError(f'Wrong scenario: {live_name}; this program is for {CASE_NAME}')
                    if math.hypot(gps.posX-START_GPS[0], gps.posY-START_GPS[1]) > 5 or abs(wrap(yaw-START_GPS[2])) > 0.15:
                        raise RuntimeError('Connected too late or wrong starting pose; start a fresh case')
                    task_id = bytes(info.taskId).split(b'\x00')[0].decode('ascii', errors='replace')
                    try:
                        evaluation_path = evaluation_record_path(directory, task_id)
                        print(f'Evaluation task_id={task_id}; record={evaluation_path}', flush=True)
                    except (OSError, ValueError, TypeError) as path_exc:
                        print(f'Evaluation record path unavailable: {path_exc}; '
                              'saving remains enabled, verification will be UNCONFIRMED.', flush=True)
                    phase = 'API_TAKEOVER'
                    takeover_requested = True
                    # SimOnePNCAPI.h declares SetDriverName as void.  The bundled
                    # Python wrapper incorrectly exposes its undefined register
                    # value as c_bool, so a False result does not mean failure.
                    driver_result = SoSetDriverName('Ego', 'APA')
                    mode_ok = SoSetDriveMode('Ego', ESimOne_Drive_Mode(0))
                    print(f'SetDriverName=called(result ignored: {driver_result!r}); '
                          f'SetDriveMode={mode_ok!r}', flush=True)
                    if not mode_ok:
                        raise RuntimeError(f'Cannot enable APA API control: '
                                           f'SetDriverName called, SetDriveMode={mode_ok!r}')
                    first_stamp = gps.timestamp
                    configured = True
                    phase = 'INITIAL_BRAKE'
                    # Latch the brake before SDK service calls that may block.
                    initial_control = command(brake=1.0, direction=0,
                                              stamp=gps.timestamp, frame=gps.frame)
                    ok = SoSetDrive('Ego', initial_control)
                    writer.writerow((0.0, gps.timestamp, gps.frame, 'INITIAL_BRAKE',
                                     gps.posX, gps.posY, math.degrees(yaw), speed, 0, gps.gear,
                                     0, gps.steering, 0, 1, 0, 0, 0, ok, False,
                                     gps.angVelZ, gps.velX, gps.velY))
                    log.flush()
                    if not ok:
                        raise RuntimeError('SetDrive rejected initial neutral brake')
                    print('Initial neutral brake accepted; initializing SDK services.', flush=True)
                    state_registered = bool(SoRegisterVehicleState('Ego', state_ids, len(state_ids)))
                    print(f'Wheel state registration={state_registered}', flush=True)
                    evaluation = bool(SoInitEvaluationServiceWithLocalData('Ego', True))
                    last_eval = time.monotonic()
                    print(f'Evaluation init={evaluation}; startup services '
                          f'took {last_eval-now:.2f} wall seconds; refreshing GPS.', flush=True)
                    continue
                sim_s = (gps.timestamp-first_stamp)/1000.0
                if controller is None:
                    if speed >= 0.025:
                        stationary_since = None
                    elif stationary_since is None:
                        stationary_since = sim_s
                    if stationary_since is None or sim_s-stationary_since < 0.3:
                        ok = SoSetDrive('Ego', command(brake=1.0, direction=0,
                                                       stamp=gps.timestamp, frame=gps.frame))
                        writer.writerow((round(sim_s, 3), gps.timestamp, gps.frame, 'INITIAL_BRAKE',
                                         gps.posX, gps.posY, math.degrees(yaw), speed, 0, gps.gear,
                                         0, gps.steering, 0, 1, 0, 0, 0, ok, False,
                                         gps.angVelZ, gps.velX, gps.velY))
                        if now-last_print > 1:
                            last_print = now
                            log.flush()
                            print(f'{sim_s:6.2f}s INITIAL_BRAKE speed={speed:.3f}m/s '
                                  f'gear=N brake=1.0 drive_ok={ok}', flush=True)
                        if not ok or sim_s > 12:
                            raise RuntimeError('Initial neutral brake cannot stop Ego within 12 simulation seconds; '
                                               'check API takeover / competing controller')
                        if not evaluation and now-last_eval >= 1:
                            evaluation = bool(SoInitEvaluationServiceWithLocalData('Ego', True))
                            last_eval = time.monotonic()
                            stationary_since = None  # Require fresh stopped GPS after a blocking call.
                            print(f'Evaluation retry={evaluation}', flush=True)
                        time.sleep(0.005)
                        continue
                    extra = SimOne_Data_Vehicle_Extra()
                    geometry = None
                    if state_registered and SoGetVehicleState('Ego', extra) and extra.dataSize >= 8:
                        geometry = measured_geometry(gps, extra.extra_states[:8])
                    if geometry is not None:
                        WHEELBASE, GPS_TO_REAR = geometry
                        print(f'Measured wheelbase={WHEELBASE:.3f}m; GPS to rear axle={GPS_TO_REAR:.3f}m', flush=True)
                        _, moves = plan_maneuver()
                        pose = (gps.posX-GPS_TO_REAR*math.cos(yaw),
                                gps.posY-GPS_TO_REAR*math.sin(yaw), yaw)
                    else:
                        print('Wheel geometry unavailable: using exported 3.1m wheelbase/front-reference '
                              'assumption. Live vehicle origin still needs verification.', flush=True)
                    controller = ParkingController(pose, moves)
                    # Settle the initial N -> D shift just like later D/R shifts.
                    controller.shift_until = sim_s + 0.8
                    phase = 'DRIVING'
                    print(f'GPS reference offset={GPS_TO_REAR:.3f}m; wheelbase={WHEELBASE:.3f}m; '
                          f'initial D shift settles at sim={controller.shift_until:.2f}s', flush=True)
                if sim_s > 118:
                    raise RuntimeError('118 simulation seconds reached; task did not finish before scene timeout')
                control_state = controller.name
                steer, throttle, brake, direction, errors = controller.update(pose, speed, sim_s)
                # Current scene has static neighbours. Validate their live positions when available.
                if controller.name.startswith('PARK') or controller.name.startswith('EXIT'):
                    obs = SimOne_Data_Obstacle()
                    if SoGetGroundTruth('Ego', obs):
                        for o in obs.obstacle[:min(obs.obstacleSize, len(obs.obstacle))]:
                            # Some versions include Ego itself, with centroid rather than GPS position.
                            ego_center = (pose[0]+BODY_CENTER_FROM_REAR*math.cos(yaw),
                                          pose[1]+BODY_CENTER_FROM_REAR*math.sin(yaw))
                            if (abs(o.length-CAR_LENGTH) < 0.1 and abs(o.width-CAR_WIDTH) < 0.1
                                    and math.hypot(o.posX-ego_center[0], o.posY-ego_center[1]) < 0.4):
                                continue
                            if o.length > 0 and o.width > 0 and math.hypot(o.posX-gps.posX, o.posY-gps.posY) > 0.3:
                                if overlap(footprint(pose, 0.03), rectangle(o.posX, o.posY, o.oriZ, o.length, o.width)):
                                    raise RuntimeError('Live ground-truth obstacle too close; emergency brake')
                last_control = command(steer, throttle, brake, direction, gps.timestamp, gps.frame)
                ok = SoSetDrive('Ego', last_control)
                writer.writerow((round(sim_s, 3), gps.timestamp, gps.frame, control_state,
                                 gps.posX, gps.posY, math.degrees(yaw), speed, last_control.gear.value, gps.gear,
                                 steer, gps.steering, throttle, brake, errors[0], math.degrees(errors[1]),
                                 errors[2], ok, in_bay(pose), gps.angVelZ, gps.velX, gps.velY))
                if not ok:
                    raise RuntimeError('SetDrive rejected control command')
                if now-last_print > 1:
                    last_print = now
                    log.flush()
                    print(f'{sim_s:6.2f}s {control_state:12s} speed={speed:.2f} '
                          f'gear={"N" if direction==0 else ("D" if direction==1 else "R")} steer={steer:+.3f} '
                          f'throttle={throttle:.3f} brake={brake:.3f} '
                          f'error={errors[0]:+.3f}m', flush=True)
                # Retry evaluation only under an already latched stationary brake,
                # never while the last command is accelerating the vehicle.
                if not evaluation and throttle == 0 and brake >= 0.5 and speed < 0.025 and now-last_eval >= 1:
                    evaluation = bool(SoInitEvaluationServiceWithLocalData('Ego', True))
                    last_eval = time.monotonic()
                    print(f'Evaluation retry={evaluation}', flush=True)
                time.sleep(0.005)
    except (Exception, KeyboardInterrupt) as exc:
        failure = exc
        print(f'APA STOP: {exc}; phase={phase}; last GPS timestamp={last_stamp}', flush=True)
        traceback.print_exc()
    finally:
        if connected:
            if failure is not None and takeover_requested:
                try:
                    # Latch brake in API until vehicle stops (bounded wait), then disconnect.
                    deadline = time.monotonic()+3
                    while time.monotonic() < deadline and SoGetCaseRunStatus() == 2:
                        g = SimOne_Data_Gps()
                        valid_gps = SoGetGps('Ego', g)
                        SoSetDrive('Ego', command(brake=1.0, direction=0,
                                                 stamp=g.timestamp if valid_gps else (last_stamp or 0),
                                                 frame=g.frame if valid_gps else 0))
                        if valid_gps and math.hypot(g.velX, g.velY) < 0.025:
                            break
                        time.sleep(0.02)
                except Exception as brake_exc:
                    print(f'Brake cleanup: {brake_exc}', flush=True)
            elif failure is not None:
                print('Stopped before API takeover: no brake/control command sent to the case.', flush=True)
            if evaluation:
                try:
                    evaluation_result = finalize_evaluation(
                        SoSaveEvaluationRecord, SoGetCaseRunStatus, evaluation_path,
                        stopped=platform_stopped)
                except Exception as save_exc:
                    print(f'Evaluation save error: {save_exc}', flush=True)
            try:
                SoTerminateSimOneAPI()
            except Exception as end_exc:
                print(f'Terminate error: {end_exc}', flush=True)
        if stop_requested(stop_request, os.getpid()):
            stop_request.unlink()
        if lock.exists() and lock.read_text(encoding='ascii') == str(os.getpid()):
            lock.unlink()
        if dll_scope is not None:
            dll_scope.close()
        os.chdir(original_directory)
        print(f'Log retained: {log_path}', flush=True)
    if failure is not None:
        raise SystemExit(1)
    if (controller is None or finish_distance > 5.2
            or (controller.segment < len(controller.segments)-1 and not controller.complete)):
        print('Case stopped before parking/exit completed. Inspect CSV and platform failure details.', flush=True)
        raise SystemExit(2)
    if not (evaluation_result['saved'] and evaluation_result['verified']
            and evaluation_result['end_reason'] == 1):
        print('Parking + exit phases completed, but evaluation Goal is UNCONFIRMED. '
              'Exit code 3 means evaluation cleanup needs checking, NOT a simulator score. '
              'Keep this log and use the platform report.', flush=True)
        raise SystemExit(3)
    print('Parking + exit phases completed; SDK evaluation endreason=1 (GOAL). '
          'This is NOT a full-marks guarantee; verify actual score in the platform report.', flush=True)


def self_test_evaluation():
    """Mock SDK, filesystem and wall clock; no DLL imports or real waits/writes."""
    from unittest.mock import Mock

    class OfflineClock:
        def __init__(self):
            self.now = 0.0

        def time(self):
            return self.now

        def sleep(self, seconds):
            assert 0 < seconds <= 0.05
            self.now += seconds

    def check(payload, *, stopped=True, status=1, save_ok=True, read_error=None):
        timer, messages, save_times = OfflineClock(), [], []
        path = Mock()
        path.read_text.return_value = json.dumps(payload)
        path.read_text.side_effect = read_error
        save = Mock(side_effect=lambda: save_times.append(timer.now) or save_ok)
        result = finalize_evaluation(save, lambda: status, path, stopped=stopped,
                                     clock=timer.time, sleep=timer.sleep, report=messages.append)
        return result, timer.now, save_times, path, messages

    result, elapsed, times, _, _ = check({'endreason': 1, 'records': []})
    assert result == {'saved': True, 'end_reason': 1, 'verified': True}
    assert len(times) == 1 and times[0] >= EVALUATION_END_GRACE_SECONDS
    for payload in ({'endreason': 0, 'records': []}, {}, {'endreason': True},
                    {'endreason': '1'}, {'endreason': -1}, []):
        result, elapsed, times, _, messages = check(payload)
        assert result['saved'] and not result['verified'], 'Invented evaluation success'
        assert len(times) == 1
        assert elapsed <= EVALUATION_END_GRACE_SECONDS+EVALUATION_VERIFY_SECONDS+1e-8
        assert any('WARNING' in message for message in messages)
    result, _, _, _, _ = check({'endreason': 8, 'records': [{'category': 'judge'}]})
    assert result['verified'] and result['end_reason'] == 8, 'Failure reason was rewritten'
    result, elapsed, times, path, _ = check({}, status=2)
    assert not result['saved'] and not times and elapsed == 0
    path.read_text.assert_not_called()
    result, _, times, path, _ = check({}, save_ok=False)
    assert not result['saved'] and len(times) == 1
    path.read_text.assert_not_called()
    result, elapsed, _, _, _ = check({}, read_error=FileNotFoundError('mock missing record'))
    assert not result['verified'] and elapsed <= 2.75+1e-8
    result, elapsed, times, _, _ = check({'endreason': 3}, stopped=False)
    assert result['end_reason'] == 3 and times == [0.0] and elapsed == 0
    timer = OfflineClock()
    path = Mock()
    path.read_text.side_effect = ['{"endreason":', '{"endreason": 1, "records": []}']
    result = finalize_evaluation(lambda: True, lambda: 1, path, stopped=True,
                                 clock=timer.time, sleep=timer.sleep, report=lambda _: None)
    assert result['verified'] and result['end_reason'] == 1
    for path in (None, Mock(read_text=Mock(side_effect=PermissionError('mock denied')))):
        result = finalize_evaluation(lambda: True, lambda: 1, path,
                                     clock=timer.time, sleep=timer.sleep, report=lambda _: None)
        assert result['saved'] and not result['verified']
    try:
        evaluation_record_path(Path(__file__).parent, '../../other-task')
    except ValueError:
        pass
    else:
        raise AssertionError('Invalid task ID accepted as a record path')
    print('PASS offline evaluation: bounded end-event grace, one SDK save, real reason '
          'preserved, missing/invalid data rejected, no SDK/file writes', flush=True)


def self_test(moves):
    """Offline closed-loop bicycle + lagged steering/pedal plant, no SimOne connection."""
    from types import SimpleNamespace
    self_test_evaluation()
    reference_box = footprint((-20.0, -4.5, 0.0))
    for offset in (0.0, 1.4, 3.1):
        for yaw in (0.0, PI/2, -0.7):
            rear_x, rear_y = -20.0, -4.5
            c, s = math.cos(yaw), math.sin(yaw)
            gps = SimpleNamespace(posX=rear_x+offset*c,
                                  posY=rear_y+offset*s, oriZ=yaw)
            values = []
            for along in (3.1, 0.0):
                for side in (1, -1):
                    values.extend((rear_x+along*c-side*0.9*s,
                                   rear_y+along*s+side*0.9*c))
            measured = measured_geometry(gps, values)
            assert measured is not None and abs(measured[0]-3.1) < 1e-6
            assert abs(measured[1]-offset) < 1e-6, 'GPS origin calibration failed'
            assert abs(gps.posX-measured[1]*c-rear_x) < 1e-6
            assert abs(gps.posY-measured[1]*s-rear_y) < 1e-6
    # Changing the SDK origin is a coordinate conversion, not a body change.
    assert footprint((-20.0, -4.5, 0.0)) == reference_box
    print('PASS offline calibration: rear/mid/front GPS origins, three headings', flush=True)
    start = (START_GPS[0]-GPS_TO_REAR*math.cos(START_GPS[2]),
             START_GPS[1]-GPS_TO_REAR*math.sin(START_GPS[2]), START_GPS[2])
    startup = ParkingController(start, moves)
    startup.shift_until = 0.8
    for tm in (0.0, 0.4, 0.79):
        _, throttle, brake, direction, _ = startup.update(start, 0.0, tm)
        assert direction == 1 and throttle == 0 and brake >= 0.5, 'Unsafe initial N -> D shift'
    _, throttle, brake, direction, _ = startup.update(start, 0.0, 0.81)
    assert direction == 1 and throttle > 0 and brake == 0, 'Initial brake never releases'
    print('PASS offline startup: D settles under brake, then throttle replaces brake', flush=True)
    # Test dynamics variations rather than just comparing the path with itself.
    cases = [(steering, braking, initial_speed)
             for initial_speed in (0.0, 6.944444444444445)
             for steering, braking in ((1.0, 1.0), (0.93, 0.85), (1.07, 1.1))]
    for steering_scale, brake_scale, initial_speed in cases:
        controller = ParkingController(start, moves)
        if initial_speed == 0:
            controller.shift_until = 0.8
        pose, speed, delta, tm = start, initial_speed, 0.0, 0.0
        motion_direction = 1
        hold_total = 0.0
        dt = 0.02
        while tm < 118 and not controller.complete:
            steer, throttle, brake, direction, _ = controller.update(pose, speed, tm)
            delta += (steering_scale*(-steer*FULL_STEER_RAD)-delta)*min(1, dt/0.12)
            acceleration = 2.8*throttle-4.5*brake_scale*brake-0.32*speed
            speed = max(0, speed+acceleration*dt)
            # Neutral removes propulsion, not the inertia of a moving car.
            if direction:
                motion_direction = direction
            pose = integrate(pose, motion_direction*speed*dt, math.tan(delta)/WHEELBASE)
            tm += dt
            if controller.name == 'HOLD_12S' and speed < 0.025 and in_bay(pose):
                hold_total += dt
            if not safe(pose, 0.0):
                raise AssertionError(f'Collision/boundary in plant test at {tm:.2f}s')
        if not controller.complete or hold_total < PARK_SECONDS-0.1:
            raise AssertionError(f'Incomplete test: {tm:.1f}s state={controller.name} hold={hold_total:.2f}')
        print(f'PASS offline plant: steering={steering_scale}, braking={brake_scale}, '
              f'initial_speed={initial_speed:.2f}m/s, elapsed={tm:.2f}s, '
              f'valid bay stop={hold_total:.2f}s', flush=True)
    print('Offline checks passed. This is NOT a simulator score or a full-marks guarantee.', flush=True)


def run_logged(offline=False):
    directory = Path(__file__).resolve().parent
    stem = time.strftime('ParkingAPA-%Y%m%d-%H%M%S')+f'-{os.getpid()}-{time.time_ns()%1000000:06d}'
    diagnostic = directory/(stem+'.log')
    with diagnostic.open('x', encoding='utf-8') as log:
        with contextlib.redirect_stdout(DiagnosticStream(sys.stdout, log)), \
                contextlib.redirect_stderr(DiagnosticStream(sys.stderr, log)):
            print(f'APA session PID={os.getpid()}; diagnostic log: {diagnostic}', flush=True)
            print(f'Mode={"OFFLINE" if offline else "LIVE"}; Python={sys.executable}', flush=True)
            try:
                _, moves = plan_maneuver()
                if offline:
                    self_test(moves)
                else:
                    run_live(moves, directory/(stem+'.csv'))
            except SystemExit as exc:
                print(f'APA exit code={exc.code}', flush=True)
                raise
            except (Exception, KeyboardInterrupt):
                traceback.print_exc()
                print('APA startup/runtime failed. Full Python traceback retained above.', flush=True)
                raise SystemExit(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--self-test', action='store_true', help='Offline only, never connect to SimOne')
    parser.add_argument('--stop', action='store_true', help='Request graceful APA stop; never kill other Python processes')
    args = parser.parse_args()
    if args.stop:
        directory = Path(__file__).resolve().parent
        lock = directory/'.ParkingAPA.lock'
        if lock.exists() and pid_alive(pid := int(lock.read_text(encoding='ascii').strip())):
            (directory/'.ParkingAPA.stop').write_text(str(pid), encoding='ascii')
            print(f'APA stop requested for PID {pid}; brake only if API takeover was attempted, '
                  'then save evaluation if initialized and disconnect. '
                  'A blocked SDK connection may delay stopping.')
        else:
            print('No active ParkingAPA process.')
        return
    run_logged(offline=args.self_test)


if __name__ == '__main__':
    main()
