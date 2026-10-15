# -*- coding: utf-8 -*-
"""Independent controllers for seventeen exported NEVC automatic-driving cases.

Uses the exported straight route plus live GPS/GroundTruth for LKA + ACC + AEB.
Moving starts enter normal risk-based control; there is no mandatory startup stop.
Cruise speed is kept when clearance permits, including with a faster lead vehicle.
Cut-ins use projected entry clearance and progressive gap recovery. Small gaps
alone do not latch full braking behind a vehicle already pulling away.
Only this process sends Ego's final control. No actor teleporting, scenario edits,
synthetic judgement records or automatic uploads. A score requires a live test.
--self-test is offline: no SDK imports, connection, control commands or files.
--stop requests a PID-specific graceful stop; it never kills other programs.
Evaluation cleanup reuses ParkingAPA's pure helpers, NOT its driving algorithm.
Case 13 has separate geometry/path/state logic; case 24's tested algorithm is
unchanged. Case 13 brakes first and only reverses if a forward bypass cannot fit.
Case 10 has its own route, live-light adapter and stop-line approach controller.
Cases 20/32 use the actual exported close-line/10-second standstill ending.
Cases 21/22/26/30/33/37 have separate live-actor risk and route logic.
"""

import argparse
import contextlib
import csv
import ctypes
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback


START = (-450.0000350756041, -17.96411720326303)
GOAL = (-200.000039, -17.311375698781752)
ROAD_HEADING = 0.002610960124348275
ROAD_C, ROAD_S = math.cos(ROAD_HEADING), math.sin(ROAD_HEADING)
ROUTE_LENGTH = math.hypot(GOAL[0]-START[0], GOAL[1]-START[1])
CRUISE_MPS = 25.0/3.6
WHEELBASE_M = 2.9187  # AionV fallback measured in this installation's APA run.
FULL_STEER_RAD = 7*math.pi/36
EGO_WIDTH_M = 2.21
GPS_TO_FRONT_M = 4.5  # Conservative rear-GPS to front extent; not a score value.
MIN_GAP_M = 14.0      # Buffer above this case's 10 m failure condition.
FOLLOW_REACTION_S = 1.0  # Algorithm reserve, not a scenario/scoring parameter.
FOLLOW_DECEL_MPS2 = 2.0
CUTIN_LOOKAHEAD_S = 6.0  # Live lateral motion, not a scenario-specific timer.
GAP_RECOVERY_S = 4.0
MAX_RECOVERY_DELTA_MPS = 2.0  # Limit routine gap recovery, never emergency braking.
BRAKE_RISE_PER_S = 1.2
EMERGENCY_RESPONSE_S = 0.4
COLLISION_BUFFER_M = 1.5
LOOP_WALL_S = 0.02
VEHICLE_TYPES = {6, 18, 19, 20}


def clip(value, low, high):
    return max(low, min(high, value))


def wrap(value):
    return (value+math.pi) % (2*math.pi)-math.pi


def road_coordinates(x, y):
    dx, dy = x-START[0], y-START[1]
    return dx*ROAD_C+dy*ROAD_S, -dx*ROAD_S+dy*ROAD_C


@dataclass(frozen=True)
class EgoState:
    x: float
    y: float
    yaw: float
    speed: float
    stamp: int = 0
    frame: int = 0


@dataclass(frozen=True)
class Obstacle:
    ident: int
    kind: int
    x: float
    y: float
    vx: float
    vy: float
    yaw: float
    length: float
    width: float


@dataclass(frozen=True)
class Lead:
    ident: int
    gap: float
    speed: float
    predicted: bool
    entry_s: float = 0.0

    def projected_gap(self, ego_speed):
        # Constant-velocity estimate until first corridor entry; recomputed from
        # fresh observations each frame. It is NOT the platform's DHW value.
        return self.gap+(self.speed-ego_speed)*self.entry_s


@dataclass(frozen=True)
class Command:
    steering: float
    throttle: float
    brake: float
    target_speed: float
    mode: str
    lateral: float
    distance: float
    lead: Lead | None = None
    ttc: float = float('inf')


def startup_pose_report(ego):
    """Explain the existing start-pose guard without changing its thresholds."""
    dx, dy = ego.x-START[0], ego.y-START[1]
    distance = math.hypot(dx, dy)
    yaw_error = wrap(ego.yaw-ROAD_HEADING)
    along, lateral = road_coordinates(ego.x, ego.y)
    reasons = []
    if distance > 8:
        reasons.append('position_offset')
    if abs(yaw_error) > 0.15:
        reasons.append('heading_offset')
    report = (
        f'START_POSE: frame={ego.frame}; timestamp_ms={ego.stamp}; '
        f'speed={ego.speed:.3f}m/s ({ego.speed*3.6:.3f}km/h)\n'
        f'  ACTUAL: x={ego.x:.6f}m; y={ego.y:.6f}m; '
        f'yaw={ego.yaw:.6f}rad ({math.degrees(ego.yaw):.6f}deg)\n'
        f'  EXPECTED: x={START[0]:.6f}m; y={START[1]:.6f}m; '
        f'yaw={ROAD_HEADING:.6f}rad ({math.degrees(ROAD_HEADING):.6f}deg)\n'
        f'  DEVIATION: dx={dx:+.6f}m; dy={dy:+.6f}m; '
        f'distance={distance:.6f}m; along={along:+.6f}m; lateral={lateral:+.6f}m; '
        f'yaw_error={yaw_error:+.6f}rad ({math.degrees(yaw_error):+.6f}deg)\n'
        f'  CHECK: distance_limit=8.000m; abs_yaw_error_limit=0.150000rad; '
        f'result={"REJECT" if reasons else "ACCEPT"}; '
        f'reasons={",".join(reasons) if reasons else "none"}'
    )
    return report, tuple(reasons)


def find_self_id(ego, obstacles):
    """Learn Ego's GT ID only from a unique near-collinear startup candidate."""
    candidates = []
    for obj in obstacles:
        dx, dy = obj.x-ego.x, obj.y-ego.y
        forward, lateral = dx*ROAD_C+dy*ROAD_S, -dx*ROAD_S+dy*ROAD_C
        if (obj.kind in VEHICLE_TYPES and -4.5 < forward < 4.5
                and abs(lateral) < 0.6 and abs(wrap(obj.yaw-ego.yaw)) < 0.15
                and abs(math.hypot(obj.vx, obj.vy)-ego.speed) < 0.8):
            candidates.append(obj.ident)
    return candidates[0] if len(candidates) == 1 else None


def candidate_leads(ego, obstacles, self_id=None):
    """Vehicles overlapping, or predicted to enter, the route corridor.

    Side-lane vehicles are not blindly treated as front obstacles. Their lateral
    velocity is used to anticipate a cut-in; no fixed wait or actor speed script.
    """
    ego_s, _ = road_coordinates(ego.x, ego.y)
    leads = []
    for obj in obstacles:
        if obj.ident == self_id or obj.kind not in VEHICLE_TYPES:
            continue
        obj_s, lateral = road_coordinates(obj.x, obj.y)
        forward = obj_s-ego_s
        if not 0 < forward < 100:
            continue
        yaw_error = wrap(obj.yaw-ROAD_HEADING)
        along_extent = (abs(math.cos(yaw_error))*obj.length
                        + abs(math.sin(yaw_error))*obj.width)/2
        lateral_extent = (abs(math.sin(yaw_error))*obj.length
                          + abs(math.cos(yaw_error))*obj.width)/2
        corridor = EGO_WIDTH_M/2+lateral_extent+0.35
        vy = -obj.vx*ROAD_S+obj.vy*ROAD_C
        inside = abs(lateral) <= corridor
        entry_s = 0.0 if inside else float('inf')
        if not inside and lateral*vy < 0:
            entry_s = (abs(lateral)-corridor)/abs(vy)
        predicted = not inside and entry_s <= CUTIN_LOOKAHEAD_S
        if not inside and not predicted:
            continue
        gap = forward-GPS_TO_FRONT_M-along_extent
        speed = max(0.0, obj.vx*ROAD_C+obj.vy*ROAD_S)
        leads.append(Lead(obj.ident, gap, speed, predicted, entry_s))
    return leads


def select_lead(ego, obstacles, self_id=None):
    """Geometry-only nearest lead; control checks ALL candidate vehicles."""
    return min(candidate_leads(ego, obstacles, self_id),
               key=lambda lead: lead.gap, default=None)


def lead_speed_plan(ego_speed, lead):
    """Separate advance gap planning from immediate emergency collision risk."""
    closing = max(0.0, ego_speed-lead.speed)
    gap = lead.projected_gap(ego_speed)
    ttc = max(0.0, lead.gap)/closing if closing > 0.05 else float('inf')
    braking_reserve = (EMERGENCY_RESPONSE_S*closing
                       + max(0.0, ego_speed**2-lead.speed**2)/(2*4.0))
    if lead.predicted:
        # A beside-us vehicle must not cause a distance-only full stop while it
        # still has time before entering. Routine control builds clearance now;
        # emergency braking remains available for imminent overlapping paths.
        emergency = (lead.entry_s <= EMERGENCY_RESPONSE_S
                     and (gap < COLLISION_BUFFER_M
                          or (closing > 0.1
                              and gap < COLLISION_BUFFER_M+braking_reserve)))
    else:
        # Keep the existing 11.5 m closing-risk reserve. Release braking when
        # the lead pulls away, instead of holding Ego at zero until gap > 11.5.
        emergency = (lead.gap < COLLISION_BUFFER_M
                     or (closing > 0.1 and lead.gap < 11.5+braking_reserve))
    if emergency:
        return 0.0, 'CUTIN_AEB' if lead.predicted else 'AEB', ttc, True
    if gap < MIN_GAP_M:
        # Matching a cut-in's speed cannot restore a small gap. Temporarily
        # travel a little slower, then naturally return to cruise as gap grows.
        # There is no artificial minimum speed: stopped/slower obstacles and
        # actual danger may still require stopping regardless of scoring.
        recovery = min(MAX_RECOVERY_DELTA_MPS, (MIN_GAP_M-gap)/GAP_RECOVERY_S)
        target = clip(lead.speed-recovery, 0.0, CRUISE_MPS)
    else:
        reaction_term = FOLLOW_DECEL_MPS2*FOLLOW_REACTION_S
        safe_closing = (math.sqrt(reaction_term**2
                                 + 2*FOLLOW_DECEL_MPS2*(gap-MIN_GAP_M))
                        - reaction_term)
        target = clip(lead.speed+safe_closing, 0.0, CRUISE_MPS)
    mode = 'CUTIN_PREDICT' if lead.predicted else (
        'FOLLOW' if target == CRUISE_MPS else 'FOLLOW_GAP')
    return target, mode, ttc, False


class CutInController:
    def __init__(self):
        self.integral = 0.0
        self.previous_steer = 0.0
        self.previous_brake = 0.0
        self.previous_lead = None

    def update(self, ego, obstacles, dt, self_id=None):
        values = (ego.x, ego.y, ego.yaw, ego.speed, dt)
        if not all(math.isfinite(value) for value in values) or ego.speed < 0:
            raise ValueError('Invalid ego state')
        dt = clip(dt, 0.005, 0.15)
        along, lateral = road_coordinates(ego.x, ego.y)
        distance = math.hypot(GOAL[0]-ego.x, GOAL[1]-ego.y)
        if abs(lateral) > 1.2 or abs(wrap(ego.yaw-ROAD_HEADING)) > 0.4:
            return Command(0, 0, 1, 0, 'LANE_FAILSAFE', lateral, distance)
        lookahead = clip(3.5+0.35*ego.speed, 4.0, 7.0)
        target_s = min(ROUTE_LENGTH+8, along+lookahead)
        tx, ty = START[0]+target_s*ROAD_C, START[1]+target_s*ROAD_S
        alpha = wrap(math.atan2(ty-ego.y, tx-ego.x)-ego.yaw)
        wheel_angle = math.atan2(2*WHEELBASE_M*math.sin(alpha),
                                 max(1.0, math.hypot(tx-ego.x, ty-ego.y)))
        steer = clip(-wheel_angle/FULL_STEER_RAD, -0.45, 0.45)
        steer = clip(steer, self.previous_steer-1.2*dt, self.previous_steer+1.2*dt)
        self.previous_steer = steer
        lead = None
        target_speed, mode, ttc = CRUISE_MPS, 'CRUISE', float('inf')
        for candidate in candidate_leads(ego, obstacles, self_id):
            planned_speed, planned_mode, planned_ttc, emergency = lead_speed_plan(ego.speed, candidate)
            if emergency:
                self.integral = 0
                self.previous_brake = 1.0
                self.previous_lead = candidate.ident
                return Command(steer, 0, 1, 0, planned_mode, lateral, distance, candidate, planned_ttc)
            if lead is None or planned_speed < target_speed:
                lead, target_speed, mode, ttc = candidate, planned_speed, planned_mode, planned_ttc
        lead_id = lead.ident if lead else None
        if lead_id != self.previous_lead:
            self.integral = 0.0  # Do not carry cruise wind-up into a new cut-in.
            self.previous_lead = lead_id
        # The platform completes this case within its 5 m goal radius; do not
        # slow early for a parking-style approach. Retain an endpoint backstop
        # only after reaching the last metre if platform termination is delayed.
        if along >= ROUTE_LENGTH-1.0:
            self.integral = 0
            return Command(steer, 0, 0.7, 0, 'GOAL_HOLD', lateral, distance, lead, ttc)
        if target_speed <= 0.05:
            self.integral = 0
            self.previous_brake = 0.65
            return Command(steer, 0, 0.65, 0, mode, lateral, distance, lead, ttc)
        error = target_speed-ego.speed
        self.integral = clip(self.integral+error*dt, -4, 4)
        if ego.speed > target_speed+0.3:
            self.integral = min(0, self.integral)
            throttle, brake = 0, clip(0.10+0.16*(ego.speed-target_speed), 0.08, 0.7)
        else:
            throttle = clip(0.12+0.18*error+0.025*self.integral, 0, 0.55)
            brake = 0
        # Only routine braking rises progressively. Emergency commands above
        # bypass this limiter; releasing the brake is immediate, not delayed.
        brake = min(brake, self.previous_brake+BRAKE_RISE_PER_S*dt)
        self.previous_brake = brake
        return Command(steer, throttle, brake, target_speed, mode,
                       lateral, distance, lead, ttc)


# Case 13 data come from db54b282-f47d-4cbe-8b96-ec0787b79a00.sim.
# Never assign these values to the case 24 constants above.
CASE13_START = (-269.8987451961563, -17.493878848486037)
CASE13_GOAL = (-192.00267, -13.79048290897974)
CASE13_HEADING = 0.002610960124355431
CASE13_C, CASE13_S = math.cos(CASE13_HEADING), math.sin(CASE13_HEADING)
CASE13_SOLID_TYPES = VEHICLE_TYPES | {4, 5, 7, 8, 9, 17, 21, 22, 23, 27, 29}
CASE13_MAX_REVERSE_M = 16.0  # Bounded algorithm reserve, not a scoring edit.
CASE13_STAGING_RESERVE_M = 0.5
CASE13_LEFT_SIGNAL = 1 << 1  # SDK ESimOne_Signal_Light_LeftBlinker.
CASE13_BRAKE_SIGNAL = 1 << 3
CASE13_REVERSE_SIGNAL = 1 << 6
CASE13_FIXED = (
    Obstacle(-13001, 7, -260.6, -16.112919837761524, 0, 0, 0, 1.2, 0.5),
    Obstacle(-13002, 7, -260.65, -17.457198195324235, 0, 0, 0, 1.2, 0.5),
    Obstacle(-13003, 7, -260.6, -18.757731394977853, 0, 0, 0, 1.2, 0.5),
)


def case13_coordinates(x, y):
    dx, dy = x-CASE13_START[0], y-CASE13_START[1]
    return dx*CASE13_C+dy*CASE13_S, -dx*CASE13_S+dy*CASE13_C


def case13_world(along, lateral):
    return (CASE13_START[0]+along*CASE13_C-lateral*CASE13_S,
            CASE13_START[1]+along*CASE13_S+lateral*CASE13_C)


def supported_case(name):
    if name.startswith(('24.', '24_', '24-', '24 ')) and '切入' in name:
        return 24
    if name.startswith(('13.', '13_', '13-', '13 ')) and '前方障碍物起步' in name:
        return 13
    if name.startswith(('10.', '10_', '10-', '10 ')) and '机动车信号灯识别及响应' in name:
        return 10
    if name.startswith(('35.', '35_', '35-', '35 ')) and '行人和非机动车通行' in name:
        return 35
    if name.startswith(('18.', '18_', '18-', '18 ')) and '无信号灯路口车辆冲突通行' in name:
        return 18
    for number, title in ((15, '弯道内跟车'), (17, '避让低速行驶车辆变道'), (31, '路口车辆冲突通行')):
        if name.startswith(tuple(f'{number}{sep}' for sep in ('.', '_', '-', ' '))) and title in name:
            return number
    for number,title in EXPANDED_TITLES.items():
        if name.startswith(tuple(f'{number}{sep}' for sep in ('.','_','-',' '))) and title in name:
            return number
    if name.startswith(('40.','40_','40-','40 ')) and '冲突对象突然出现' in name:
        return 40
    raise RuntimeError(f'Wrong case: {name}; supported exported cases: 10,13,15,17,18,20,21,22,24,26,30,31,32,33,35,37,40')


def case13_pose_report(ego):
    along, lateral = case13_coordinates(ego.x, ego.y)
    distance = math.hypot(along, lateral)
    yaw_error = wrap(ego.yaw-CASE13_HEADING)
    reasons = []
    # A late takeover near these close obstacles is unsafe, unlike case 24.
    if distance > 2.0:
        reasons.append('position_offset_or_late_start')
    if abs(yaw_error) > 0.15:
        reasons.append('heading_offset')
    return (f'START_POSE case=13: frame={ego.frame}; speed={ego.speed:.3f}m/s; '
            f'actual=({ego.x:.6f},{ego.y:.6f},{ego.yaw:.6f}); '
            f'expected=({CASE13_START[0]:.6f},{CASE13_START[1]:.6f},{CASE13_HEADING:.6f}); '
            f'distance={distance:.6f}m; lateral={lateral:.6f}m; yaw_error={yaw_error:.6f}rad; '
            f'result={"REJECT" if reasons else "ACCEPT"}; reasons={",".join(reasons) or "none"}',
            tuple(reasons))


@dataclass(frozen=True)
class Case13Command(Command):
    gear: int = 1  # SDK: 0=N, 1=D, 2=R. Case 24 still sends its original D.
    signal_lights: int = 0


class Case13SignalWriter:
    """Case-13-only adapter; injectable sender/structure for offline testing."""

    def __init__(self, sender, data_type):
        self.sender, self.data_type = sender, data_type
        self.last_flags = self.last_stamp = None

    @classmethod
    def from_sdk(cls):
        from SimOnePNCAPI import SoSetSignalLights
        from SimOneIOStruct import SimOne_Data_Signal_Lights
        return cls(SoSetSignalLights, SimOne_Data_Signal_Lights)

    def apply(self, ego, command):
        flags = command.signal_lights
        if flags == self.last_flags and self.last_stamp is not None and ego.stamp-self.last_stamp < 500:
            return
        data = self.data_type()
        data.timestamp, data.frame, data.signalLights = ego.stamp, ego.frame, flags
        if not self.sender('Ego', data):
            raise RuntimeError('Case 13 SetSignalLights rejected; no un-signalled lane-change throttle sent')
        if flags != self.last_flags:
            print(f'CASE13_SIGNALS: frame={ego.frame}; flags={flags}; '
                  f'left={bool(flags & CASE13_LEFT_SIGNAL)}; accepted=True.', flush=True)
        self.last_flags, self.last_stamp = flags, ego.stamp


class ObstacleStartController:
    """Position-feedback bypass, polygon collision checks and stopped shifts.

    Known static geometry is used even if GT omits static props. Live solids are
    ALSO checked, so unexpected cars/pedestrians cannot be ignored. No timing of
    actors, teleporting, removal of obstacles or manufactured scoring records.
    """

    def __init__(self):
        from ParkingAPA import rectangle, overlap, integrate
        self.rectangle, self.overlap, self.integrate = rectangle, overlap, integrate
        self.wheelbase, self.gps_to_rear = 3.1, 3.1  # Export fallback, APA convention.
        self.measured = False
        self.phase = 'INITIAL_BRAKE'
        self.still_s = self.shift_s = 0.0
        self.reverse_target = None
        self.reverse_origin_s = None
        self.reverse_replans = 0
        self.reverse_lateral = 0.0
        self.signal_wait_s = 0.0
        self.last_plan_diagnostics = []
        self.points = []
        self.index = 0
        self.integral = self.previous_steer = 0.0
        self.geometry_reader = None

    def register_geometry_reader(self):
        # Lazy SDK access, ONLY after case 13 has taken control and applied the
        # initial safety brake. Offline tests and case 24 never execute this.
        from SimOneIOStruct import ESimOne_Data_Vehicle_State as State
        from SimOneIOStruct import SimOne_Data_Vehicle_Extra
        from SimOnePNCAPI import SoRegisterVehicleState, SoGetVehicleState
        states = (State*8)(State.ESimOne_Data_Vehicle_State_SO_X_L1,
                           State.ESimOne_Data_Vehicle_State_SO_Y_L1,
                           State.ESimOne_Data_Vehicle_State_SO_X_R1,
                           State.ESimOne_Data_Vehicle_State_SO_Y_R1,
                           State.ESimOne_Data_Vehicle_State_SO_X_L2,
                           State.ESimOne_Data_Vehicle_State_SO_Y_L2,
                           State.ESimOne_Data_Vehicle_State_SO_X_R2,
                           State.ESimOne_Data_Vehicle_State_SO_Y_R2)
        registered = bool(SoRegisterVehicleState('Ego', states, len(states)))
        print(f'Case 13 wheel-state registration={registered}; fixed static map remains active.', flush=True)
        if registered:
            self.geometry_reader = (SoGetVehicleState, SimOne_Data_Vehicle_Extra)

    def observe_geometry(self, gps):
        if self.measured or self.phase != 'INITIAL_BRAKE' or not self.geometry_reader:
            return
        if math.hypot(gps.velX, gps.velY) > 0.04 or self.still_s < 0.2:
            return
        from ParkingAPA import measured_geometry
        read, extra_type = self.geometry_reader
        extra = extra_type()
        if read('Ego', extra) and extra.dataSize >= 8:
            measured = measured_geometry(gps, extra.extra_states[:8])
            if measured is not None:
                self.wheelbase, self.gps_to_rear = measured
                self.measured = True
                print(f'Case 13 measured geometry: wheelbase={self.wheelbase:.4f}m; '
                      f'GPS_to_rear={self.gps_to_rear:.4f}m.', flush=True)

    def rear_pose(self, ego):
        return (ego.x-self.gps_to_rear*math.cos(ego.yaw),
                ego.y-self.gps_to_rear*math.sin(ego.yaw), ego.yaw)

    def body(self, pose, margin=0.1):
        # Fixed body in rear-axle coordinates; GPS calibration does not change
        # the actual car's physical dimensions or body-to-axle placement.
        return self.rectangle(*pose, 4.57+2*margin, 2.21+2*margin, 1.7)

    def hazards(self, obstacles, self_id):
        return list(CASE13_FIXED)+[o for o in obstacles if o.ident != self_id]

    def safety_issue(self, pose, obstacles, horizon=0.0, margin=0.1):
        body = self.body(pose, margin)
        # Map has two 3.5 m lanes, broken centre divider and solid outer edges.
        # Relative to Ego's original lane centre: lower=-1.75, upper=5.25.
        if any(not -1.63 < case13_coordinates(x, y)[1] < 5.13 for x, y in body):
            return 'outer_edge'
        for o in obstacles:
            if self.overlap(body, self.rectangle(o.x+o.vx*horizon, o.y+o.vy*horizon,
                                                o.yaw, o.length, o.width)):
                return f'obstacle:{o.ident}'
        return None

    def safe(self, pose, obstacles, horizon=0.0, margin=0.1):
        return self.safety_issue(pose, obstacles, horizon, margin) is None

    def plan_forward(self, pose, solids=CASE13_FIXED, *, quiet=False):
        start_s, start_l = case13_coordinates(*pose[:2])
        goal_s, goal_l = case13_coordinates(*CASE13_GOAL)
        end_s = goal_s-self.gps_to_rear+2.0
        self.last_plan_diagnostics = []
        # Search spatial lane-change lengths, not a pre-timed steering script.
        for length in (12.0, 14.0, 16.0, 18.0, 20.0, 22.0):
            points = []
            for i in range(math.ceil((end_s-start_s)/0.1)+1):
                s = min(end_s, start_s+i*0.1)
                u = clip((s-start_s)/length, 0.0, 1.0)
                f = 10*u**3-15*u**4+6*u**5
                slope = (goal_l-start_l)*(30*u*u-60*u**3+30*u**4)/length
                second = (goal_l-start_l)*(60*u-180*u*u+120*u**3)/(length*length)
                x, y = case13_world(s, start_l+(goal_l-start_l)*f)
                curvature = second/(1+slope*slope)**1.5
                point = (x, y, CASE13_HEADING+math.atan(slope), curvature)
                issue = ('steering_limit' if abs(math.atan(self.wheelbase*curvature)) > 0.50
                         else self.safety_issue(point[:3], solids))
                if issue:
                    self.last_plan_diagnostics.append((length, round(s, 3), issue))
                    break
                points.append(point)
            else:
                if not quiet:
                    print(f'Case 13 validated forward bypass: lane_shift={length:.1f}m; '
                          f'start_s={start_s:.3f}m; samples={len(points)}.', flush=True)
                return points
        if not quiet:
            print(f'CASE13_PLAN_REJECT: rear_s={start_s:.3f}m; rear_l={start_l:.3f}m; '
                  f'wheelbase={self.wheelbase:.4f}m; GPS_to_rear={self.gps_to_rear:.4f}m; '
                  f'first_failures={self.last_plan_diagnostics}.', flush=True)
        return None

    def choose_staging(self, pose, solids, *, max_reverse=CASE13_MAX_REVERSE_M):
        """Validate reverse sweep AND full forward route before choosing a cusp.

        Fixed 8 m staging failed in the measured run. Search position-relative
        room with a tolerance reserve, without weakening body/road checks.
        """
        start_s, lateral = case13_coordinates(*pose[:2])
        for distance in range(2, math.floor(max_reverse)+1, 2):
            count = math.ceil(distance/0.1)
            for i in range(count+1):
                travel = distance*i/count
                x, y = case13_world(start_s-travel, lateral)
                # Live moving hazards are also checked at estimated reverse
                # arrival time. The continuous guard remains active in motion.
                if not self.safe((x, y, CASE13_HEADING), solids, travel/1.2):
                    break
            else:
                target_s = start_s-distance
                target_pose = (*case13_world(target_s, lateral), CASE13_HEADING)
                points = self.plan_forward(target_pose, solids, quiet=True)
                if points is None:
                    continue
                # The actual stopped cusp can be a few cm ahead of the target.
                # Prove a route also exists half a metre nearer the obstacles.
                reserve_pose = (*case13_world(target_s+CASE13_STAGING_RESERVE_M, lateral),
                                CASE13_HEADING)
                if self.plan_forward(reserve_pose, solids, quiet=True) is None:
                    continue
                print(f'CASE13_STAGING_ACCEPT: reverse={distance:.2f}m; from_s={start_s:.3f}m; '
                      f'target_s={target_s:.3f}m; tolerance={CASE13_STAGING_RESERVE_M:.2f}m; '
                      f'full_forward_route_valid=True.', flush=True)
                return target_s, points
        raise RuntimeError('Case 13: reverse corridor is blocked or no full bypass fits within '
                           f'{max_reverse:.2f}m rearward reserve; no blind reversing')

    def begin_forward(self, points):
        self.points, self.index, self.integral = points, 0, 0.0
        self.phase, self.signal_wait_s, self.shift_s = 'SIGNAL_WAIT', 0.0, 0.0

    def pedals(self, target, speed, dt):
        if target <= 0.04:
            self.integral = 0.0
            return 0.0, 0.65
        error = target-speed
        self.integral = clip(self.integral+error*dt, -0.6, 0.9)
        if speed > target+0.12:
            self.integral = min(0.0, self.integral)
            return 0.0, clip(0.10+0.25*(speed-target), 0.05, 0.85)
        return clip(0.10+0.30*error+0.10*self.integral, 0.0, 0.45), 0.0

    def command(self, ego, steering=0, throttle=0, brake=0.65, target=0,
                gear=None, mode=None, lateral=0):
        if gear is None:
            gear = 2 if self.phase in ('REVERSE', 'SHIFT_REVERSE') else 1
        flags = CASE13_LEFT_SIGNAL if self.phase in ('SIGNAL_WAIT', 'SHIFT_DRIVE', 'BYPASS') else 0
        if brake > 0:
            flags |= CASE13_BRAKE_SIGNAL
        if gear == 2:
            flags |= CASE13_REVERSE_SIGNAL
        return Case13Command(steering, throttle, brake, target, mode or 'OBSTACLE_'+self.phase,
                             lateral, math.hypot(ego.x-CASE13_GOAL[0], ego.y-CASE13_GOAL[1]),
                             gear=gear, signal_lights=flags)

    def update(self, ego, obstacles, dt, self_id=None):
        if not all(math.isfinite(v) for v in (ego.x, ego.y, ego.yaw, ego.speed, dt)) or ego.speed < 0:
            raise ValueError('Invalid case 13 ego state')
        dt = clip(dt, 0.005, 0.15)
        pose = self.rear_pose(ego)
        solids = self.hazards(obstacles, self_id)
        if not self.safe(pose, solids, margin=0.02):
            return self.command(ego, brake=1, mode='OBSTACLE_COLLISION_OR_EDGE_FAILSAFE')
        rear_s, rear_l = case13_coordinates(*pose[:2])
        if self.phase == 'INITIAL_BRAKE':
            self.still_s = self.still_s+dt if ego.speed < 0.04 else 0.0
            if self.still_s < 0.5:
                return self.command(ego, brake=1)
            if not self.measured:
                print('Case 13: wheel geometry unavailable; using exported front-reference '
                      'fallback (3.1m). This assumption needs live verification.', flush=True)
            self.points = self.plan_forward(pose, solids) or []
            self.integral = 0.0
            if self.points:
                self.begin_forward(self.points)
            else:
                print('CASE13_SOLIDS: '+str([(o.ident, o.kind, round(o.x, 4), round(o.y, 4),
                                              round(o.yaw, 4), round(o.length, 4), round(o.width, 4))
                                             for o in solids]), flush=True)
                self.reverse_origin_s, self.reverse_lateral = rear_s, rear_l
                self.reverse_target, _ = self.choose_staging(pose, solids)
                self.phase, self.shift_s = 'SHIFT_REVERSE', 0.0
        if self.phase == 'SIGNAL_WAIT':
            self.signal_wait_s = self.signal_wait_s+dt if ego.speed < 0.04 else 0.0
            if self.signal_wait_s >= 1.0:
                self.phase, self.shift_s = 'SHIFT_DRIVE', 0.0
            return self.command(ego, mode='OBSTACLE_SIGNAL_WAIT')
        if self.phase.startswith('SHIFT_'):
            self.shift_s = self.shift_s+dt if ego.speed < 0.04 else 0.0
            gear = 2 if self.phase == 'SHIFT_REVERSE' else 1
            if self.shift_s >= 0.6:
                self.phase = 'REVERSE' if gear == 2 else 'BYPASS'
                self.integral = 0.0
            return self.command(ego, gear=gear, mode='OBSTACLE_GEAR_SETTLE')
        direction, lateral_error = 1, 0.0
        if self.phase == 'REVERSE':
            remaining = rear_s-self.reverse_target
            lateral_error = rear_l-self.reverse_lateral
            yaw_error = wrap(ego.yaw-CASE13_HEADING)
            curvature = -0.50*lateral_error+1.5*yaw_error
            target = min(1.2, math.sqrt(max(0, 1.6*remaining)), max(0, remaining)*1.2)
            if remaining <= 0.07:
                target = 0.0
                if ego.speed < 0.04:
                    self.points = self.plan_forward(pose, solids) or []
                    if not self.points:
                        remaining_reserve = CASE13_MAX_REVERSE_M-(self.reverse_origin_s-rear_s)
                        if self.reverse_replans >= 2 or remaining_reserve < 2:
                            raise RuntimeError('Case 13: bounded staging replans exhausted; inspect CASE13_PLAN_REJECT')
                        self.reverse_target, _ = self.choose_staging(pose, solids,
                                                                   max_reverse=remaining_reserve)
                        self.reverse_replans += 1
                        self.phase, self.shift_s = 'SHIFT_REVERSE', 0.0
                        self.integral = 0.0
                        return self.command(ego, gear=2, mode='OBSTACLE_STAGING_REPLAN')
                    self.begin_forward(self.points)
                    return self.command(ego, gear=2)
            direction = -1
        else:
            lo, hi = max(0, self.index-4), min(len(self.points), self.index+45)
            self.index = min(range(lo, hi), key=lambda i: math.hypot(pose[0]-self.points[i][0],
                                                                    pose[1]-self.points[i][1]))
            point = self.points[self.index]
            lateral_error = -(pose[0]-point[0])*math.sin(point[2])+(pose[1]-point[1])*math.cos(point[2])
            yaw_error = wrap(pose[2]-point[2])
            curvature = point[3]-0.5*lateral_error-1.5*yaw_error
            past_obstacles = rear_s > max(case13_coordinates(o.x, o.y)[0] for o in CASE13_FIXED)+3.0
            _, goal_l = case13_coordinates(*CASE13_GOAL)
            aligned_left = abs(rear_l-goal_l) < 0.15 and abs(wrap(ego.yaw-CASE13_HEADING)) < 0.08
            target = 4.5 if past_obstacles and aligned_left and abs(yaw_error) < 0.15 else 1.6
            self.phase = 'CRUISE' if target == 4.5 else 'BYPASS'
            if abs(lateral_error) > 0.6 or abs(yaw_error) > 0.4:
                return self.command(ego, brake=1, mode='OBSTACLE_TRACK_FAILSAFE', lateral=lateral_error)
            goal_s, _ = case13_coordinates(*CASE13_GOAL)
            if case13_coordinates(ego.x, ego.y)[0] >= goal_s-0.8:
                return self.command(ego, mode='OBSTACLE_GOAL_HOLD')
        steering = clip(-math.atan(self.wheelbase*curvature)/FULL_STEER_RAD, -0.9, 0.9)
        steering = clip(steering, self.previous_steer-1.8*dt, self.previous_steer+1.8*dt)
        self.previous_steer = steering
        actual_curvature = math.tan(-steering*FULL_STEER_RAD)/self.wheelbase
        # Immediate swept-body guard, including linearly predicted live solids.
        check_distance = ego.speed*0.3+ego.speed**2/(2*4.0)+0.3
        for i in range(1, math.ceil(check_distance/0.12)+1):
            distance = min(check_distance, i*0.12)
            future = self.integrate(pose, direction*distance, actual_curvature)
            if not self.safe(future, solids, distance/max(ego.speed, 0.5), margin=0.02):
                self.integral = 0.0
                return self.command(ego, steering, brake=1, gear=1 if direction > 0 else 2,
                                    mode='OBSTACLE_RISK_BRAKE', lateral=lateral_error)
        throttle, brake = self.pedals(target, ego.speed, dt)
        return self.command(ego, steering, throttle, brake, target,
                            1 if direction > 0 else 2, lateral=lateral_error)


# Independent case 10 data, from a932033c-368c-4896-a50a-50c05afee45f.sim.
# Road 290, lane -2 -> straight traffic light 52, stop line 463. Light 51
# belongs to lane -1; selecting a nearest/random lamp would be incorrect.
CASE10_START = (-120.0, 1.97918)
CASE10_ROUTE_START = (-120.0, 2.047364011838772)
CASE10_GOAL = (68.853426, 1.620461)
CASE10_HEADING = math.atan2(CASE10_GOAL[1]-CASE10_ROUTE_START[1],
                            CASE10_GOAL[0]-CASE10_ROUTE_START[0])
CASE10_C, CASE10_S = math.cos(CASE10_HEADING), math.sin(CASE10_HEADING)
CASE10_LENGTH = math.dist(CASE10_ROUTE_START, CASE10_GOAL)
CASE10_LINE_POINT = (-19.42804928631577, 7.0934104779955796)
CASE10_LINE_HEADING = -0.002525247157512519
CASE10_LIGHT_ID = 52
CASE10_BUILD = '20261013-empty-gt-fix'
CASE10_CRUISE_MPS = 30.0/3.6
CASE10_STOP_GAP_M = 0.25  # Desired physical bumper clearance; NOT a score edit.
CASE10_BODY_FRONT_M = 1.7+4.57/2  # Same rear-axle AionV body used in case 13.


def case10_coordinates(x, y):
    dx, dy = x-CASE10_ROUTE_START[0], y-CASE10_ROUTE_START[1]
    return dx*CASE10_C+dy*CASE10_S, -dx*CASE10_S+dy*CASE10_C


def case10_world(along, lateral=0.0):
    return (CASE10_ROUTE_START[0]+along*CASE10_C-lateral*CASE10_S,
            CASE10_ROUTE_START[1]+along*CASE10_S+lateral*CASE10_C)


def case10_pose_report(ego):
    along, lateral = case10_coordinates(ego.x, ego.y)
    yaw_error = wrap(ego.yaw-CASE10_HEADING)
    reasons = []
    # Allow a brief moving-start launch delay, but never late takeover at the
    # traffic-switch trigger. No compulsory startup stop in this moving case.
    if not -8.0 <= along <= 25.0 or abs(lateral) > 0.9:
        reasons.append('position_offset_or_late_start')
    if abs(yaw_error) > 0.15:
        reasons.append('heading_offset')
    return (f'START_POSE case=10: actual=({ego.x:.6f},{ego.y:.6f},{ego.yaw:.6f}); '
            f'speed={ego.speed:.3f}m/s; along={along:.3f}m; lateral={lateral:.3f}m; '
            f'result={"REJECT" if reasons else "ACCEPT"}; reasons={",".join(reasons) or "none"}',
            tuple(reasons))


@dataclass(frozen=True)
class Case10Light:
    status: int = 0
    countdown: int = -1
    valid: bool = False
    layout: str = 'unavailable'

    @property
    def green(self):
        return self.valid and self.status == 2


class Case10LightReader:
    """Read-only, case-10-only adapter; SDK files/prototypes remain untouched.

    This installed header packs bool+int+int+enum (13 bytes), whereas its Python
    wrapper declares four ints (16 bytes). Receive into a generous raw buffer,
    then identify the layout by the EXACT returned OpenDRIVE ID. Never infer a
    permissive light from a misaligned enum or switch phases using a timer.
    """

    def __init__(self, sender):
        self.sender = sender
        self.last_diagnostic = None

    @classmethod
    def from_sdk(cls):
        from SimOneSensorAPI import SoGetTrafficLights
        return cls(SoGetTrafficLights)

    @staticmethod
    def decode(raw, light_id=CASE10_LIGHT_ID):
        candidates = []
        for start, layout in ((1, 'packed13'), (4, 'python16')):
            if len(raw) < start+12:
                continue
            ident, countdown, status = (int.from_bytes(raw[i:i+4], 'little', signed=True)
                                         for i in (start, start+4, start+8))
            # Countdown units/range are not documented by this installed
            # header. Log the raw integer, but never use it to veto a valid
            # lamp, authorize crossing, or schedule a phase transition.
            if ident == light_id and 1 <= status <= 7:
                candidates.append(Case10Light(status, countdown, True, layout))
        return candidates[0] if len(candidates) == 1 else Case10Light(layout='invalid_payload')

    def read(self):
        # No cached-green fallback. The wrapper only takes pointer(data), with
        # no structure-specific argtypes; this buffer also guards ABI size.
        data = ctypes.create_string_buffer(b'\xff'*64, 64)
        ok = bool(self.sender('Ego', CASE10_LIGHT_ID, data))
        light = self.decode(data.raw) if ok else Case10Light(layout='API_false')
        diagnostic = (light.valid, light.status, light.layout)
        if diagnostic != self.last_diagnostic:
            print(f'CASE10_LIGHT: id={CASE10_LIGHT_ID}; valid={light.valid}; '
                  f'status={light.status}; countdown={light.countdown}; layout={light.layout}; '
                  '1=red,2=green,3=yellow,5=green_blink.', flush=True)
            self.last_diagnostic = diagnostic
        return light


class Case10GroundTruthPolicy:
    """Ego-only light case may have NO obstacle publication at all.

    The exported case contains only Ego. The failing run returned False/count=0/
    stamp=0 rather than True with an empty list. Allow that
    specific response for this route ONLY with a newly read, recognized lamp.
    Do not mask non-empty/stale responses or an outage after a real hazard has
    appeared. Case 13/24 never use this policy. No elapsed-time fallback.
    """

    def __init__(self):
        self.hazard_seen = False
        self.last_source = None

    def check(self, truth_ok, count, stamp, peers, light):
        if truth_ok:
            self.hazard_seen |= bool(peers)
            allowed, source = True, 'LIVE_GT'
        elif (count == 0 and stamp == 0 and not self.hazard_seen
              and light.valid and light.status in (1, 2, 3, 4, 5, 6)):
            allowed, source = True, 'EGO_ONLY_NO_GT'
        else:
            allowed, source = False, 'WAIT_GT_OR_LIGHT'
        if source != self.last_source:
            print(f'CASE10_GROUNDTRUTH: source={source}; objects={count}; '
                  f'stamp={stamp}; hazard_seen={self.hazard_seen}; '
                  f'live_light_valid={light.valid}; live_light={light.status}; '
                  'empty GT permitted only in this Ego-only light case.', flush=True)
            self.last_source = source
        return allowed, source


@dataclass(frozen=True)
class Case10Command(Command):
    light_status: int = 0
    light_countdown: int = -1
    light_valid: bool = False
    light_layout: str = 'unavailable'
    stopline_gap: float = float('inf')


class TrafficLightController:
    def __init__(self):
        self.integral = self.previous_steer = 0.0
        # This AionV/zero-offset IMU was measured in case 13. Stationary wheel
        # data below can refine the reference; exported OSC box centre (-1.4)
        # is not mixed with the live GPS reference.
        self.gps_to_rear, self.wheelbase = 0.0, WHEELBASE_M
        self.geometry_reader = None
        self.measured = self.entered = False
        self.last_gap = None
        self.last_green = False
        self.previous_mode = None

    def register_geometry_reader(self):
        from SimOneIOStruct import ESimOne_Data_Vehicle_State as State
        from SimOneIOStruct import SimOne_Data_Vehicle_Extra
        from SimOnePNCAPI import SoRegisterVehicleState, SoGetVehicleState
        states = (State*8)(State.ESimOne_Data_Vehicle_State_SO_X_L1,
                           State.ESimOne_Data_Vehicle_State_SO_Y_L1,
                           State.ESimOne_Data_Vehicle_State_SO_X_R1,
                           State.ESimOne_Data_Vehicle_State_SO_Y_R1,
                           State.ESimOne_Data_Vehicle_State_SO_X_L2,
                           State.ESimOne_Data_Vehicle_State_SO_Y_L2,
                           State.ESimOne_Data_Vehicle_State_SO_X_R2,
                           State.ESimOne_Data_Vehicle_State_SO_Y_R2)
        registered = bool(SoRegisterVehicleState('Ego', states, len(states)))
        if registered:
            self.geometry_reader = SoGetVehicleState, SimOne_Data_Vehicle_Extra
        print(f'CASE10_GEOMETRY: registration={registered}; '
              f'fallback_GPS_to_rear={self.gps_to_rear:.4f}m.', flush=True)

    def observe_geometry(self, gps):
        if self.measured or not self.geometry_reader or math.hypot(gps.velX, gps.velY) > 0.04:
            return
        from ParkingAPA import measured_geometry
        read, data_type = self.geometry_reader
        data = data_type()
        if read('Ego', data) and data.dataSize >= 8:
            measured = measured_geometry(gps, data.extra_states[:8])
            if measured:
                self.wheelbase, self.gps_to_rear = measured
                self.measured = True
                print(f'CASE10_GEOMETRY: wheelbase={self.wheelbase:.4f}m; '
                      f'GPS_to_rear={self.gps_to_rear:.4f}m; calibrated=True.', flush=True)

    def front_gap(self, ego):
        # Stop-line normal follows road 290, not world X. Include the leading
        # bumper corner under yaw; logs show actual PHYSICAL body clearance.
        c, s = math.cos(CASE10_LINE_HEADING), math.sin(CASE10_LINE_HEADING)
        yaw = wrap(ego.yaw-CASE10_LINE_HEADING)
        front_extent = ((CASE10_BODY_FRONT_M-self.gps_to_rear)*math.cos(yaw)
                        + EGO_WIDTH_M/2*abs(math.sin(yaw)))
        return ((CASE10_LINE_POINT[0]-ego.x)*c+(CASE10_LINE_POINT[1]-ego.y)*s-front_extent)

    def find_self(self, ego, objects):
        candidates = []
        for obj in objects:
            dx, dy = obj.x-ego.x, obj.y-ego.y
            along, lateral = dx*CASE10_C+dy*CASE10_S, -dx*CASE10_S+dy*CASE10_C
            if (obj.kind in VEHICLE_TYPES
                    and abs(along-(1.7-self.gps_to_rear)) < 0.35 and abs(lateral) < 0.15
                    and abs(wrap(obj.yaw-ego.yaw)) < 0.08
                    and abs(math.hypot(obj.vx, obj.vy)-ego.speed) < 0.5
                    and abs(obj.length-4.57) < 0.5 and abs(obj.width-2.21) < 0.3):
                candidates.append(obj.ident)
        return candidates[0] if len(candidates) == 1 else None

    def update(self, ego, objects, dt, self_id=None, *, light=None):
        if not all(math.isfinite(v) for v in (ego.x, ego.y, ego.yaw, ego.speed, dt)) or ego.speed < 0:
            raise ValueError('Invalid case 10 ego state')
        dt = clip(dt, 0.005, 0.15)
        light = light or Case10Light()
        along, lateral = case10_coordinates(ego.x, ego.y)
        distance = math.dist((ego.x, ego.y), CASE10_GOAL)
        gap = self.front_gap(ego)
        def command(mode, target=0.0, throttle=0.0, brake=0.0, lead=None, ttc=float('inf')):
            return Case10Command(steer, throttle, brake, target, mode, lateral, distance,
                                 lead, ttc, light.status, light.countdown, light.valid,
                                 light.layout, gap)
        steer = 0.0
        if abs(lateral) > 0.8 or abs(wrap(ego.yaw-CASE10_HEADING)) > 0.3:
            return command('LIGHT_LANE_FAILSAFE', brake=1.0)
        lookahead = clip(4.0+0.4*ego.speed, 4.0, 9.0)
        tx, ty = case10_world(along+lookahead)
        alpha = wrap(math.atan2(ty-ego.y, tx-ego.x)-ego.yaw)
        wheel = math.atan2(2*self.wheelbase*math.sin(alpha), math.hypot(tx-ego.x, ty-ego.y))
        steer = clip(-wheel/FULL_STEER_RAD, -0.4, 0.4)
        steer = clip(steer, self.previous_steer-1.2*dt, self.previous_steer+1.2*dt)
        self.previous_steer = steer
        if gap <= 0 and not self.entered:
            if light.green or (self.last_green and self.last_gap is not None and self.last_gap > 0):
                self.entered = True
            else:
                return command('LIGHT_ENTRY_FAILSAFE', brake=1.0)
        self.last_gap, self.last_green = gap, light.green
        if along >= CASE10_LENGTH-1:
            return command('LIGHT_GOAL_HOLD', brake=0.7)
        target, mode = CASE10_CRUISE_MPS, 'LIGHT_CRUISE'
        must_stop = not self.entered and not light.green
        if not self.entered:
            # Ensure stopping reserve at the position-triggered signal change,
            # even when the current light is green. Not a scripted red timer.
            if gap < 40:
                target, mode = min(target, 5.0), 'LIGHT_APPROACH'
            if gap < 12 and light.green:
                target, mode = CASE10_CRUISE_MPS, 'LIGHT_GREEN_GO'
            if must_stop:
                mode = ('LIGHT_RED_APPROACH' if light.valid and light.status in (1, 4)
                        else 'LIGHT_YELLOW_APPROACH' if light.valid and light.status in (3, 5, 6)
                        else 'LIGHT_UNKNOWN_APPROACH')
                room = gap-CASE10_STOP_GAP_M
                target = min(target, math.sqrt(2*1.5*max(0.0, room)), 0.8*max(0.0, room))
                if room <= 0.06:
                    self.integral = 0.0
                    return command('LIGHT_RED_HOLD' if light.valid and light.status in (1, 4)
                                   else 'LIGHT_SIGNAL_HOLD', brake=0.75)
        # Independent corridor hazard guard; do not feed case 10 geometry into
        # the cut-in controller or apply its 14 m following score buffer here.
        lead = None
        for obj in objects:
            if obj.ident == self_id:
                continue
            obj_s, obj_l = case10_coordinates(obj.x, obj.y)
            delta = wrap(obj.yaw-CASE10_HEADING)
            half_l = (abs(math.cos(delta))*obj.length+abs(math.sin(delta))*obj.width)/2
            half_w = (abs(math.sin(delta))*obj.length+abs(math.cos(delta))*obj.width)/2
            if abs(obj_l-lateral) > EGO_WIDTH_M/2+half_w+0.3:
                continue
            hazard_gap = obj_s-along-(CASE10_BODY_FRONT_M-self.gps_to_rear)-half_l
            if obj_s <= along or hazard_gap > 80:
                continue
            speed = max(0.0, obj.vx*CASE10_C+obj.vy*CASE10_S)
            closing = max(0.0, ego.speed-speed)
            candidate = Lead(obj.ident, hazard_gap, speed, False)
            if hazard_gap < 1.5+closing*0.25+max(0.0, ego.speed**2-speed**2)/8:
                self.integral = 0.0
                return command('LIGHT_AEB', brake=1.0, lead=candidate,
                               ttc=max(0.0, hazard_gap)/closing if closing > 0.05 else float('inf'))
            safe_speed = speed+math.sqrt(2*1.5*max(0.0, hazard_gap-5.0))
            if safe_speed < target:
                target, mode, lead = safe_speed, 'LIGHT_FOLLOW', candidate
        if mode != self.previous_mode:
            self.integral = 0.0  # Immediate green release; no braking wind-up.
            self.previous_mode = mode
        error = target-ego.speed
        self.integral = clip(self.integral+error*dt, -3, 3)
        if error < -0.06:
            self.integral = min(0.0, self.integral)
            brake = clip(0.05+0.22*(-error), 0.04, 0.85)
            if must_stop:
                room = max(0.04, gap-CASE10_STOP_GAP_M-ego.speed*0.15)
                required = ego.speed**2/(2*room)
                brake = max(brake, clip(required/7.0, 0.0, 1.0))
            throttle = 0.0
        elif error > 0.03:
            throttle, brake = clip(0.10+0.18*error+0.025*self.integral, 0.0, 0.55), 0.0
        else:
            throttle, brake = (0.10 if target > 1.0 else 0.0), 0.0
        return command(mode, target, throttle, brake, lead)


# Case 35: b48aa416-502e-4c07-8ad7-9485fcdb3769.sim. Independent geometry.
CASE35_START = (14.999989, -11.536283618913284)
CASE35_GOAL = (70.30568, -50.62022)
CASE35_HEADING = 0.004186265445891809
CASE35_C, CASE35_S = math.cos(CASE35_HEADING), math.sin(CASE35_HEADING)
CASE35_LINE = (57.404020056604026, -6.108722051045096)  # Road 168, stop line 303.
CASE35_STOP_GAP = 0.25
CASE35_BODY_FRONT = 1.7+4.57/2
CASE35_RIGHT_SIGNAL = 1 << 2
CASE35_BRAKE_SIGNAL = 1 << 3
CASE35_VRU_TYPES = {4, 8, 17, 21, 27}
CASE35_BUILD = '20261013-live-vru-right-turn'
# Road 216: lane -1 connects incoming 168/-2 -> outgoing 211/-2.
# Tuple: reference x/y/heading, length/s, bU/cU/dU, bV/cV/dV (normalized p).
CASE35_GEOMETRIES = (
    (57.31867281910979, -9.609110007979572, 0.004186265445892977,
     7.190231706164008, 0.0, 7.096930661240684, 0.08724032184854824,
     -0.42787942954984715, -2.220446049250313e-16, -2.707349748697948, 0.5350354864350733),
    (64.08399902891672, -11.753121688301663, 5.720738404383667,
     15.149189027792435, 7.190231706164008, 14.576398575980908,
     2.5322872485430388, -3.9328180914646254, 7.993605777301127e-15,
     -6.416404686035888, 0.1819917283642436),
)
CASE35_WIDTHS = (
    (0.0, 3.4994988187343132, -0.006310222566819404, -0.0022418924522843607, 0.0001711951812076649),
    (8.999999792667063, 3.385914815148198, -0.0056968409942285796, 0.0017146185465376767, -8.149669555470797e-05),
    (16.99999960837112, 3.4083493649264653, 0.012179380585624806, 0.0051088032751086705, -0.0008665293910380328),
    (20.99999951622315, 3.483349857588586, 0.034444804904186534, -0.028262715759586377, 0.008854093055645043),
)


def case35_incoming(x, y):
    dx, dy = x-CASE35_START[0], y-CASE35_START[1]
    return dx*CASE35_C+dy*CASE35_S, -dx*CASE35_S+dy*CASE35_C


def case35_pose_report(ego):
    along, lateral = case35_incoming(ego.x, ego.y)
    reasons = []
    if not -8 <= along <= 12 or abs(lateral) > 0.7:
        reasons.append('position_offset_or_late_start')
    if abs(wrap(ego.yaw-CASE35_HEADING)) > 0.15:
        reasons.append('heading_offset')
    return (f'START_POSE case=35: actual=({ego.x:.6f},{ego.y:.6f},{ego.yaw:.6f}); '
            f'speed={ego.speed:.3f}m/s; along={along:.3f}m; lateral={lateral:.3f}m; '
            f'result={"REJECT" if reasons else "ACCEPT"}; reasons={",".join(reasons) or "none"}',
            tuple(reasons))


@dataclass(frozen=True)
class Case35PathPoint:
    s: float
    x: float
    y: float
    yaw: float
    curvature: float


def case35_route():
    """Sample the exported OpenDRIVE LANE centre, not a guessed turning arc."""
    curve = []
    for x, y, hdg, length, start_s, bu, cu, du, bv, cv, dv in CASE35_GEOMETRIES:
        count = math.ceil(length/.08)
        for i in range(count+1):
            if curve and i == 0:
                continue
            p = i/count
            u, v = bu*p+cu*p*p+du*p**3, bv*p+cv*p*p+dv*p**3
            pu, pv = bu+2*cu*p+3*du*p*p, bv+2*cv*p+3*dv*p*p
            yaw = hdg+math.atan2(pv, pu)
            ref_s = start_s+length*p
            ws, a, b, c, d = max((w for w in CASE35_WIDTHS if w[0] <= ref_s), key=lambda w:w[0])
            q = ref_s-ws
            width = a+b*q+c*q*q+d*q**3
            rx, ry = x+u*math.cos(hdg)-v*math.sin(hdg), y+u*math.sin(hdg)+v*math.cos(hdg)
            curve.append((rx+width/2*math.sin(yaw), ry-width/2*math.cos(yaw)))
    approach_length = math.dist(CASE35_START, curve[0])
    count = math.ceil((approach_length+8)/.08)
    # Uniform join spacing prevents a millimetre-sized final straight segment
    # from amplifying tiny export rounding differences into false curvature.
    coordinates = [(CASE35_START[0]+s*(curve[0][0]-CASE35_START[0])/approach_length,
                    CASE35_START[1]+s*(curve[0][1]-CASE35_START[1])/approach_length)
                   for s in (-8+(approach_length+8)*i/count for i in range(count))]
    coordinates += curve
    exit_heading = -1.5651466693913327
    x, y = curve[-1]
    exit_length = math.dist((x, y), CASE35_GOAL)+8
    coordinates += [(x+s*math.cos(exit_heading), y+s*math.sin(exit_heading))
                    for s in (i*.08 for i in range(1, math.ceil(exit_length/.08)+1))]
    headings = [math.atan2(b[1]-a[1], b[0]-a[0]) for a,b in zip(coordinates, coordinates[1:])]
    headings.append(headings[-1])
    points, along = [], -8.0
    for i, (x, y) in enumerate(coordinates):
        if i:
            along += math.dist(coordinates[i-1], (x, y))
        left, right = max(0, i-2), min(len(coordinates)-1, i+2)
        span = sum(math.dist(coordinates[j], coordinates[j+1]) for j in range(left, right))
        curvature = wrap(headings[right]-headings[left])/max(.01, span)
        points.append(Case35PathPoint(along, x, y, headings[i], curvature))
    return points


@dataclass(frozen=True)
class Case35Command(Command):
    gear: int = 1
    signal_lights: int = 0
    stopline_gap: float = float('inf')
    vru_blockers: str = ''
    route_s: float = 0.0


class Case35SignalWriter:
    def __init__(self, sender, data_type):
        self.sender, self.data_type = sender, data_type
        self.last_flags = self.last_stamp = None

    @classmethod
    def from_sdk(cls):
        from SimOnePNCAPI import SoSetSignalLights
        from SimOneIOStruct import SimOne_Data_Signal_Lights
        return cls(SoSetSignalLights, SimOne_Data_Signal_Lights)

    def apply(self, ego, command):
        flags = command.signal_lights
        if flags == self.last_flags and self.last_stamp is not None and ego.stamp-self.last_stamp < 500:
            return
        data = self.data_type()
        data.timestamp, data.frame, data.signalLights = ego.stamp, ego.frame, flags
        if not self.sender('Ego', data):
            raise RuntimeError('Case 35 signal-lights command rejected; drive command withheld')
        if flags != self.last_flags:
            print(f'CASE35_SIGNALS: flags={flags}; right={bool(flags & CASE35_RIGHT_SIGNAL)}; accepted=True.', flush=True)
        self.last_flags, self.last_stamp = flags, ego.stamp


class VRUYieldController:
    """Full stop at line, live pedestrian/bicycle clearance, lane-centre right turn.

    No actor-speed script, disappearance assumption or fixed-duration wait.
    A missing tracked VRU is NOT assumed to have cleared the crossing.
    """

    def __init__(self):
        from ParkingAPA import rectangle, overlap, integrate
        self.rectangle, self.overlap, self.integrate = rectangle, overlap, integrate
        self.points = case35_route()
        self.index = None
        self.integral = self.previous_steer = 0.0
        self.wheelbase, self.gps_to_rear = WHEELBASE_M, 0.0
        self.geometry_reader = None
        self.measured = False
        self.roles = {}
        self.missing_s = self.line_still_s = self.clear_s = 0.0
        self.stopped_at_line = self.released = False
        self.last_roles = None

    def register_geometry_reader(self):
        from SimOneIOStruct import ESimOne_Data_Vehicle_State as State
        from SimOneIOStruct import SimOne_Data_Vehicle_Extra
        from SimOnePNCAPI import SoRegisterVehicleState, SoGetVehicleState
        states = (State*8)(State.ESimOne_Data_Vehicle_State_SO_X_L1,
                           State.ESimOne_Data_Vehicle_State_SO_Y_L1,
                           State.ESimOne_Data_Vehicle_State_SO_X_R1,
                           State.ESimOne_Data_Vehicle_State_SO_Y_R1,
                           State.ESimOne_Data_Vehicle_State_SO_X_L2,
                           State.ESimOne_Data_Vehicle_State_SO_Y_L2,
                           State.ESimOne_Data_Vehicle_State_SO_X_R2,
                           State.ESimOne_Data_Vehicle_State_SO_Y_R2)
        registered = bool(SoRegisterVehicleState('Ego', states, len(states)))
        if registered:
            self.geometry_reader = SoGetVehicleState, SimOne_Data_Vehicle_Extra
        print(f'CASE35_GEOMETRY: registration={registered}; fallback_GPS_to_rear=0m.', flush=True)

    def observe_geometry(self, gps):
        if self.measured or not self.geometry_reader or math.hypot(gps.velX, gps.velY) > .04:
            return
        from ParkingAPA import measured_geometry
        read, data_type = self.geometry_reader
        data = data_type()
        if read('Ego', data) and data.dataSize >= 8:
            result = measured_geometry(gps, data.extra_states[:8])
            if result:
                self.wheelbase, self.gps_to_rear = result
                self.measured = True
                print(f'CASE35_GEOMETRY: wheelbase={self.wheelbase:.4f}m; GPS_to_rear={self.gps_to_rear:.4f}m.', flush=True)

    def rear_pose(self, ego):
        return ego.x-self.gps_to_rear*math.cos(ego.yaw), ego.y-self.gps_to_rear*math.sin(ego.yaw), ego.yaw

    def front_gap(self, ego):
        dx, dy = CASE35_LINE[0]-ego.x, CASE35_LINE[1]-ego.y
        yaw = wrap(ego.yaw-CASE35_HEADING)
        extent = ((CASE35_BODY_FRONT-self.gps_to_rear)*math.cos(yaw)
                  + EGO_WIDTH_M/2*abs(math.sin(yaw)))
        return dx*CASE35_C+dy*CASE35_S-extent

    def find_self(self, ego, objects):
        candidates = []
        for obj in objects:
            dx, dy = obj.x-ego.x, obj.y-ego.y
            c, s = math.cos(ego.yaw), math.sin(ego.yaw)
            forward, lateral = dx*c+dy*s, -dx*s+dy*c
            if (obj.kind in VEHICLE_TYPES and abs(forward-(1.7-self.gps_to_rear)) < .4
                    and abs(lateral) < .2 and abs(wrap(obj.yaw-ego.yaw)) < .1
                    and abs(math.hypot(obj.vx, obj.vy)-ego.speed) < .6
                    and abs(obj.length-4.57) < .5 and abs(obj.width-2.21) < .3):
                candidates.append(obj.ident)
        return candidates[0] if len(candidates) == 1 else None

    def crossing_blockers(self, objects, self_id):
        peers = [obj for obj in objects if obj.ident != self_id]
        for role, anchor_x, kinds in (('ped', 59.4, {4}), ('bike', 61.2, {8, 17, 21, 27, 6})):
            if role not in self.roles:
                candidates = [obj for obj in peers if obj.kind in kinds and abs(obj.x-anchor_x) < 2
                              and -27 < obj.y < 10 and obj.ident not in self.roles.values()]
                if len(candidates) == 1:
                    self.roles[role] = candidates[0].ident
        if self.roles != self.last_roles:
            print(f'CASE35_ACTORS: {self.roles}; both_live_actors_required=True.', flush=True)
            self.last_roles = dict(self.roles)
        observed = {obj.ident:obj for obj in peers}
        missing = [role for role in ('ped', 'bike') if role not in self.roles
                   or self.roles[role] not in observed]
        blockers = ['missing:'+role for role in missing]
        for role, ident in self.roles.items():
            obj = observed.get(ident)
            if obj is None:
                continue
            dx, dy = obj.x-CASE35_LINE[0], obj.y-CASE35_LINE[1]
            lateral = -dx*CASE35_S+dy*CASE35_C
            yaw = wrap(obj.yaw-CASE35_HEADING)
            extent = (abs(math.sin(yaw))*obj.length+abs(math.cos(yaw))*obj.width)/2
            # Ped heads south, bicycle north. Use observed physical extents and
            # the far crosswalk edge (+/-9.1m), not a 26-second release timer.
            cleared = (lateral+extent < -9.4 if role == 'ped' else lateral-extent > 9.4)
            toward = (-obj.vx*CASE35_S+obj.vy*CASE35_C)*(1 if role == 'ped' else -1) > .15
            if not cleared or toward:
                blockers.append(f'{role}:{ident}')
        # Unexpected VRUs also block if in, or projected to enter, this crossing.
        for obj in peers:
            if obj.ident in self.roles.values() or obj.kind not in CASE35_VRU_TYPES:
                continue
            for horizon in (0., 1., 2., 3.):
                dx, dy = obj.x+obj.vx*horizon-CASE35_LINE[0], obj.y+obj.vy*horizon-CASE35_LINE[1]
                forward, lateral = dx*CASE35_C+dy*CASE35_S, -dx*CASE35_S+dy*CASE35_C
                if -.5 < forward < 6.5 and abs(lateral) < 10.5:
                    blockers.append(f'other:{obj.ident}')
                    break
        return blockers, missing, peers

    def track(self, pose, speed, dt):
        x, y, yaw = pose
        if self.index is None:
            candidates = range(len(self.points))
        else:
            candidates = range(max(0, self.index-15), min(len(self.points), self.index+100))
        self.index = min(candidates, key=lambda i:(self.points[i].x-x)**2+(self.points[i].y-y)**2)
        point = self.points[self.index]
        lateral = -(x-point.x)*math.sin(point.yaw)+(y-point.y)*math.cos(point.yaw)
        heading_error = wrap(yaw-point.yaw)
        target_index = self.index
        lookahead = clip(2.5+.25*speed, 3.0, 4.5)
        while target_index+1 < len(self.points) and self.points[target_index].s < point.s+lookahead:
            target_index += 1
        target = self.points[target_index]
        alpha = wrap(math.atan2(target.y-y, target.x-x)-yaw)
        wheel = math.atan2(2*self.wheelbase*math.sin(alpha), math.hypot(target.x-x, target.y-y))
        steering = clip(-wheel/FULL_STEER_RAD, -.8, .8)
        steering = clip(steering, self.previous_steer-1.8*dt, self.previous_steer+1.8*dt)
        self.previous_steer = steering
        curvature = max(abs(p.curvature) for p in self.points[self.index:min(len(self.points), target_index+25)])
        return steering, lateral, heading_error, point.s, curvature

    def update(self, ego, objects, dt, self_id=None):
        if not all(math.isfinite(v) for v in (ego.x, ego.y, ego.yaw, ego.speed, dt)) or ego.speed < 0:
            raise ValueError('Invalid case 35 ego state')
        dt = clip(dt, .005, .15)
        pose = self.rear_pose(ego)
        steer, lateral, heading_error, route_s, curvature = self.track(pose, ego.speed, dt)
        gap = self.front_gap(ego)
        distance = math.dist((ego.x, ego.y), CASE35_GOAL)
        blockers, missing, peers = self.crossing_blockers(objects, self_id)
        flags = CASE35_RIGHT_SIGNAL if gap < 20 and pose[1] > -27 else 0
        def command(mode, target=0., throttle=0., brake=0., lead=None, ttc=float('inf')):
            return Case35Command(steer, throttle, brake, target, mode, lateral, distance, lead,
                                 ttc, 1, flags | (CASE35_BRAKE_SIGNAL if brake > .02 else 0),
                                 gap, ','.join(blockers), route_s)
        if abs(lateral) > .65 or abs(heading_error) > .35:
            return command('VRU_LANE_FAILSAFE', brake=1.)
        if missing:
            self.missing_s += dt
            self.clear_s = 0
            if self.missing_s > 5:
                raise RuntimeError(f'Case 35 live actors unavailable: {missing}; no blind/timer release')
            return command('VRU_WAIT_ACTORS', brake=1.)
        self.missing_s = 0
        if not self.stopped_at_line and gap < -.05:
            return command('VRU_STOPLINE_FAILSAFE', brake=1.)
        if not self.stopped_at_line and .03 < gap < .5 and ego.speed < .05:
            self.line_still_s += dt
            if self.line_still_s >= .3:
                self.stopped_at_line = True
        else:
            self.line_still_s = 0
        if not blockers:
            self.clear_s += dt
        else:
            self.clear_s = 0
        if self.stopped_at_line and self.clear_s >= .25:
            self.released = True
        # Current and short-horizon polygon collision guards cover BOTH known
        # actors and unexpected solids. There is no distance-only AEB latch.
        wheel_curvature = math.tan(-steer*FULL_STEER_RAD)/self.wheelbase
        for obj in peers:
            for horizon in (0., .25, .5):
                future = self.integrate(pose, ego.speed*horizon, wheel_curvature)
                body = self.rectangle(*future, 4.67, 2.31, 1.7)
                hazard = self.rectangle(obj.x+obj.vx*horizon, obj.y+obj.vy*horizon,
                                        obj.yaw, obj.length+.2, obj.width+.2)
                if self.overlap(body, hazard):
                    self.integral = 0
                    return command('VRU_AEB', brake=1., lead=Lead(obj.ident, math.dist((ego.x,ego.y),(obj.x,obj.y)),
                                                                math.hypot(obj.vx,obj.vy), False), ttc=horizon)
        if distance < 1:
            return command('VRU_GOAL_HOLD', brake=.7)
        target = min(15/3.6, math.sqrt(.8/max(.01, curvature)))
        mode = 'VRU_RIGHT_TURN' if curvature > .015 else 'VRU_CRUISE'
        must_stop = not self.released or (bool(blockers) and gap > 0)
        if must_stop:
            room = max(0., gap-CASE35_STOP_GAP)
            target = min(target, math.sqrt(3*room), .8*room)
            mode = 'VRU_APPROACH_STOP'
            if room <= .06:
                self.integral = 0
                return command('VRU_YIELD_HOLD' if blockers else 'VRU_STOP_CONFIRM', brake=.75)
        else:
            if gap > 0:
                mode = 'VRU_CLEAR_GO'
        error = target-ego.speed
        self.integral = clip(self.integral+error*dt, -3, 3)
        if error < -.06:
            self.integral = min(0., self.integral)
            brake = clip(.05+.22*(-error), .04, .85)
            if must_stop:
                required = ego.speed**2/(2*max(.04, gap-CASE35_STOP_GAP-.15*ego.speed))
                brake = max(brake, clip(required/7, 0., 1.))
            throttle = 0.
        elif error > .03:
            throttle, brake = clip(.1+.18*error+.025*self.integral, 0., .55), 0.
        else:
            throttle, brake = (.1 if target > 1 else 0.), 0.
        return command(mode, target, throttle, brake)


# Case 18: 4cb4bd15-02e2-4eed-8f85-2183bcd4a48d.sim. Independent straight route.
CASE18_START = (-97.10138154160659, 2.0395385170814624)
CASE18_GOAL = (51.499448, 1.6642838971589824)
CASE18_HEADING = -0.002525247157515231
CASE18_C, CASE18_S = math.cos(CASE18_HEADING), math.sin(CASE18_HEADING)
CASE18_LENGTH = math.dist(CASE18_START, CASE18_GOAL)
CASE18_CRUISE = 25/3.6
CASE18_BODY_FRONT = 1.7+4.57/2
CASE18_BODY_REAR = 4.57/2-1.7
CASE18_YIELD_BUFFER = 3.0
CASE18_DECEL_GATE_X = -18.6  # Region ends at -18.8; never overrides an emergency.
CASE18_BUILD = '20261013-live-cross-traffic'


def case18_coordinates(x, y):
    dx, dy = x-CASE18_START[0], y-CASE18_START[1]
    return dx*CASE18_C+dy*CASE18_S, -dx*CASE18_S+dy*CASE18_C


def case18_world(along, lateral=0.):
    return (CASE18_START[0]+along*CASE18_C-lateral*CASE18_S,
            CASE18_START[1]+along*CASE18_S+lateral*CASE18_C)


def case18_pose_report(ego):
    along, lateral = case18_coordinates(ego.x, ego.y)
    reasons=[]
    if not -8 <= along <= 12 or abs(lateral) > .7:
        reasons.append('position_offset_or_late_start')
    if abs(wrap(ego.yaw-CASE18_HEADING)) > .15:
        reasons.append('heading_offset')
    return (f'START_POSE case=18: actual=({ego.x:.6f},{ego.y:.6f},{ego.yaw:.6f}); '
            f'speed={ego.speed:.3f}m/s; along={along:.3f}m; lateral={lateral:.3f}m; '
            f'result={"REJECT" if reasons else "ACCEPT"}; reasons={",".join(reasons) or "none"}',tuple(reasons))


@dataclass(frozen=True)
class Case18Forecast:
    ident: int
    enter: float
    leave: float
    ego_enter: float
    ego_leave: float
    stop_room: float
    direction: int
    risk: bool


@dataclass(frozen=True)
class Case18Command(Command):
    conflict_id: int | None = None
    cross_enter: float = float('inf')
    cross_leave: float = float('inf')
    ego_enter: float = float('inf')
    yield_ids: str = ''


class CrossTrafficController:
    """Live occupancy-window prediction, yielding latch and polygon AEB.

    Stationary cross traffic FAR outside our corridor must not cause a startup
    stop: this export triggers Audi by Ego TTC<2.5s. Once a conflict is observed,
    yielding persists until its physical rear has cleared the far corridor edge.
    No exported actor-speed/trajectory-time script is used for driving.
    """

    def __init__(self):
        from ParkingAPA import rectangle, overlap, integrate
        self.rectangle, self.overlap, self.integrate = rectangle, overlap, integrate
        self.wheelbase, self.gps_to_rear = WHEELBASE_M, 0.
        self.geometry_reader = None
        self.measured = False
        self.integral = self.previous_steer = 0.
        self.cross_id = None
        self.missing_s = 0.
        self.yielding = {}
        self.clear_times = {}
        self.last_yields = None

    def register_geometry_reader(self):
        from SimOneIOStruct import ESimOne_Data_Vehicle_State as State
        from SimOneIOStruct import SimOne_Data_Vehicle_Extra
        from SimOnePNCAPI import SoRegisterVehicleState, SoGetVehicleState
        states = (State*8)(State.ESimOne_Data_Vehicle_State_SO_X_L1,
                           State.ESimOne_Data_Vehicle_State_SO_Y_L1,
                           State.ESimOne_Data_Vehicle_State_SO_X_R1,
                           State.ESimOne_Data_Vehicle_State_SO_Y_R1,
                           State.ESimOne_Data_Vehicle_State_SO_X_L2,
                           State.ESimOne_Data_Vehicle_State_SO_Y_L2,
                           State.ESimOne_Data_Vehicle_State_SO_X_R2,
                           State.ESimOne_Data_Vehicle_State_SO_Y_R2)
        registered = bool(SoRegisterVehicleState('Ego', states, len(states)))
        if registered:self.geometry_reader = SoGetVehicleState, SimOne_Data_Vehicle_Extra
        print(f'CASE18_GEOMETRY: registration={registered}; fallback_GPS_to_rear=0m.',flush=True)

    def observe_geometry(self, gps):
        if self.measured or not self.geometry_reader or math.hypot(gps.velX,gps.velY) > .04:return
        from ParkingAPA import measured_geometry
        read,data_type=self.geometry_reader
        data=data_type()
        if read('Ego',data) and data.dataSize >= 8:
            result=measured_geometry(gps,data.extra_states[:8])
            if result:
                self.wheelbase,self.gps_to_rear=result;self.measured=True
                print(f'CASE18_GEOMETRY: wheelbase={self.wheelbase:.4f}m; GPS_to_rear={self.gps_to_rear:.4f}m.',flush=True)

    def find_self(self, ego, objects):
        candidates=[]
        for obj in objects:
            dx,dy=obj.x-ego.x,obj.y-ego.y
            along,lateral=dx*CASE18_C+dy*CASE18_S,-dx*CASE18_S+dy*CASE18_C
            if (obj.kind in VEHICLE_TYPES and abs(along-(1.7-self.gps_to_rear)) < .4
                    and abs(lateral) < .2 and abs(wrap(obj.yaw-ego.yaw)) < .1
                    and abs(math.hypot(obj.vx,obj.vy)-ego.speed) < .6
                    and abs(obj.length-4.57) < .5 and abs(obj.width-2.21) < .3):
                candidates.append(obj.ident)
        return candidates[0] if len(candidates)==1 else None

    @staticmethod
    def travel_time(distance, speed):
        if distance <= 0:return 0.
        speed=max(0.,speed)
        if speed >= CASE18_CRUISE:return distance/max(.1,speed)
        acceleration=2.
        tc=(CASE18_CRUISE-speed)/acceleration
        dc=speed*tc+.5*acceleration*tc*tc
        if distance <= dc:return (math.sqrt(speed*speed+2*acceleration*distance)-speed)/acceleration
        return tc+(distance-dc)/CASE18_CRUISE

    def forecast(self, ego, obj):
        es,_=case18_coordinates(ego.x,ego.y)
        os,lateral=case18_coordinates(obj.x,obj.y)
        yaw=wrap(obj.yaw-CASE18_HEADING)
        half_s=(abs(math.cos(yaw))*obj.length+abs(math.sin(yaw))*obj.width)/2
        half_l=(abs(math.sin(yaw))*obj.length+abs(math.cos(yaw))*obj.width)/2
        front,rear=CASE18_BODY_FRONT-self.gps_to_rear,CASE18_BODY_REAR+self.gps_to_rear
        delta=os-es
        if delta+half_s+rear+.7 < 0:return None
        vl=-obj.vx*CASE18_S+obj.vy*CASE18_C
        corridor=EGO_WIDTH_M/2+half_l+.5
        if abs(vl) < .08:
            if abs(lateral) > corridor:return None
            enter,leave=0.,float('inf')
            direction=-1 if lateral >= 0 else 1
        else:
            a,b=sorted(((-corridor-lateral)/vl,(corridor-lateral)/vl))
            if b < 0:return None
            enter,leave=max(0.,a),b
            direction=1 if vl>0 else -1
        # Along-motion is included at first corridor entry; recomputed from
        # every fresh observation rather than using the export's 20 km/h value.
        va=obj.vx*CASE18_C+obj.vy*CASE18_S
        entry_delta=delta+va*enter
        ego_enter=self.travel_time(entry_delta-half_s-front-.6,ego.speed)
        ego_leave=self.travel_time(entry_delta+half_s+rear+.6,ego.speed)
        risk=max(enter,ego_enter-.25) <= min(leave,ego_leave+.35)
        stop_room=delta-half_s-front-CASE18_YIELD_BUFFER
        return Case18Forecast(obj.ident,enter,leave,ego_enter,ego_leave,stop_room,direction,risk)

    def physical_clear(self, ego, obj, direction):
        es,_=case18_coordinates(ego.x,ego.y)
        os,lateral=case18_coordinates(obj.x,obj.y)
        yaw=wrap(obj.yaw-CASE18_HEADING)
        hs=(abs(math.cos(yaw))*obj.length+abs(math.sin(yaw))*obj.width)/2
        hl=(abs(math.sin(yaw))*obj.length+abs(math.cos(yaw))*obj.width)/2
        vl=-obj.vx*CASE18_S+obj.vy*CASE18_C
        passed=es-(CASE18_BODY_REAR+self.gps_to_rear) > os+hs+.8
        far=(lateral-hl > EGO_WIDTH_M/2+.7 if direction>0
             else lateral+hl < -EGO_WIDTH_M/2-.7)
        return passed or far and vl*direction >= -.08

    def update(self, ego, objects, dt, self_id=None):
        if not all(math.isfinite(v) for v in (ego.x,ego.y,ego.yaw,ego.speed,dt)) or ego.speed < 0:
            raise ValueError('Invalid case 18 ego state')
        dt=clip(dt,.005,.15)
        es,lateral=case18_coordinates(ego.x,ego.y)
        distance=math.dist((ego.x,ego.y),CASE18_GOAL)
        lookahead=clip(3.5+.35*ego.speed,4.,7.)
        tx,ty=case18_world(es+lookahead)
        alpha=wrap(math.atan2(ty-ego.y,tx-ego.x)-ego.yaw)
        wheel=math.atan2(2*self.wheelbase*math.sin(alpha),math.hypot(tx-ego.x,ty-ego.y))
        steer=clip(-wheel/FULL_STEER_RAD,-.45,.45)
        steer=clip(steer,self.previous_steer-1.2*dt,self.previous_steer+1.2*dt)
        self.previous_steer=steer
        peers=[obj for obj in objects if obj.ident != self_id]
        observed={obj.ident:obj for obj in peers}
        forecasts={obj.ident:f for obj in peers if (f:=self.forecast(ego,obj)) is not None}
        focus=min((f for f in forecasts.values() if f.risk),key=lambda f:f.ego_enter,default=None)
        def command(mode,target=0.,throttle=0.,brake=0.,lead=None,ttc=float('inf')):
            f=focus or forecasts.get(self.cross_id)
            return Case18Command(steer,throttle,brake,target,mode,lateral,distance,lead,ttc,
                                 f.ident if f else self.cross_id,f.enter if f else float('inf'),
                                 f.leave if f else float('inf'),f.ego_enter if f else float('inf'),
                                 ','.join(map(str,sorted(self.yielding))))
        if abs(lateral) > .8 or abs(wrap(ego.yaw-CASE18_HEADING)) > .3:
            return command('CROSS_LANE_FAILSAFE',brake=1.)
        if self.cross_id is None:
            candidates=[o for o in peers if o.kind in VEHICLE_TYPES and abs(o.x+3.1276533668)<8 and -42<o.y<35]
            if len(candidates)==1:
                self.cross_id=candidates[0].ident
                print(f'CASE18_ACTOR: Audi_GT_id={self.cross_id}; live_velocity_only=True.',flush=True)
        missing=self.cross_id is None or self.cross_id not in observed or any(i not in observed for i in self.yielding)
        if missing:
            self.missing_s+=dt
            if self.missing_s>5:raise RuntimeError('Case 18 cross-traffic data missing; no disappearance/timer release')
            return command('CROSS_WAIT_ACTOR',brake=1.)
        self.missing_s=0.
        for ident,f in forecasts.items():
            if f.risk and not self.physical_clear(ego,observed[ident],f.direction):
                self.yielding.setdefault(ident,f.direction)
        for ident,direction in list(self.yielding.items()):
            self.clear_times[ident]=(self.clear_times.get(ident,0.)+dt if self.physical_clear(ego,observed[ident],direction) else 0.)
            if self.clear_times[ident] >= .2:
                del self.yielding[ident];self.clear_times.pop(ident,None)
        ids=tuple(sorted(self.yielding))
        if ids!=self.last_yields:
            print(f'CASE18_YIELD: ids={ids}; actual_rear_clear_required=True.',flush=True)
            self.last_yields=ids
        # Short-horizon swept physical boxes are an emergency backstop, including
        # traffic not selected as the main Audi. This bypasses the decel gate.
        rear=(ego.x-self.gps_to_rear*math.cos(ego.yaw),ego.y-self.gps_to_rear*math.sin(ego.yaw),ego.yaw)
        curvature=math.tan(-steer*FULL_STEER_RAD)/self.wheelbase
        for obj in peers:
            for horizon in (0.,.2,.4,.6):
                future=self.integrate(rear,ego.speed*horizon,curvature)
                if self.overlap(self.rectangle(*future,4.67,2.31,1.7),
                                self.rectangle(obj.x+obj.vx*horizon,obj.y+obj.vy*horizon,obj.yaw,obj.length+.2,obj.width+.2)):
                    self.integral=0.
                    return command('CROSS_AEB',brake=1.,lead=Lead(obj.ident,math.dist((ego.x,ego.y),(obj.x,obj.y)),
                                                                math.hypot(obj.vx,obj.vy),False),ttc=horizon)
        if es >= CASE18_LENGTH-1:return command('CROSS_GOAL_HOLD',brake=.7)
        target,mode=CASE18_CRUISE,'CROSS_CRUISE'
        room=None
        if self.yielding:
            rooms=[]
            for ident in self.yielding:
                obj=observed[ident];os,_=case18_coordinates(obj.x,obj.y);yaw=wrap(obj.yaw-CASE18_HEADING)
                hs=(abs(math.cos(yaw))*obj.length+abs(math.sin(yaw))*obj.width)/2
                rooms.append(os-es-hs-(CASE18_BODY_FRONT-self.gps_to_rear)-CASE18_YIELD_BUFFER)
            room=min(rooms)
            reserve=.15*ego.speed+ego.speed**2/(2*4.5)+.25
            # Far-side traffic remains observed. We maintain the approach only
            # while a full stopping reserve exists; no forced "minimum speed".
            if ego.x < CASE18_DECEL_GATE_X and room > reserve:
                mode='CROSS_PREDICT_RESERVE'
                room=None
            else:
                mode='CROSS_YIELD_APPROACH'
                target=min(target,math.sqrt(3*max(0.,room)),.9*max(0.,room))
                if room <= .06:
                    self.integral=0.
                    return command('CROSS_YIELD_HOLD',brake=.75)
        error=target-ego.speed
        self.integral=clip(self.integral+error*dt,-3,3)
        if error < -.08:
            self.integral=min(0.,self.integral)
            throttle,brake=0.,clip(.05+.22*(-error),.04,.85)
            if room is not None:
                required=ego.speed**2/(2*max(.04,room-.15*ego.speed))
                brake=max(brake,clip(required/7,0.,1.))
        elif error > .03:
            throttle,brake=clip(.1+.18*error+.025*self.integral,0.,.55),0.
        else:throttle,brake=(.1 if target>1 else 0.),0.
        return command(mode,target,throttle,brake)


# Three-case batch: 6f7cb920-5f31-4cd4-a481-f6aa42d2afab.sim. Existing
# controllers above are intentionally NOT refactored into this new framework.
BATCH_BUILD = '20261013-batch-15-17-31'
BATCH_PROFILES = {
    15: ((69.600185,-3.4642091148056076),(68.847985,24.781886),-0.001577285659118677,25/3.6),
    17: ((-499.300335,-18.087617),(-373.199397,-14.263581413065765),0.002610960124359349,25/3.6),
    31: ((-80.998391,2.008975409461999),(36.501051,1.7021586369903654),-0.002525247157512519,40/3.6),
}
# Road id, lane width polynomial (a,b,c,d), normalized OpenDRIVE geometries.
# A line geometry has five fields; paramPoly3 adds bU,cU,dU,bV,cV,dV.
CASE15_ROADS = (
    (73,(3.972458501429597,0.,0.,0.),
     ((-174.0000000006997,-1.0937499969467959,6.2816080215204675,317.0003943051993,0.),)),
    (799,(3.9710781123473615,.00013868838529686504,2.1000937807734036e-05,-7.066832082429843e-07),(
        (142.99999998302815,-1.5937499655002318,6.281608021522447,5.430651137329586,0.,5.3967841191649235,.16550619192906701,-.24446989262118368,1.4432899320127035e-15,.714611401428658,.20520642061236005),
        (148.31926460144194,-.6823210060296109,.3870302068385789,6.430215121054623,5.430651137329586,6.357910469482556,.20048186155555126,-.4218179762788683,-1.9984014443252818e-15,1.7880830901042524,-.125109630764908),
        (153.37426884250357,3.1738374010907235,.9146077014983841,4.466136318011998,11.86086625838421,4.441107473255585,.07390138041189509,-.14920786417139897,-9.325873406851315e-15,.8390310191850188,-.02464946909839183),
        (155.39259687285266,7.129818638522679,1.2841586796836777,3.481487267628662,16.327002576396207,3.4696753382625802,.039031063951982414,-.07320837564312566,6.189493362285248e-15,.4764190848247599,.009379497276215676),
    )),
    (764,(3.9687728828851903,-1.4746664770684385e-05,-2.2502922826864033e-06,7.630825811588427e-08),(
        (155.8979326631597,10.562497285291482,1.5707963267930776,7.915815898655287,0.,7.7804137767459185,.3842143280052932,-.7946186487712934,-8.881784197001252e-16,2.734310671056651,-.24075915642053403),
        (153.40438114853697,17.93250674127594,2.2269041949882937,7.424433763570354,7.915815898655287,7.313067838367277,.33742663276941975,-.6681461428749262,7.549516567451064e-15,2.3184048684199503,-.1440605922341609),
        (147.42199632671984,22.13869659103947,2.8394438228073833,2.972864501904902,15.340249662225641,2.9637465709863906,.035605830849740006,-.0601115424144818,-1.4988010832439613e-15,.34419635807435456,.03614152767953627),
        (144.50272720051458,22.650225188303057,3.11164761193798,1.346551732110533,18.313114164130543,1.3464906940860988,-.0007981551342324256,.0003614108520679782,-3.3306690738754696e-16,.06574496378834646,-.031452785706012155),
    )),
    (79,(3.969030006565536,0.,0.,0.),
     ((143.15624998537834,22.65625000191294,3.1392276601684657,316.15666476548904,0.,316.15664535759737,-.000291118370626009,.00019407891375067265,-5.2263748884229244e-14,.7431244893424666,-.49541633670941954),)),
)


@dataclass(frozen=True)
class BatchPoint:
    s: float
    x: float
    y: float
    yaw: float
    curvature: float
    half_width: float = 1.75


class BatchRoute:
    def __init__(self, coordinates):
        # Remove tiny joins before differentiating. Retain real corners/curves.
        clean=[]
        for p in coordinates:
            if not clean or math.dist(clean[-1][:2],p[:2])>=.04:clean.append(p)
        headings=[]
        for i,p in enumerate(clean):
            a,b=clean[max(0,i-1)],clean[min(len(clean)-1,i+1)]
            headings.append(math.atan2(b[1]-a[1],b[0]-a[0]))
        distances=[0.]
        for a,b in zip(clean,clean[1:]):distances.append(distances[-1]+math.dist(a[:2],b[:2]))
        self.points=[]
        for i,p in enumerate(clean):
            a,b=max(0,i-2),min(len(clean)-1,i+2)
            k=wrap(headings[b]-headings[a])/max(.01,distances[b]-distances[a])
            self.points.append(BatchPoint(distances[i],p[0],p[1],headings[i],k,p[2]))
        self.hints={}
        self.s_values=distances

    @classmethod
    def straight(cls,start,heading,length,lateral=0.):
        c,s=math.cos(heading),math.sin(heading)
        return cls([(start[0]+u*c-lateral*s,start[1]+u*s+lateral*c,1.75)
                    for u in (-8+i*.1 for i in range(math.ceil((length+16)/.1)+1))])

    def project(self,x,y,key=None):
        previous=self.hints.get(key) if key is not None else None
        candidates=range(len(self.points)) if previous is None else range(max(0,previous-30),min(len(self.points),previous+130))
        i=min(candidates,key=lambda i:(self.points[i].x-x)**2+(self.points[i].y-y)**2)
        p=self.points[i]
        if math.dist((x,y),(p.x,p.y))>3 and previous is not None:
            i=min(range(len(self.points)),key=lambda i:(self.points[i].x-x)**2+(self.points[i].y-y)**2)
            p=self.points[i]
        if key is not None:self.hints[key]=i
        dx,dy=x-p.x,y-p.y
        return p.s+dx*math.cos(p.yaw)+dy*math.sin(p.yaw),-dx*math.sin(p.yaw)+dy*math.cos(p.yaw),p,i

    def sample(self,s):
        import bisect
        i=max(0,min(len(self.points)-2,bisect.bisect_right(self.s_values,s)-1))
        a,b=self.points[i:i+2];u=clip((s-a.s)/max(.001,b.s-a.s),0.,1.)
        return BatchPoint(s,a.x+(b.x-a.x)*u,a.y+(b.y-a.y)*u,
                          wrap(a.yaw+wrap(b.yaw-a.yaw)*u),a.curvature+(b.curvature-a.curvature)*u,a.half_width)


def case15_route():
    coordinates=[]
    for rid,width,geometries in CASE15_ROADS:
        for g in geometries:
            x,y,h,length,start_s=g[:5];count=math.ceil(length/.08)
            reference=[]
            for i in range(count+1):
                p=i/count
                if len(g)==5:u,v,pu,pv=length*p,0.,length,0.
                else:
                    bu,cu,du,bv,cv,dv=g[5:]
                    u,v=bu*p+cu*p*p+du*p**3,bv*p+cv*p*p+dv*p**3
                    pu,pv=bu+2*cu*p+3*du*p*p,bv+2*cv*p+3*dv*p*p
                reference.append((x+u*math.cos(h)-v*math.sin(h),y+u*math.sin(h)+v*math.cos(h),h+math.atan2(pv,pu)))
            ref_s=[0.]
            for a,b in zip(reference,reference[1:]):ref_s.append(ref_s[-1]+math.dist(a[:2],b[:2]))
            for i,(rx,ry,yaw) in enumerate(reference):
                s=start_s+length*ref_s[i]/max(.001,ref_s[-1]);a,b,c,d=width
                w=a+b*s+c*s*s+d*s**3
                coordinates.append((rx+w/2*math.sin(yaw),ry-w/2*math.cos(yaw),w/2))
    start,goal,_,_=BATCH_PROFILES[15]
    first=min(range(len(coordinates)),key=lambda i:math.dist(coordinates[i][:2],start))
    last=min(range(first,len(coordinates)),key=lambda i:math.dist(coordinates[i][:2],goal))
    return BatchRoute(coordinates[max(0,first-100):min(len(coordinates),last+650)])


def batch_pose_report(ego,number):
    start,goal,heading,_=BATCH_PROFILES[number]
    c,s=math.cos(heading),math.sin(heading);dx,dy=ego.x-start[0],ego.y-start[1]
    along,lateral=dx*c+dy*s,-dx*s+dy*c
    reasons=[]
    if not -8<=along<=10 or abs(lateral)>.7:reasons.append('position_offset_or_late_start')
    if abs(wrap(ego.yaw-heading))>.15:reasons.append('heading_offset')
    return (f'START_POSE case={number}: actual=({ego.x:.6f},{ego.y:.6f},{ego.yaw:.6f}); '
            f'speed={ego.speed:.3f}; along={along:.3f}; lateral={lateral:.3f}; '
            f'result={"REJECT" if reasons else "ACCEPT"}; reasons={",".join(reasons) or "none"}',tuple(reasons))


@dataclass(frozen=True)
class BatchCommand(Command):
    gear: int = 1
    signal_lights: int = 0
    route_s: float = 0.
    stopline_gap: float = float('inf')
    light_status: int = 0
    light_countdown: int = -1
    light_valid: bool = False
    light_layout: str = ''
    yield_ids: str = ''


class BatchSignalWriter:
    def __init__(self,sender,data_type,number):
        self.sender,self.data_type,self.number=sender,data_type,number
        self.last_flags=self.last_stamp=None

    @classmethod
    def from_sdk(cls,number):
        from SimOnePNCAPI import SoSetSignalLights
        from SimOneIOStruct import SimOne_Data_Signal_Lights
        return cls(SoSetSignalLights,SimOne_Data_Signal_Lights,number)

    def apply(self,ego,cmd):
        flags=cmd.signal_lights
        if flags==self.last_flags and self.last_stamp is not None and ego.stamp-self.last_stamp<500:return
        data=self.data_type();data.timestamp,data.frame,data.signalLights=ego.stamp,ego.frame,flags
        if not self.sender('Ego',data):raise RuntimeError(f'Case {self.number} signal command rejected; throttle withheld')
        if flags!=self.last_flags:print(f'CASE{self.number}_SIGNALS: flags={flags}; accepted=True.',flush=True)
        self.last_flags,self.last_stamp=flags,ego.stamp


class Case31LightReader(Case10LightReader):
    def read(self):
        data=ctypes.create_string_buffer(b'\xff'*64,64)
        light=self.decode(data.raw) if self.sender('Ego',52,data) else Case10Light(layout='API_false')
        diagnostic=(light.valid,light.status,light.layout)
        if diagnostic!=self.last_diagnostic:
            print(f'CASE31_LIGHT: id=52; valid={light.valid}; status={light.status}; layout={light.layout}.',flush=True)
            self.last_diagnostic=diagnostic
        return light


class BatchControllerBase:
    """New-cases-only geometry/path/control helpers; old tested code untouched."""
    def __init__(self,number,route):
        from ParkingAPA import rectangle,overlap,integrate
        self.rectangle,self.overlap,self.integrate=rectangle,overlap,integrate
        self.number,self.route=number,route
        self.start,self.goal,self.heading,self.cruise=BATCH_PROFILES[number]
        self.c,self.s=math.cos(self.heading),math.sin(self.heading)
        self.wheelbase,self.gps_to_rear=WHEELBASE_M,0.
        self.geometry_reader=None;self.measured=False
        self.integral=self.previous_steer=self.missing_s=0.
        self.actor_id=None
        self.goal_s=self.route.project(*self.goal)[0]

    def coordinates(self,x,y):
        dx,dy=x-self.start[0],y-self.start[1]
        return dx*self.c+dy*self.s,-dx*self.s+dy*self.c

    def world(self,s,l=0.):
        return self.start[0]+s*self.c-l*self.s,self.start[1]+s*self.s+l*self.c

    def register_geometry_reader(self):
        from SimOneIOStruct import ESimOne_Data_Vehicle_State as State,SimOne_Data_Vehicle_Extra
        from SimOnePNCAPI import SoRegisterVehicleState,SoGetVehicleState
        values=[getattr(State,'ESimOne_Data_Vehicle_State_SO_'+n) for n in ('X_L1','Y_L1','X_R1','Y_R1','X_L2','Y_L2','X_R2','Y_R2')]
        states=(State*8)(*values)
        ok=bool(SoRegisterVehicleState('Ego',states,8))
        if ok:self.geometry_reader=SoGetVehicleState,SimOne_Data_Vehicle_Extra
        print(f'CASE{self.number}_GEOMETRY: registration={ok}; fallback_GPS_to_rear=0m.',flush=True)

    def observe_geometry(self,gps):
        if self.measured or not self.geometry_reader or math.hypot(gps.velX,gps.velY)>.04:return
        from ParkingAPA import measured_geometry
        read,data_type=self.geometry_reader;data=data_type()
        if read('Ego',data) and data.dataSize>=8:
            result=measured_geometry(gps,data.extra_states[:8])
            if result:
                self.wheelbase,self.gps_to_rear=result;self.measured=True
                print(f'CASE{self.number}_GEOMETRY: wheelbase={self.wheelbase:.4f}; GPS_to_rear={self.gps_to_rear:.4f}.',flush=True)

    def find_self(self,ego,objects):
        candidates=[]
        for obj in objects:
            dx,dy=obj.x-ego.x,obj.y-ego.y;c,s=math.cos(ego.yaw),math.sin(ego.yaw)
            if (obj.kind in VEHICLE_TYPES and abs(dx*c+dy*s-(1.7-self.gps_to_rear))<.4
                    and abs(-dx*s+dy*c)<.2 and abs(wrap(obj.yaw-ego.yaw))<.1
                    and abs(math.hypot(obj.vx,obj.vy)-ego.speed)<.6
                    and abs(obj.length-4.57)<.5 and abs(obj.width-2.21)<.3):candidates.append(obj.ident)
        return candidates[0] if len(candidates)==1 else None

    def rear_pose(self,ego):
        return ego.x-self.gps_to_rear*math.cos(ego.yaw),ego.y-self.gps_to_rear*math.sin(ego.yaw),ego.yaw

    def track(self,ego,dt):
        pose=self.rear_pose(ego);es,lateral,p,i=self.route.project(*pose[:2],key='ego')
        lookahead=clip(2.2+.3*ego.speed,2.8,5.0)
        target=self.route.sample(es+lookahead)
        alpha=wrap(math.atan2(target.y-pose[1],target.x-pose[0])-ego.yaw)
        wheel=math.atan2(2*self.wheelbase*math.sin(alpha),math.hypot(target.x-pose[0],target.y-pose[1]))
        steer=clip(-wheel/FULL_STEER_RAD,-.75,.75)
        steer=clip(steer,self.previous_steer-1.6*dt,self.previous_steer+1.6*dt);self.previous_steer=steer
        curvature=max(abs(q.curvature) for q in self.route.points[i:min(len(self.route.points),i+100)])
        return pose,es,lateral,wrap(ego.yaw-p.yaw),steer,curvature

    def collision(self,pose,speed,steer,peers):
        curvature=math.tan(-steer*FULL_STEER_RAD)/self.wheelbase
        for obj in peers:
            for t in (0.,.25,.5):
                future=self.integrate(pose,speed*t,curvature)
                if self.overlap(self.rectangle(*future,4.67,2.31,1.7),
                                self.rectangle(obj.x+obj.vx*t,obj.y+obj.vy*t,obj.yaw,obj.length+.2,obj.width+.2)):
                    return Lead(obj.ident,math.dist(pose[:2],(obj.x,obj.y)),math.hypot(obj.vx,obj.vy),False),t
        return None,float('inf')

    def pedals(self,ego,target,dt,room=None):
        error=target-ego.speed;self.integral=clip(self.integral+error*dt,-3,3)
        if error<-.06:
            self.integral=min(0.,self.integral);throttle=0.;brake=clip(.05+.22*(-error),.04,.85)
            if room is not None:brake=max(brake,clip(ego.speed**2/(2*max(.04,room-.15*ego.speed))/7,0.,1.))
        elif error>.03:throttle,brake=clip(.1+.18*error+.025*self.integral,0.,.55),0.
        else:throttle,brake=(.1 if target>1 else 0.),0.
        return throttle,brake

    def following(self,ego,pose,es,peers,target,buffer=14.):
        best=None;mode='CRUISE';ttc=float('inf');emergency=False
        for obj in peers:
            if obj.kind not in VEHICLE_TYPES|{8,17,21,27}:continue
            os,ol,p,_=self.route.project(obj.x,obj.y,key=('obj',obj.ident))
            if not es<os<es+90 or abs(ol)>EGO_WIDTH_M/2+obj.width/2+.3:continue
            gap=os-es-(1.7+4.57/2)-obj.length/2
            speed=max(0.,obj.vx*math.cos(p.yaw)+obj.vy*math.sin(p.yaw))
            closing=max(0.,ego.speed-speed);reserve=.2*closing+max(0.,ego.speed**2-speed**2)/8
            urgent=gap<1.5 or closing>.2 and gap<12.0+reserve
            if gap<buffer:planned=max(0.,speed-(buffer-gap)/3)
            else:planned=speed+math.sqrt(.9**2+3.6*(gap-buffer))-.9
            planned=min(target,planned)
            if urgent or best is None or planned<target:
                best=Lead(obj.ident,gap,speed,False);target=planned;mode='FOLLOW';ttc=max(0.,gap)/closing if closing>.05 else float('inf')
            if urgent:emergency=True;break
        return target,mode,best,ttc,emergency

    def require_actor(self,ego,peers,dt,candidate):
        if self.actor_id is None:
            options=[o for o in peers if candidate(o)]
            if len(options)==1:
                self.actor_id=options[0].ident
                print(f'CASE{self.number}_ACTOR: GT_id={self.actor_id}; live_data_required=True.',flush=True)
        actor=next((o for o in peers if o.ident==self.actor_id),None)
        if actor is None:
            self.missing_s+=dt
            if self.missing_s>5:raise RuntimeError(f'Case {self.number} actor missing; no blind release')
        else:self.missing_s=0.
        return actor


class CurveFollowingController(BatchControllerBase):
    def __init__(self):super().__init__(15,case15_route())

    def update(self,ego,objects,dt,self_id=None):
        if not all(math.isfinite(v) for v in (ego.x,ego.y,ego.yaw,ego.speed,dt)) or ego.speed<0:raise ValueError('Invalid case 15 state')
        dt=clip(dt,.005,.15);pose,es,lat,he,steer,k=self.track(ego,dt);distance=math.dist((ego.x,ego.y),self.goal)
        peers=[o for o in objects if o.ident!=self_id]
        def cmd(mode,target=0.,throttle=0.,brake=0.,lead=None,ttc=float('inf')):
            return BatchCommand(steer,throttle,brake,target,mode,lat,distance,lead,ttc,route_s=es)
        if abs(lat)>.45 or abs(he)>.35:return cmd('CURVE_LANE_FAILSAFE',brake=1.)
        actor=self.require_actor(ego,peers,dt,lambda o:o.kind in VEHICLE_TYPES and 90<o.x<110 and -6<o.y<0)
        if actor is None:return cmd('CURVE_WAIT_ACTOR',brake=1.)
        collision,t=self.collision(pose,ego.speed,steer,peers)
        if collision:return cmd('CURVE_AEB',brake=1.,lead=collision,ttc=t)
        if es>=self.goal_s-1:return cmd('CURVE_GOAL_HOLD',brake=.7)
        target=min(self.cruise,math.sqrt(1.8/max(.01,k)))
        target,mode,lead,ttc,urgent=self.following(ego,pose,es,peers,target,buffer=16.5)
        if urgent:return cmd('CURVE_DISTANCE_AEB',brake=1.,lead=lead,ttc=ttc)
        throttle,brake=self.pedals(ego,target,dt)
        return cmd('CURVE_'+mode,target,throttle,brake,lead,ttc)


class SlowVehicleLaneChangeController(BatchControllerBase):
    def __init__(self):
        start,goal,h,_=BATCH_PROFILES[17]
        super().__init__(17,BatchRoute.straight(start,h,math.dist(start,goal)))
        self.phase='PRE_SIGNAL';self.signal_s=0.;self.change_start=None;self.change_length=None

    def left_clear(self,ego,peers):
        es,_=self.coordinates(*self.rear_pose(ego)[:2])
        for obj in peers:
            os,ol=self.coordinates(obj.x,obj.y)
            if abs(ol-3.5)>EGO_WIDTH_M/2+obj.width/2+.4:continue
            speed=obj.vx*self.c+obj.vy*self.s
            if -30-max(0.,speed-ego.speed)*5 < os-es < 35:return False
        return True

    def plan_change(self,ego,peers):
        pose=self.rear_pose(ego);start_s,start_l=self.coordinates(*pose[:2]);goal_s,_=self.coordinates(*self.goal)
        for length in (24.,28.,32.,36.):
            if start_s+length>goal_s-12:continue
            coordinates=[];safe=True
            speed=max(2.,min(4.,ego.speed))
            for i in range(math.ceil((goal_s+10-start_s)/.12)+1):
                s=start_s+i*.12;u=clip((s-start_s)/length,0.,1.)
                f=10*u**3-15*u**4+6*u**5
                derivative=(3.5-start_l)*(30*u*u-60*u**3+30*u**4)/length
                yaw=self.heading+math.atan(derivative)
                x,y=self.world(s,start_l+(3.5-start_l)*f)
                body=self.rectangle(x,y,yaw,4.77,2.41,1.7)
                if any(not -1.68<self.coordinates(bx,by)[1]<5.18 for bx,by in body):safe=False;break
                t=(s-start_s)/speed
                if any(self.overlap(body,self.rectangle(o.x+o.vx*t,o.y+o.vy*t,o.yaw,o.length+.25,o.width+.25)) for o in peers):safe=False;break
                coordinates.append((x,y,1.75))
            if safe:
                self.route=BatchRoute(coordinates);self.goal_s=self.route.project(*self.goal)[0]
                self.change_start,self.change_length=start_s,length
                print(f'CASE17_PLAN: start={start_s:.3f}; length={length}; full_swept_route_valid=True.',flush=True)
                return True
        return False

    def update(self,ego,objects,dt,self_id=None):
        if not all(math.isfinite(v) for v in (ego.x,ego.y,ego.yaw,ego.speed,dt)) or ego.speed<0:raise ValueError('Invalid case 17 state')
        dt=clip(dt,.005,.15);peers=[o for o in objects if o.ident!=self_id]
        pose,es,lat,he,steer,k=self.track(ego,dt);distance=math.dist((ego.x,ego.y),self.goal)
        flags=2 if self.phase!='PASS' else 0
        def cmd(mode,target=0.,throttle=0.,brake=0.,lead=None,ttc=float('inf')):
            return BatchCommand(steer,throttle,brake,target,mode,lat,distance,lead,ttc,signal_lights=flags|(8 if brake>.02 else 0),route_s=es)
        if abs(lat)>.65 or abs(he)>.35:return cmd('CHANGE_LANE_FAILSAFE',brake=1.)
        actor=self.require_actor(ego,peers,dt,lambda o:o.kind in VEHICLE_TYPES and -485<o.x<-465 and -20<o.y<-16)
        if actor is None:return cmd('CHANGE_WAIT_ACTOR',brake=1.)
        if any(not -1.68<self.coordinates(x,y)[1]<5.18 for x,y in self.rectangle(*pose,4.57,2.21,1.7)):
            return cmd('CHANGE_EDGE_FAILSAFE',brake=1.)
        self.signal_s+=dt
        if self.phase=='PRE_SIGNAL' and self.signal_s>=1 and self.left_clear(ego,peers) and self.plan_change(ego,peers):
            self.phase='LANE_CHANGE'
            pose,es,lat,he,steer,k=self.track(ego,dt)
        if self.phase=='LANE_CHANGE':
            s,l=self.coordinates(*pose[:2])
            if s>=self.change_start+self.change_length and abs(l-3.5)<.15 and abs(wrap(ego.yaw-self.heading))<.05:
                self.phase='PASS';flags=0;self.integral=0.
        collision,t=self.collision(pose,ego.speed,steer,peers)
        if collision:return cmd('CHANGE_AEB',brake=1.,lead=collision,ttc=t)
        if distance<1:return cmd('CHANGE_GOAL_HOLD',brake=.7)
        target=self.cruise if self.phase=='PASS' else 4.
        if self.phase=='PRE_SIGNAL':
            target,_,lead,ttc,urgent=self.following(ego,pose,es,peers,target,buffer=12.)
            if urgent:return cmd('CHANGE_FOLLOW_AEB',brake=1.,lead=lead,ttc=ttc)
        else:lead,ttc=None,float('inf')
        throttle,brake=self.pedals(ego,target,dt)
        return cmd('CHANGE_'+self.phase,target,throttle,brake,lead,ttc)


class JunctionMergeController(BatchControllerBase):
    def __init__(self):
        start,goal,h,_=BATCH_PROFILES[31]
        super().__init__(31,BatchRoute.straight(start,h,math.dist(start,goal)))
        self.entered=self.last_green=False
        self.approach_cap=4.
        self.previous_speed=None
        self.feedback_throttle=0.
        self.restart_required=False

    def line_gap(self,ego):
        dx,dy=-19.42804928631577-ego.x,7.09341047799558-ego.y
        yaw=wrap(ego.yaw-self.heading)
        extent=(1.7+4.57/2-self.gps_to_rear)*math.cos(yaw)+EGO_WIDTH_M/2*abs(math.sin(yaw))
        return dx*self.c+dy*self.s-extent

    def update(self,ego,objects,dt,self_id=None,*,light=None):
        if not all(math.isfinite(v) for v in (ego.x,ego.y,ego.yaw,ego.speed,dt)) or ego.speed<0:raise ValueError('Invalid case 31 state')
        dt=clip(dt,.005,.15)
        accel=0. if self.previous_speed is None else (ego.speed-self.previous_speed)/dt
        self.previous_speed=ego.speed
        light=light or Case10Light();pose,es,lat,he,steer,k=self.track(ego,dt)
        peers=[o for o in objects if o.ident!=self_id];distance=math.dist((ego.x,ego.y),self.goal);gap=self.line_gap(ego)
        def cmd(mode,target=0.,throttle=0.,brake=0.,lead=None,ttc=float('inf')):
            self.feedback_throttle=throttle
            if -43.6<=ego.x<=15.4 and ego.speed<.2 and brake>.3:self.restart_required=True
            return BatchCommand(steer,throttle,brake,target,mode,lat,distance,lead,ttc,route_s=es,stopline_gap=gap,
                                light_status=light.status,light_countdown=light.countdown,light_valid=light.valid,light_layout=light.layout)
        if abs(lat)>.65 or abs(he)>.3:return cmd('MERGE_LANE_FAILSAFE',brake=1.)
        if gap<=0 and not self.entered:
            if light.green or self.last_green:self.entered=True
            else:return cmd('MERGE_ENTRY_FAILSAFE',brake=1.)
        self.last_green=light.green
        actor=self.require_actor(ego,peers,dt,lambda o:o.kind in VEHICLE_TYPES and -2<o.x<8 and -20<o.y<5)
        if actor is None:return cmd('MERGE_WAIT_ACTOR',brake=1.)
        collision,t=self.collision(pose,ego.speed,steer,peers)
        if collision:return cmd('MERGE_AEB',brake=1.,lead=collision,ttc=t)
        if es>=self.goal_s-1:return cmd('MERGE_GOAL_HOLD',brake=.7)
        # Reduce early, before the acceleration-warning region (-43.6..15.4),
        # and approach at a stable low speed. Actor activation is position-based.
        target=min(self.cruise,math.sqrt(4*max(0.,-53.-ego.x)+4.0**2)) if ego.x<-53 else 4.
        if ego.x>20.:target=self.cruise
        mode='MERGE_APPROACH';room=None
        if gap>0 and not light.green:
            room=max(0.,gap-.25);target=min(target,math.sqrt(3*room),.8*room);mode='MERGE_SIGNAL_APPROACH'
            if room<=.06:return cmd('MERGE_SIGNAL_HOLD',brake=.75)
        # The cross vehicle turns INTO our route. Forecast its moving box
        # entering the lane, then switch to ACC once aligned -- never wait for
        # its lateral position to clear an edge it will not cross.
        os,ol=self.coordinates(actor.x,actor.y)
        vlat=-actor.vx*self.s+actor.vy*self.c
        future_entry=abs(ol)<5.5 or ol*vlat<0 and abs(ol)/max(.1,abs(vlat))<5
        if future_entry and abs(wrap(actor.yaw-self.heading))>.35 and os>self.coordinates(ego.x,ego.y)[0]:
            separation=os-self.coordinates(ego.x,ego.y)[0]-(1.7+4.57/2-self.gps_to_rear)-actor.width/2-4.
            if separation<25:
                room=max(0.,separation);target=min(target,math.sqrt(3*room),.8*room);mode='MERGE_YIELD'
                if room<=.06:return cmd('MERGE_YIELD_HOLD',brake=.75)
        target,follow,lead,ttc,urgent=self.following(ego,pose,es,peers,target,buffer=13.)
        if urgent:return cmd('MERGE_FOLLOW_AEB',brake=1.,lead=lead,ttc=ttc)
        if lead:mode='MERGE_FOLLOW'
        if -43.6<=ego.x<=15.4:
            # Keep the requested junction speed non-increasing to avoid
            # unnecessary acceleration after a small merge-speed adjustment.
            # A necessary restart after a full safety stop remains permitted.
            if self.approach_cap<.5 and lead and lead.gap>=13 and abs(wrap(actor.yaw-self.heading))<.2:
                self.approach_cap=min(4.,target)
            self.approach_cap=min(self.approach_cap,target)
            target=self.approach_cap
        throttle,brake=self.pedals(ego,target,dt,room)
        # The exported warning tests *actual positive longitudinal acceleration*,
        # not an increasing target speed. Start this feedback trim BEFORE the
        # region, then request slight deceleration instead of repeatedly using
        # the speed loop's fixed sustaining throttle. 40 is a conservative
        # feedback divisor, not a fitted SDK actuator/drag coefficient. Safety
        # braking has priority. A necessary restart after a red/blocked stop
        # cannot also satisfy a literal "never accelerate" warning condition.
        # Bounds include a body-length reserve on both region edges.
        if -53.<=ego.x<=20. and not self.restart_required and target>.5 and ego.speed>.5 and brake==0.:
            # Frame-rate scaling keeps high-rate feedback from overreacting to
            # a delayed actuator (the offline suite includes a 100ms delay).
            throttle=clip(self.feedback_throttle+(-.06-accel)*clip(dt/.1,.05,1.)/40,0.,.55)
        return cmd(mode,target,throttle,brake,lead,ttc)



# Export 0f556220: eight independent additions. Existing controller definitions
# and constants remain unchanged; exported actor routes are NOT replayed here.
EXPANDED_BUILD = '20261013-batch-20-21-22-26-30-32-33-37'
EXPANDED_PROFILES = {
    20: ((-65.446164209321,1.9596010995756616),(-23.663035,1.854087),-.0025252471575125,25/3.6),
    21: ((-238.55475330634562,-17.412040749556972),(-184.54526,-13.771012),.0026109601243643494,25/3.6),
    22: ((-100.10825044686968,.741577011134867),(100.996809,.2418620834983894),-.005805450013805,25/3.6),
    26: ((-274.80135915088914,-17.506679407114596),(-156.900304,-13.6988318247948),.0026109601243526487,25/3.6),
    30: (CASE35_START,CASE35_GOAL,CASE35_HEADING,25/3.6),
    32: ((-105.601925,2.010499),(-23.663035,1.803582),-.0025252471575125,25/3.6),
    33: (CASE35_START,CASE35_GOAL,CASE35_HEADING,25/3.6),
    37: ((-67.8989565377474,1.9657950195969425),(38.097083,1.698128252175025),-.0025252471575125,25/3.6),
}
EXPANDED_TITLES = {
    20:'停止线识别及响应',21:'左侧车辆通行起步',22:'上坡-下坡路跟车',26:'避让故障车辆变道',
    30:'无信号灯路口非机动车冲突通行',32:'拥堵路口通行',33:'群体行人通行',37:'行人违章通行',
}
# Map 8354e508: lane -1 of roads 20 -> 30 -> 25, exact horizontal geometry
# and elevation pieces. Only controller feed-forward, never a scene/map edit.
EXPANDED_HILL_ROADS = (
    (-300.74054057476167,3.6563803189691955,6.277379857165781,258.3793540995452,),
    (-42.365540574958615,2.1563803189768427,6.277379857165766,100.00056789593559,100.00031249943675,0.00138506453825471,-0.0019313674593064434,-5.551115123125783e-16,0.213957073119725,0.11658888012087476,),
    (57.63445942496643,1.906380318978103,0.0019714119397970177,253.62549285295785,),
)
EXPANDED_HILL_ELEVATION = (
    (0,0,0.04145638292307272,0.024556063098222055,0.016900413451976213,),
    (0.9999983956107146,0.0829126328217923,0.0706347362689176,0.02398051541544993,-0.004695663252771484,),
    (2.9999956887616923,0.2825385626805368,0.11020882759345017,0.005720570046394082,-0.0002613429874920429,),
    (4.999993599032173,0.5237474825823124,0.12995493844457706,-0.01568540559638021,0.010180308899550076,),
    (6.999992065329994,0.8023579178345004,0.12625121730781202,0.018885548114864335,-0.002586178933007561,),
    (9.99999067791968,1.2812544361047025,0.16973763465084454,-0.007139073306003984,0.0035025181804323674,),
    (12.999990215523066,1.8207835687673528,0.16610332570712247,-0.032345800588294976,0.010037764855034974,),
    (16.99999072228081,2.6100812101099753,0.1415089440140489,0.018189353733374734,-0.0010770927110033987,),
    (27.999995714280914,4.933981749901755,0.33151775577366327,-0.047499908222045306,0.004169497805037155,),
    (32.99999830842914,5.925260487859005,0.21153887922340336,-0.010300605048408186,0.00030760829468660933,),
    (36.99999999605549,6.626293497686156,0.19186564357452723,-0.02340284592442946,0.003155176081533044,),
    (40.00000088634834,7.076454690939388,0.1366383497859132,0.0007736491127584692,-0.0017484777704959088,),
    (43.00000136684139,7.44612372770919,0.14110701375750223,-0.03794900075858104,0.008042625705767109,),
    (45.000001427264614,7.640882763021679,0.08582253015127228,-0.005255813848297603,-0.00026134223333707006,),
    (47.00000126132873,7.789413819832204,0.06166317758808935,-0.003943482695400694,-0.0011788509910221848,),
    (49.00000085650136,7.887535423448153,0.06348609355222341,-0.06311729793441402,0.02848175709864683,),
    (50.0000005613979,7.9163859694667185,0.015125233012213394,0.005510835332873463,-0.005493652884832357,),
    (51.50058895464821,7.93292892165482,-0.0040868780966686205,-0.0045529594484615315,-0.0008131128622780888,),
    (53.500587601737024,7.90003846813841,-0.03205603240542728,-0.008431225930112085,0.000312911705729256,),
    (55.50058701698632,7.804704829522686,-0.062025967433527784,-0.007805397527312719,0.00031291139457869776,),
    (57.500587095150244,7.651934588707913,-0.08949259664576108,-0.007179570688188869,0.000312911131409963,),
    (59.5005877340036,7.44673432859435,-0.11445592618278827,-0.006553745200608022,0.0003129109147757233,),
    (61.500588834403146,7.194110632082262,-0.13691596127493347,-0.005927920859802954,0.00031291074320046163,),
    (63.500590300348755,6.899070082071912,-0.15687270627770306,0.01882462056782997,-0.011750437915758125,),
    (65.50059203904475,6.566619261463559,-0.14838642348033768,-0.02193464918625214,0.003137151705900609,),
    (68.50059496355685,6.008750673274103,-0.195291177556616,0.020878087679368416,-0.007996594545434254,),
    (71.50059801303087,5.394871005053119,-0.1715586249148425,-0.014440827190904955,0.0009368541769471193,),
    (76.5006026458258,4.293162834533824,-0.20475243677305346,-0.013908592450417248,0.0017310282677408468,),
    (82.50060608115703,2.9378403569915936,-0.2770567062664557,0.022573612489641,-0.0017825347678014551,),
    (86.50060629019977,2.0767090685687934,-0.24270607995193058,0.026099130894521646,-0.003773287341291992,),
    (89.50060504191062,1.4816044832147437,-0.18799011970187846,0.0025206835124645032,0.0003129122539438189,),
    (92.50060241280876,0.9487693388478684,-0.16441749899482622,-0.002511967710532854,0.0023033790337947603,),
    (95.50059826631957,0.4951008527565506,-0.17594741290430604,0.0449246124094804,-0.009740489279767447,),
    (97.50059460106188,0.24498097701586197,-0.11313477824392255,-0.008056563068110369,0.0070096626247453785,),
    (99.50059018066707,0.042562739981631736,-0.24499193846019196,0.554346112312895,-0.46923400895628414,),
)


def expanded_pose_report(ego,number):
    start,_,h,_=EXPANDED_PROFILES[number]
    c,s=math.cos(h),math.sin(h);dx,dy=ego.x-start[0],ego.y-start[1]
    along,lat=dx*c+dy*s,-dx*s+dy*c
    reasons=[]
    if not -8<=along<=8 or abs(lat)>.65:reasons.append('position_offset_or_late_start')
    if abs(wrap(ego.yaw-h))>.15:reasons.append('heading_offset')
    return (f'START_POSE case={number}: actual=({ego.x:.6f},{ego.y:.6f},{ego.yaw:.6f}); '
            f'speed={ego.speed:.3f}; along={along:.3f}; lateral={lat:.3f}; '
            f'result={"REJECT" if reasons else "ACCEPT"}; reasons={",".join(reasons) or "none"}',tuple(reasons))


def expanded_hill_route():
    coords=[]
    for g in EXPANDED_HILL_ROADS:
        x,y,h,length=g[:4];n=math.ceil(length/.1)
        for i in range(n+1):
            p=i/n
            if len(g)==4:u,v,du,dv=length*p,0.,length,0.
            else:
                bu,cu,eu,bv,cv,ev=g[4:]
                u,v=bu*p+cu*p*p+eu*p**3,bv*p+cv*p*p+ev*p**3
                du,dv=bu+2*cu*p+3*eu*p*p,bv+2*cv*p+3*ev*p*p
            yaw=h+math.atan2(dv,du)
            coords.append((x+u*math.cos(h)-v*math.sin(h)+1.75*math.sin(yaw),
                           y+u*math.sin(h)+v*math.cos(h)-1.75*math.cos(yaw),1.75))
    start,goal,_,_=EXPANDED_PROFILES[22]
    first=min(range(len(coords)),key=lambda i:math.dist(coords[i][:2],start))
    last=min(range(first,len(coords)),key=lambda i:math.dist(coords[i][:2],goal))
    return BatchRoute(coords[max(0,first-80):last+550])


def expanded_hill_grade(x,y):
    g=EXPANDED_HILL_ROADS[1]
    s=(x-g[0])*math.cos(g[2])+(y-g[1])*math.sin(g[2])
    if not 0<=s<=g[3]:return 0.
    e=next(e for e in reversed(EXPANDED_HILL_ELEVATION) if e[0]<=s)
    ds=s-e[0]
    return clip(e[2]+2*e[3]*ds+3*e[4]*ds*ds,-.4,.4)


class ExpandedBase(BatchControllerBase):
    def __init__(self,number,route=None):
        from ParkingAPA import rectangle,overlap,integrate
        self.rectangle,self.overlap,self.integrate=rectangle,overlap,integrate
        self.number=number
        self.start,self.goal,self.heading,self.cruise=EXPANDED_PROFILES[number]
        self.c,self.s=math.cos(self.heading),math.sin(self.heading)
        self.route=route or BatchRoute.straight(self.start,self.heading,math.dist(self.start,self.goal)+10)
        self.wheelbase,self.gps_to_rear=WHEELBASE_M,0.
        self.geometry_reader=None;self.measured=False
        self.integral=self.previous_steer=self.missing_s=0.
        self.actor_id=None;self.roles={};self.last_roles={}
        self.goal_s=self.route.project(*self.goal)[0]

    def actors(self,peers,dt,specs):
        for role,x,y,kinds in specs:
            if role in self.roles:continue
            candidates=[o for o in peers if o.kind in kinds and abs(o.x-x)<10 and abs(o.y-y)<2
                        and o.ident not in self.roles.values()]
            if len(candidates)==1:self.roles[role]=candidates[0].ident
        if self.roles!=self.last_roles:
            print(f'CASE{self.number}_ACTORS: {self.roles}; live_data_required=True.',flush=True)
            self.last_roles=dict(self.roles)
        observed={o.ident:o for o in peers}
        missing=[r for r,_,_,_ in specs if r not in self.roles or self.roles[r] not in observed]
        self.missing_s=self.missing_s+dt if missing else 0.
        if self.missing_s>5:raise RuntimeError(f'Case {self.number} actors missing: {missing}; no timer release')
        return {r:observed[self.roles[r]] for r,_,_,_ in specs if r not in missing},missing

    def state(self,ego,dt):
        if not all(math.isfinite(v) for v in (ego.x,ego.y,ego.yaw,ego.speed,dt)) or ego.speed<0:
            raise ValueError(f'Invalid case {self.number} state')
        return clip(dt,.005,.15)


class ExpandedLightReader(Case10LightReader):
    def __init__(self,sender,number):
        super().__init__(sender);self.number=number

    @classmethod
    def from_sdk(cls,number):
        from SimOneSensorAPI import SoGetTrafficLights
        return cls(SoGetTrafficLights,number)

    def read(self):
        data=ctypes.create_string_buffer(b'\xff'*64,64)
        light=self.decode(data.raw) if self.sender('Ego',52,data) else Case10Light(layout='API_false')
        diagnostic=(light.valid,light.status,light.layout)
        if diagnostic!=self.last_diagnostic:
            print(f'CASE{self.number}_LIGHT: id=52; valid={light.valid}; status={light.status}; layout={light.layout}.',flush=True)
            self.last_diagnostic=diagnostic
        return light


class HillFollowingController(ExpandedBase):
    def __init__(self):super().__init__(22,expanded_hill_route())

    def update(self,ego,objects,dt,self_id=None):
        dt=self.state(ego,dt);pose,es,lat,he,steer,k=self.track(ego,dt)
        peers=[o for o in objects if o.ident!=self_id]
        def cmd(mode,target=0.,throttle=0.,brake=0.,lead=None,ttc=float('inf')):
            return BatchCommand(steer,throttle,brake,target,mode,lat,math.dist((ego.x,ego.y),self.goal),lead,ttc,route_s=es)
        if abs(lat)>.45 or abs(he)>.3:return cmd('HILL_LANE_FAILSAFE',brake=1.)
        actors,missing=self.actors(peers,dt,(('lead',-70.3968641093,.5690871051,VEHICLE_TYPES),))
        if missing:return cmd('HILL_WAIT_ACTOR',brake=1.)
        collision,t=self.collision(pose,ego.speed,steer,peers)
        if collision:return cmd('HILL_AEB',brake=1.,lead=collision,ttc=t)
        if es>=self.goal_s-1:return cmd('HILL_GOAL_HOLD',brake=.75)
        target,mode,lead,ttc,urgent=self.following(ego,pose,es,peers,self.cruise,buffer=18.)
        if urgent:return cmd('HILL_DISTANCE_AEB',brake=1.,lead=lead,ttc=ttc)
        error=target-ego.speed;self.integral=clip(self.integral+error*dt,-5.,5.)
        # Map grade supplies gravity compensation. Live feedback corrects engine
        # and drag differences; the SDK's gear and pedal modes remain untouched.
        grade=expanded_hill_grade(*pose[:2])
        desired=.9*error+.32*self.integral+9.81*grade+.18+.035*ego.speed
        throttle,brake=(clip(desired/5,0.,.95),0.) if desired>=0 else (0.,clip(-desired/7,0.,1.))
        return cmd('HILL_'+mode,target,throttle,brake,lead,ttc)


class ExpandedLaneChangeController(ExpandedBase):
    def __init__(self,number):
        super().__init__(number)
        self.phase='WAIT_LEFT' if number==21 else 'PRE_SIGNAL'
        self.signal_s=0.;self.change_start=self.change_length=None
        self.left_clear=SlowVehicleLaneChangeController.left_clear.__get__(self)

    def plan_change(self,ego,peers):
        pose=self.rear_pose(ego);start_s,start_l=self.coordinates(*pose[:2]);goal_s,_=self.coordinates(*self.goal)
        lengths=(18.,20.,24.,28.) if self.number==21 else (24.,28.,32.,36.)
        for length in lengths:
            if start_s+length>goal_s-8:continue
            coords=[];safe=True;speed=max(2.,min(4.,ego.speed))
            for i in range(math.ceil((goal_s+10-start_s)/.12)+1):
                s=start_s+i*.12;u=clip((s-start_s)/length,0.,1.)
                f=10*u**3-15*u**4+6*u**5
                derivative=(3.5-start_l)*(30*u*u-60*u**3+30*u**4)/length
                yaw=self.heading+math.atan(derivative);x,y=self.world(s,start_l+(3.5-start_l)*f)
                body=self.rectangle(x,y,yaw,4.77,2.41,1.7);t=(s-start_s)/speed
                if any(not -1.68<self.coordinates(bx,by)[1]<5.18 for bx,by in body):
                    safe=False;break
                if any(self.overlap(body,self.rectangle(o.x+o.vx*t,o.y+o.vy*t,o.yaw,o.length+.25,o.width+.25)) for o in peers):
                    safe=False;break
                coords.append((x,y,1.75))
            if safe:
                self.route=BatchRoute(coords);self.goal_s=self.route.project(*self.goal)[0]
                self.change_start,self.change_length=start_s,length
                print(f'CASE{self.number}_PLAN: start={start_s:.3f}; length={length}; swept_route_valid=True.',flush=True)
                return True
        return False

    def update(self,ego,objects,dt,self_id=None):
        dt=self.state(ego,dt);peers=[o for o in objects if o.ident!=self_id]
        pose,es,lat,he,steer,k=self.track(ego,dt);distance=math.dist((ego.x,ego.y),self.goal)
        flags=2 if self.phase!='PASS' else 0
        def cmd(mode,target=0.,throttle=0.,brake=0.,lead=None,ttc=float('inf')):
            return BatchCommand(steer,throttle,brake,target,mode,lat,distance,lead,ttc,
                                signal_lights=flags|(8 if brake>.02 else 0),route_s=es)
        if abs(lat)>.65 or abs(he)>.35:return cmd('LANE_LANE_FAILSAFE',brake=1.)
        specs=(('front',-210.8503089115,-17.3397053856,VEHICLE_TYPES),('left',-240.6494453993,-13.9174979895,VEHICLE_TYPES)) if self.number==21 else (('front',-191.8042405865,-17.2899767477,VEHICLE_TYPES),)
        actors,missing=self.actors(peers,dt,specs)
        if missing:return cmd('LANE_WAIT_ACTORS',brake=1.)
        if self.number==26:
            # This exported static sign may be absent from SDK GroundTruth.
            # Keep its real, immutable exported footprint in the planner.
            if not any(math.dist((o.x,o.y),(-241.65,-17.6135350681))<1 for o in peers):
                peers.append(Obstacle(-26001,7,-241.65,-17.6135350681,0,0,0,1.2,.5))
        if any(not -1.68<self.coordinates(x,y)[1]<5.18 for x,y in self.rectangle(*pose,4.57,2.21,1.7)):
            return cmd('LANE_EDGE_FAILSAFE',brake=1.)
        self.signal_s+=dt
        if self.phase=='WAIT_LEFT':
            if ego.speed>.05 or not self.left_clear(ego,peers):
                return cmd('LANE_WAIT_LEFT_PASS',brake=1.)
            self.phase='PRE_SIGNAL'
        if self.phase=='PRE_SIGNAL' and self.signal_s>=1 and self.left_clear(ego,peers) and self.plan_change(ego,peers):
            self.phase='LANE_CHANGE';pose,es,lat,he,steer,k=self.track(ego,dt)
        if self.phase=='LANE_CHANGE':
            s,l=self.coordinates(*pose[:2])
            if s>=self.change_start+self.change_length and abs(l-3.5)<.15 and abs(wrap(ego.yaw-self.heading))<.05:
                self.phase='PASS';flags=0;self.integral=0.
        collision,t=self.collision(pose,ego.speed,steer,peers)
        if collision:return cmd('LANE_AEB',brake=1.,lead=collision,ttc=t)
        if distance<1:return cmd('LANE_GOAL_HOLD',brake=.75)
        target=self.cruise if self.phase=='PASS' else 3. if self.number==21 else 4.
        lead=None;ttc=float('inf')
        if self.phase=='PRE_SIGNAL':
            target,_,lead,ttc,urgent=self.following(ego,pose,es,peers,target,buffer=13.)
            if urgent:return cmd('LANE_FOLLOW_AEB',brake=1.,lead=lead,ttc=ttc)
        throttle,brake=self.pedals(ego,target,dt)
        return cmd('LANE_'+self.phase,target,throttle,brake,lead,ttc)


class ExpandedVRUController(VRUYieldController):
    def __init__(self,number):
        super().__init__();self.number=number;self.goal=CASE35_GOAL
        # Known roles are learned from live GT at the scene's initial anchors.
        # Movement directions distinguish the third, northbound pedestrian.
        self.specs=(('bike',60.2,7.7034452901,-1,{8,17,21,27,6}),) if number==30 else (
            ('ped_a',59.4,6.0873672697,-1,{4}),('ped_b',61.4,5.3939502347,-1,{4}),
            ('ped_c',60.3,-17.2653397439,1,{4}))

    def crossing_blockers(self,objects,self_id):
        peers=[o for o in objects if o.ident!=self_id]
        for role,x,y,direction,kinds in self.specs:
            if role not in self.roles:
                candidates=[o for o in peers if o.kind in kinds and abs(o.x-x)<.65 and -28<o.y<10
                            and o.ident not in self.roles.values()]
                if len(candidates)==1:self.roles[role]=candidates[0].ident
        if self.roles!=self.last_roles:
            print(f'CASE{self.number}_ACTORS: {self.roles}; all_live_actors_required=True.',flush=True)
            self.last_roles=dict(self.roles)
        observed={o.ident:o for o in peers}
        missing=[r for r,_,_,_,_ in self.specs if r not in self.roles or self.roles[r] not in observed]
        blockers=['missing:'+r for r in missing]
        for role,_,_,direction,_ in self.specs:
            o=observed.get(self.roles.get(role))
            if o is None:continue
            dx,dy=o.x-CASE35_LINE[0],o.y-CASE35_LINE[1];l=-dx*CASE35_S+dy*CASE35_C
            yaw=wrap(o.yaw-CASE35_HEADING);extent=(abs(math.sin(yaw))*o.length+abs(math.cos(yaw))*o.width)/2
            cleared=l-extent>9.4 if direction>0 else l+extent< -9.4
            toward=(-o.vx*CASE35_S+o.vy*CASE35_C)*direction<-.15
            if not cleared or toward:blockers.append(f'{role}:{o.ident}')
        for o in peers:
            if o.ident in self.roles.values() or o.kind not in CASE35_VRU_TYPES:continue
            for t in (0.,1.,2.,3.):
                dx,dy=o.x+o.vx*t-CASE35_LINE[0],o.y+o.vy*t-CASE35_LINE[1]
                if -.5<dx*CASE35_C+dy*CASE35_S<6.5 and abs(-dx*CASE35_S+dy*CASE35_C)<10.5:
                    blockers.append(f'other:{o.ident}');break
        return blockers,missing,peers


class StopResponseController(ExpandedBase):
    """Cases 20/32 finish at the stop line, by the exported 10s standstill."""
    def __init__(self,number):
        super().__init__(number);self.stopped_s=0.;self.red_observed=False;self.queue_observed=False

    def line_gap(self,ego):
        dx,dy=CASE10_LINE_POINT[0]-ego.x,CASE10_LINE_POINT[1]-ego.y
        yaw=wrap(ego.yaw-self.heading)
        front=(1.7+4.57/2-self.gps_to_rear)*math.cos(yaw)+EGO_WIDTH_M/2*abs(math.sin(yaw))
        return dx*self.c+dy*self.s-front

    def stop_confirmed(self):
        return self.stopped_s>=9.7 and (self.red_observed if self.number==20 else self.queue_observed)

    def update(self,ego,objects,dt,self_id=None,*,light=None):
        dt=self.state(ego,dt);light=light or Case10Light()
        pose,es,lat,he,steer,k=self.track(ego,dt);gap=self.line_gap(ego)
        peers=[o for o in objects if o.ident!=self_id]
        def cmd(mode,target=0.,throttle=0.,brake=0.,lead=None,ttc=float('inf')):
            return BatchCommand(steer,throttle,brake,target,mode,lat,math.dist((ego.x,ego.y),self.goal),lead,ttc,
                                route_s=es,stopline_gap=gap,light_status=light.status,light_countdown=light.countdown,
                                light_valid=light.valid,light_layout=light.layout)
        if abs(lat)>.65 or abs(he)>.3:return cmd('STOP_LANE_FAILSAFE',brake=1.)
        if self.number==32:
            specs=tuple((r,x,y,VEHICLE_TYPES) for r,x,y in (
                ('queue_1',18.3441082766,1.7480095024),('queue_2',28.3513401924,1.7227387147),('queue_3',38.6996220684,1.6966066898)))
            actors,missing=self.actors(peers,dt,specs)
            if missing:return cmd('STOP_WAIT_QUEUE',brake=1.)
            self.queue_observed=True
        elif light.valid and light.status==1:self.red_observed=True
        if gap<-.03:return cmd('STOP_LINE_FAILSAFE',brake=1.)
        if .03<gap<.5 and ego.speed<.02:self.stopped_s+=dt
        else:self.stopped_s=0.
        collision,t=self.collision(pose,ego.speed,steer,peers)
        if collision:return cmd('STOP_AEB',brake=1.,lead=collision,ttc=t)
        room=max(0.,gap-.25)
        if room<=.06:return cmd('STOP_RED_HOLD' if self.number==20 else 'STOP_CONGESTION_HOLD',brake=.75)
        target=min(self.cruise,math.sqrt(3*room),.8*room)
        throttle,brake=self.pedals(ego,target,dt,room)
        return cmd('STOP_LINE_APPROACH',target,throttle,brake)


class IllegalPedestrianController(ExpandedBase):
    def __init__(self):
        super().__init__(37);self.line_still_s=self.clear_s=0.
        self.stopped_at_line=self.released=False

    def line_gap(self,ego):
        return StopResponseController.line_gap(self,ego)

    def update(self,ego,objects,dt,self_id=None,*,light=None):
        dt=self.state(ego,dt);light=light or Case10Light()
        pose,es,lat,he,steer,k=self.track(ego,dt);gap=self.line_gap(ego)
        peers=[o for o in objects if o.ident!=self_id]
        actors,missing=self.actors(peers,dt,(('ped',-16.415349041,17.4705603595,{4}),))
        blockers=[]
        if not missing:
            ped=actors['ped'];dy=ped.y-CASE10_LINE_POINT[1];dx=ped.x-CASE10_LINE_POINT[0]
            l=-dx*self.s+dy*self.c;yaw=wrap(ped.yaw-self.heading)
            extent=(abs(math.sin(yaw))*ped.length+abs(math.cos(yaw))*ped.width)/2
            if l+extent>=-9.4 or -ped.vx*self.s+ped.vy*self.c>.15:blockers.append(str(ped.ident))
        for o in peers:
            if o.ident in self.roles.values() or o.kind not in CASE35_VRU_TYPES:continue
            for t in (0.,1.,2.,3.):
                os,ol=self.coordinates(o.x+o.vx*t,o.y+o.vy*t)
                line_s,_=self.coordinates(*CASE10_LINE_POINT)
                if -.5<os-line_s<6.5 and -9.5<ol<16.:blockers.append(str(o.ident));break
        def cmd(mode,target=0.,throttle=0.,brake=0.,lead=None,ttc=float('inf')):
            return BatchCommand(steer,throttle,brake,target,mode,lat,math.dist((ego.x,ego.y),self.goal),lead,ttc,
                                route_s=es,stopline_gap=gap,light_status=light.status,light_countdown=light.countdown,
                                light_valid=light.valid,light_layout=light.layout,yield_ids=','.join(blockers))
        if abs(lat)>.65 or abs(he)>.3:return cmd('PED_LANE_FAILSAFE',brake=1.)
        if missing:self.clear_s=0.;return cmd('PED_WAIT_ACTOR',brake=1.)
        if not self.stopped_at_line and gap<-.03:return cmd('PED_STOPLINE_FAILSAFE',brake=1.)
        if not self.stopped_at_line and .03<gap<.5 and ego.speed<.05:
            self.line_still_s+=dt
            if self.line_still_s>=.3:self.stopped_at_line=True
        else:self.line_still_s=0.
        self.clear_s=self.clear_s+dt if not blockers and light.green else 0.
        if self.stopped_at_line and self.clear_s>=.25:self.released=True
        collision,t=self.collision(pose,ego.speed,steer,peers)
        if collision:return cmd('PED_AEB',brake=1.,lead=collision,ttc=t)
        if es>=self.goal_s-1:return cmd('PED_GOAL_HOLD',brake=.75)
        must_stop=not self.released or gap>0 and (blockers or not light.green)
        room=max(0.,gap-.25) if must_stop else None
        if room is not None and room<=.06:return cmd('PED_YIELD_HOLD',brake=.75)
        target=min(20/3.6,math.sqrt(3*room),.8*room) if room is not None else 20/3.6
        throttle,brake=self.pedals(ego,target,dt,room)
        return cmd('PED_APPROACH_STOP' if must_stop else 'PED_CLEAR_GO',target,throttle,brake)


def expanded_controller(number):
    if number==22:return HillFollowingController()
    if number in (21,26):return ExpandedLaneChangeController(number)
    if number in (30,33):return ExpandedVRUController(number)
    if number in (20,32):return StopResponseController(number)
    if number==37:return IllegalPedestrianController()
    raise RuntimeError(f'Unsupported expanded case {number}')



# Case 40: supplied 4cc3cd68 export. Map-backed Ego route only; no actor clock,
# scripted actor path, fixed actor ID or "wait N seconds then drive" in control.
CASE40_START = (-137.42809825923598,2.1413736603078375)
CASE40_GOAL = (82.75077,1.585366)
CASE40_HEADING = -.0025252471575125
CASE40_CRUISE = 30/3.6
CASE40_BUILD = '20261014-live-sudden-conflict'


def case40_pose_report(ego):
    dx,dy=ego.x-CASE40_START[0],ego.y-CASE40_START[1]
    c,s=math.cos(CASE40_HEADING),math.sin(CASE40_HEADING)
    along,lat=dx*c+dy*s,-dx*s+dy*c
    reasons=[]
    if not -8<=along<=10 or abs(lat)>.65:reasons.append('position_offset_or_late_start')
    if abs(wrap(ego.yaw-CASE40_HEADING))>.15:reasons.append('heading_offset')
    return (f'START_POSE case=40: actual=({ego.x:.6f},{ego.y:.6f},{ego.yaw:.6f}); '
            f'speed={ego.speed:.3f}; along={along:.3f}; lateral={lat:.3f}; '
            f'result={"REJECT" if reasons else "ACCEPT"}; reasons={",".join(reasons) or "none"}',tuple(reasons))


@dataclass(frozen=True)
class Case40Command(BatchCommand):
    conflict_id: int | None = None
    cross_enter: float = float('inf')
    cross_leave: float = float('inf')
    ego_enter: float = float('inf')


class SuddenConflictController(BatchControllerBase):
    """Live box occupancy + cut-in prediction, followed by ACC after merging.

    A bicycle may be initially occluded: do not require its fixed spawn position
    before Ego can reach the position-based actor trigger. Once a relevant
    hazard has been seen, disappearance never acts as permission to accelerate.
    """
    def __init__(self):
        from ParkingAPA import rectangle,overlap,integrate
        self.rectangle,self.overlap,self.integrate=rectangle,overlap,integrate
        self.number=40;self.start,self.goal,self.heading,self.cruise=CASE40_START,CASE40_GOAL,CASE40_HEADING,CASE40_CRUISE
        self.c,self.s=math.cos(self.heading),math.sin(self.heading)
        self.route=BatchRoute.straight(self.start,self.heading,math.dist(self.start,self.goal)+25)
        self.goal_s=self.route.project(*self.goal)[0]
        self.wheelbase,self.gps_to_rear=WHEELBASE_M,0.
        self.geometry_reader=None;self.measured=False
        self.integral=self.previous_steer=self.missing_s=0.
        self.actor_id=None;self.seen={};self.yielding={};self.clear_times={};self.last_yields=None
        self.blocked_s=self.blink_s=self.plan_cooldown=0.
        self.bypass_pending=None;self.bypass=None

    def geometry(self,ego,obj):
        es=self.route.project(*self.rear_pose(ego)[:2],key='risk_ego')[0]
        os,l,p,_=self.route.project(obj.x,obj.y,key=('risk_object',obj.ident))
        yaw=wrap(obj.yaw-p.yaw)
        hs=(abs(math.cos(yaw))*obj.length+abs(math.sin(yaw))*obj.width)/2
        hl=(abs(math.sin(yaw))*obj.length+abs(math.cos(yaw))*obj.width)/2
        va=obj.vx*math.cos(p.yaw)+obj.vy*math.sin(p.yaw)
        vl=-obj.vx*math.sin(p.yaw)+obj.vy*math.cos(p.yaw)
        return os-es,l,hs,hl,va,vl

    def signal_flags(self,ego):
        if self.bypass is None:return 2 if self.bypass_pending is not None else 0
        start,length,return_start,return_length=self.bypass
        s,l=self.coordinates(*self.rear_pose(ego)[:2])
        if s<start+length:return 2
        if s>=return_start-3 and (s<return_start+return_length or abs(l)>.15):return 4
        return 0

    def plan_bypass(self,ego,peers):
        pose=self.rear_pose(ego);start,lateral=self.coordinates(*pose[:2])
        # The incoming approach and junction must stay in lane. The exit's
        # same-direction divider is broken; wait until the full body is out.
        if pose[0]<28 or ego.speed>3.1:return False
        for o in peers:
            os,ol=self.coordinates(o.x,o.y);va=o.vx*self.c+o.vy*self.s
            if abs(ol-3.5)<EGO_WIDTH_M/2+o.width/2+.5 and -30-max(0.,va-ego.speed)*6<os-start<40:
                return False
        lead=next((o for o in peers if o.ident==self.bypass_pending),None)
        if lead is None:return False
        os,_=self.coordinates(lead.x,lead.y);yaw=wrap(lead.yaw-self.heading)
        extent=(abs(math.cos(yaw))*lead.length+abs(math.sin(yaw))*lead.width)/2
        goal_s,_=self.coordinates(*self.goal)
        for length in (18.,20.,22.,24.):
            return_start=max(start+length+4,os+extent+(4.57/2-1.7)+4.)
            return_length=18.;end=max(goal_s+15,return_start+return_length+5)
            coords=[];safe=True
            for i in range(math.ceil((end-start+4)/.12)+1):
                s=start-4+i*.12
                u=clip((s-start)/length,0.,1.);v=clip((s-return_start)/return_length,0.,1.)
                f=lambda a:10*a**3-15*a**4+6*a**5
                df=lambda a:30*a*a-60*a**3+30*a**4
                l=lateral+(3.5-lateral)*f(u)-3.5*f(v)
                derivative=(3.5-lateral)*df(u)/length-3.5*df(v)/return_length
                x,y=self.world(s,l);heading=self.heading+math.atan(derivative)
                body=self.rectangle(x,y,heading,4.77,2.41,1.7)
                if any(not -1.68<self.coordinates(bx,by)[1]<5.18 for bx,by in body):safe=False;break
                age=max(0.,s-start)/max(1.5,min(3.,ego.speed))
                if any(self.overlap(body,self.rectangle(o.x+o.vx*age,o.y+o.vy*age,o.yaw,o.length+.25,o.width+.25)) for o in peers):
                    safe=False;break
                coords.append((x,y,1.75))
            if safe:
                self.route=BatchRoute(coords);self.goal_s=self.route.project(*self.goal)[0]
                self.bypass=(start,length,return_start,return_length)
                print(f'CASE40_BYPASS: actor={lead.ident}; left={length}m; return_start_s={return_start:.3f}; full_swept_route_valid=True.',flush=True)
                return True
        return False

    def travel_time(self,distance,speed):
        if distance<=0:return 0.
        if speed>=self.cruise:return distance/max(.1,speed)
        tc=(self.cruise-speed)/2.;dc=speed*tc+tc*tc
        return (math.sqrt(speed*speed+4*distance)-speed)/2 if distance<=dc else tc+(distance-dc)/self.cruise

    def forecast(self,ego,obj):
        delta,l,hs,hl,va,vl=self.geometry(ego,obj)
        front,rear=1.7+4.57/2,4.57/2-1.7
        if delta+hs+rear+.7<0 or delta-hs-front>110:return None
        corridor=EGO_WIDTH_M/2+hl+.4
        if abs(vl)<.06:
            if abs(l)>corridor:return None
            enter,leave=0.,float('inf');direction=1 if l<0 else -1
        else:
            a,b=sorted(((-corridor-l)/vl,(corridor-l)/vl))
            if b<0:return None
            enter,leave=max(0.,a),b;direction=1 if vl>0 else -1
        if enter>6.:return None
        at_entry=delta+va*enter
        ego_enter=self.travel_time(at_entry-hs-front-.7,ego.speed)
        ego_leave=self.travel_time(at_entry+hs+rear+.7,ego.speed)
        risk=max(enter,ego_enter-.35)<=min(leave,ego_leave+.45)
        return Case18Forecast(obj.ident,enter,leave,ego_enter,ego_leave,
                              delta-hs-front-3.,direction,risk)

    def physically_clear(self,ego,obj,direction):
        delta,l,hs,hl,va,vl=self.geometry(ego,obj)
        passed=delta+hs+(4.57/2-1.7)+.8<0
        far=l-hl>EGO_WIDTH_M/2+.7 if direction>0 else l+hl< -EGO_WIDTH_M/2-.7
        return passed or far and vl*direction>=-.08

    def update(self,ego,objects,dt,self_id=None):
        if not all(math.isfinite(v) for v in (ego.x,ego.y,ego.yaw,ego.speed,dt)) or ego.speed<0:
            raise ValueError('Invalid case 40 state')
        dt=clip(dt,.005,.15);pose,es,lat,he,steer,k=self.track(ego,dt)
        peers=[o for o in objects if o.ident!=self_id]
        for o in peers:
            if not all(math.isfinite(v) for v in (o.x,o.y,o.vx,o.vy,o.yaw,o.length,o.width)) or o.length<=0 or o.width<=0:
                raise ValueError('Invalid case 40 obstacle geometry')
        observed={o.ident:o for o in peers}
        forecasts={o.ident:f for o in peers if (f:=self.forecast(ego,o)) is not None}
        focus=min((f for f in forecasts.values() if f.risk),key=lambda f:f.ego_enter,default=None)
        def cmd(mode,target=0.,throttle=0.,brake=0.,lead=None,ttc=float('inf')):
            return Case40Command(steer,throttle,brake,target,mode,lat,math.dist((ego.x,ego.y),self.goal),lead,ttc,
                                 route_s=es,yield_ids=','.join(map(str,sorted(self.yielding))),
                                 signal_lights=self.signal_flags(ego)|(8 if brake>.02 else 0),
                                 conflict_id=focus.ident if focus else None,
                                 cross_enter=focus.enter if focus else float('inf'),
                                 cross_leave=focus.leave if focus else float('inf'),
                                 ego_enter=focus.ego_enter if focus else float('inf'))
        if abs(lat)>.65 or abs(he)>.3:return cmd('HAZARD_LANE_FAILSAFE',brake=1.)
        if any(not -1.68<self.coordinates(x,y)[1]<5.18 for x,y in self.rectangle(*pose,4.57,2.21,1.7)):
            return cmd('HAZARD_EDGE_FAILSAFE',brake=1.)
        missing=[]
        for ident,o in self.seen.items():
            if ident in observed:continue
            delta,l,hs,hl,va,vl=self.geometry(ego,o)
            inside=abs(l)<EGO_WIDTH_M/2+hl+.5
            entering=l*vl<0 and abs(l)/max(.05,abs(vl))<5
            if ident in self.yielding or -hs-1.<delta<max(25.,ego.speed*3.) and (inside or entering):
                missing.append(ident)
        self.seen.update(observed)
        if missing:
            self.missing_s+=dt
            if self.missing_s>5:raise RuntimeError(f'Case 40 observed hazard lost: {missing}; no blind release')
            self.integral=0.;return cmd('HAZARD_WAIT_OBSERVED_OBJECT',brake=1.)
        self.missing_s=0.
        for o in peers:
            f=forecasts.get(o.ident)
            delta,l,hs,hl,va,vl=self.geometry(ego,o)
            # Turning INTO the lane is not transverse traffic that will always
            # leave it. Switch to ACC as soon as live heading/velocity align.
            merged=abs(wrap(o.yaw-self.heading))<.4 and abs(vl)<.65 and va>.2
            crossing=abs(vl)>.25 or abs(wrap(o.yaw-self.heading))>.5
            if f and f.risk and crossing and not merged and not self.physically_clear(ego,o,f.direction):
                self.yielding.setdefault(o.ident,f.direction)
            if o.ident in self.yielding:
                clear=merged or self.physically_clear(ego,o,self.yielding[o.ident])
                self.clear_times[o.ident]=self.clear_times.get(o.ident,0.)+dt if clear else 0.
                if self.clear_times[o.ident]>=.2:
                    del self.yielding[o.ident];self.clear_times.pop(o.ident,None)
        ids=tuple(sorted(self.yielding))
        if ids!=self.last_yields:
            print(f'CASE40_YIELD: ids={ids}; live_footprint_clear_or_merge_required=True.',flush=True);self.last_yields=ids
        collision,t=self.collision(pose,ego.speed,steer,peers)
        if collision:self.integral=0.;return cmd('HAZARD_AEB',brake=1.,lead=collision,ttc=t)
        if es>=self.goal_s-1:return cmd('HAZARD_GOAL_HOLD',brake=.75)
        # Public map junction extent is -19.53..16.47m. A modest approach cap
        # preserves stopping room for a newly visible hazard. No startup halt.
        target=self.cruise if ego.x>24 else min(self.cruise,math.sqrt(4*max(0.,-40-ego.x)+(25/3.6)**2))
        if self.bypass is not None:target=min(target,3.)
        mode='HAZARD_CAUTION_CRUISE';best=None;ttc=float('inf');room=None
        for o in peers:
            f=forecasts.get(o.ident)
            if f is None:continue
            delta,l,hs,hl,va,vl=self.geometry(ego,o)
            if delta+hs<0:continue
            if not f.risk and f.leave+.5<f.ego_enter and o.ident not in self.yielding:continue
            gap=delta-(1.7+4.57/2)-hs;lead_speed=max(0.,va)
            projected=gap+(va-ego.speed)*f.enter
            reserve=.25*max(0.,ego.speed-lead_speed)+max(0.,ego.speed**2-lead_speed**2)/8.
            candidate=Lead(o.ident,gap,lead_speed,f.enter>.05,f.enter)
            if f.enter<.4 and (gap<1.5 or ego.speed>lead_speed+.2 and gap<reserve+2.):
                self.integral=0.;return cmd('HAZARD_GAP_AEB',brake=1.,lead=candidate,
                                            ttc=max(0.,gap)/max(.1,ego.speed-lead_speed))
            buffer=12.+.5*ego.speed
            planned=max(0.,lead_speed-(buffer-projected)/2.5) if projected<buffer else lead_speed+math.sqrt(.81+3.2*(projected-buffer))-.9
            planned=min(target,planned)
            if o.ident in self.yielding:
                this_room=max(0.,gap-3.)
                room=this_room if room is None else min(room,this_room)
                planned=min(planned,math.sqrt(3*this_room),.8*this_room)
            if best is None or planned<target:
                target=planned;best=candidate;ttc=max(0.,projected)/max(.05,ego.speed-lead_speed) if ego.speed>lead_speed+.05 else float('inf')
                mode='HAZARD_CROSS_YIELD' if o.ident in self.yielding else 'HAZARD_ENTRY_PREDICT' if f.enter>.05 else 'HAZARD_FOLLOW'
        if room is not None and room<=.06:self.integral=0.;return cmd('HAZARD_YIELD_HOLD',brake=.75,lead=best,ttc=ttc)
        # A trajectory endpoint can leave a stopped leader on our route. Do not
        # assume it will disappear or keep moving. Bypass only after live low
        # speed is confirmed, with an indicator lead-in and a collision-tested
        # left-and-return route on the exit's legal same-direction lanes.
        if self.bypass is None:
            leader=observed.get(best.ident) if best else None
            blocked=leader is not None and best.speed<.6 and 0<best.gap<35 and ego.x>28
            self.blocked_s=self.blocked_s+dt if blocked else 0.
            waiting=observed.get(self.bypass_pending)
            if waiting is not None and (waiting.vx*self.c+waiting.vy*self.s>.8 or self.geometry(ego,waiting)[0]<-5):
                self.bypass_pending=None;self.blocked_s=self.blink_s=0.
            if self.blocked_s>=1 and self.bypass_pending is None:self.bypass_pending=leader.ident;self.blink_s=0.
            if self.bypass_pending is not None:
                self.blink_s+=dt;self.plan_cooldown-=dt;target=min(target,3.)
                if self.blink_s>=1 and self.plan_cooldown<=0 and ego.speed<=3.1:
                    self.plan_cooldown=.5
                    if self.plan_bypass(ego,peers):
                        pose,es,lat,he,steer,k=self.track(ego,dt)
                        # Re-evaluate live risk on the new path next frame.
                        return cmd('HAZARD_BYPASS_READY',brake=.5)
        else:mode='HAZARD_BYPASS_RETURN' if self.signal_flags(ego)&4 else 'HAZARD_BYPASS'
        throttle,brake=self.pedals(ego,target,dt,room)
        return cmd(mode,target,throttle,brake,best,ttc)


def pid_alive(pid):
    if pid <= 0:
        return False
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel.OpenProcess.argtypes = [ctypes.c_uint32, ctypes.c_bool, ctypes.c_uint32]
    kernel.OpenProcess.restype = ctypes.c_void_p
    kernel.GetExitCodeProcess.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint32)]
    kernel.GetExitCodeProcess.restype = ctypes.c_bool
    kernel.CloseHandle.argtypes = [ctypes.c_void_p]
    handle = kernel.OpenProcess(0x1000, False, pid)
    if not handle:
        return ctypes.get_last_error() == 5  # Access denied: assume active, never kill.
    try:
        code = ctypes.c_uint32()
        return not kernel.GetExitCodeProcess(handle, ctypes.byref(code)) or code.value == 259
    finally:
        kernel.CloseHandle(handle)


def powershell_executable():
    """Resolve Windows PowerShell without relying on the launcher's PATH."""
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel.GetSystemDirectoryW.argtypes = [ctypes.POINTER(ctypes.c_wchar), ctypes.c_uint32]
    kernel.GetSystemDirectoryW.restype = ctypes.c_uint32
    buffer = ctypes.create_unicode_buffer(32768)
    length = kernel.GetSystemDirectoryW(buffer, len(buffer))
    if not length or length >= len(buffer):
        raise RuntimeError('Cannot resolve Windows system directory; no vehicle control started')
    executable = Path(buffer.value)/'WindowsPowerShell'/'v1.0'/'powershell.exe'
    if not executable.is_file():
        raise RuntimeError(f'Windows PowerShell missing: {executable}; no vehicle control started')
    return str(executable)


def ensure_no_other_controller(directory):
    for name in ('.ParkingAPA.lock', '.TestDrive.lock', '.simone_python_algorithm.pid'):
        path = directory/name
        if path.exists():
            pid = int(path.read_text(encoding='ascii').strip())
            if pid != os.getpid() and pid_alive(pid):
                raise RuntimeError(f'Another controller is active: {name}, PID={pid}')
    # LKA's mutex is private to that program; inspect known script processes too.
    script_paths = [str(directory/name) for name in
                    ('new.py', 'TestDrive.py', 'ParkingAPA.py', 'AutoDrive.py', 'PythonAPISample.py')]
    literals = ','.join("'"+path.replace("'", "''")+"'" for path in script_paths)
    query = ("$paths=@("+literals+"); @(Get-CimInstance Win32_Process -ErrorAction Stop | "
             "Where-Object { $_.Name -in @('python.exe','pythonw.exe') -and $_.CommandLine } | "
             "Where-Object { $line=$_.CommandLine; @($paths | Where-Object { "
             "$line.IndexOf($_,[StringComparison]::OrdinalIgnoreCase) -ge 0 }).Count -gt 0 } | "
             "Select-Object ProcessId) | ConvertTo-Json -Compress")
    executable = powershell_executable()
    try:
        proc = subprocess.run([executable, '-NoProfile', '-NonInteractive', '-Command', query],
                              capture_output=True, text=True, timeout=8,
                              creationflags=subprocess.CREATE_NO_WINDOW)
    except OSError as exc:
        raise RuntimeError(f'Cannot start process-safety check with {executable}; '
                           'no vehicle control started') from exc
    if proc.returncode:
        raise RuntimeError('Cannot verify other controller processes; no vehicle control started')
    processes = (json.loads(proc.stdout) or []) if proc.stdout.strip() else []
    if isinstance(processes, dict):
        processes = [processes]
    for item in processes:
        pid = int(item['ProcessId'])
        if pid != os.getpid():
            raise RuntimeError(f'Another known vehicle controller is active: PID={pid}')


class Tee:
    def __init__(self, console, log):
        self.console, self.log = console, log

    def write(self, text):
        self.log.write(text)
        self.log.flush()
        if self.console:
            try:
                self.console.write(text)
            except (OSError, UnicodeError):
                pass
        return len(text)

    def flush(self):
        self.log.flush()
        if self.console:
            try:
                self.console.flush()
            except OSError:
                pass


def run_live(directory, csv_path, stop_file):
    # Lazy SDK loading: offline tests and --stop never load DLLs or connect.
    old_directory = Path.cwd()
    os.chdir(directory)
    dll_scope = os.add_dll_directory(str(directory))
    from SimOneIOStruct import SimOne_Data_Gps, SimOne_Data_Obstacle, SimOne_Data_Control
    from SimOneIOStruct import SimOne_Data_CaseInfo, ESimOne_Drive_Mode
    from SimOneServiceAPI import SoInitSimOneAPI, SoGetCaseRunStatus, SoAPIGetCaseInfo, SoTerminateSimOneAPI
    from SimOneSensorAPI import SoGetGps, SoGetGroundTruth
    from SimOnePNCAPI import SoSetDriverName, SoSetDriveMode, SoSetDrive
    from SimOneEvaluationAPI import SoInitEvaluationServiceWithLocalData, SoSaveEvaluationRecord
    from ParkingAPA import evaluation_record_path, finalize_evaluation

    def drive(brake=1.0, steering=0.0, throttle=0.0, gear=0, gps=None):
        command = SimOne_Data_Control()
        command.EThrottleMode = command.EBrakeMode = command.ESteeringMode = 0
        command.steering, command.throttle, command.brake = steering, throttle, brake
        command.gear, command.handbrake, command.isManualGear = gear, False, False
        if gps is not None:
            command.timestamp, command.frame = gps.timestamp, gps.frame
        if not SoSetDrive('Ego', command):
            raise RuntimeError('SetDrive rejected command')

    connected = takeover = evaluation = stopped = seen_running = False
    previous_status = None
    last_stamp = first_stamp = last_usable = None
    evaluation_path = None
    evaluation_result = {'saved': False, 'end_reason': None, 'verified': False}
    failure = None
    self_id = None
    controller = CutInController()
    case_number, active_goal, sim_limit = 24, GOAL, 315
    case13_signals = None
    case10_lights = None
    case10_truth = None
    case35_signals = None
    batch_signals = batch_lights = None
    expanded_signals = expanded_lights = expanded_empty_truth = None
    case40_signals = None
    case_validated = False
    initialized = False
    final_distance = float('inf')
    deadline = time.monotonic()+180
    last_print = 0.0
    last_decision = None
    last_gps_wall = last_truth_wall = time.monotonic()
    try:
        with csv_path.open('x', encoding='utf-8-sig', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(('sim_s', 'timestamp_ms', 'frame', 'mode', 'x', 'y', 'yaw_deg',
                             'speed_mps', 'target_mps', 'steer', 'throttle', 'brake', 'gear_cmd',
                             'lead_id', 'gap_m', 'lead_mps', 'predicted_cutin', 'ttc_s',
                             'lateral_m', 'goal_distance_m', 'gt_count', 'gt_timestamp_ms',
                             'cutin_entry_s', 'projected_gap_m', 'light_id', 'light_status',
                             'light_countdown_raw', 'light_valid', 'light_layout', 'stopline_gap_m',
                             'gt_source', 'vru_blockers', 'route_s', 'conflict_id',
                             'cross_enter_s', 'cross_leave_s', 'ego_cross_enter_s', 'yield_ids'))
            print('Connecting to Ego; Init return alone is not an initialization verdict.', flush=True)
            SoInitSimOneAPI('Ego', 0, '127.0.0.1')
            connected = True
            last_gps_wall = last_truth_wall = time.monotonic()
            while True:
                now = time.monotonic()
                status = int(SoGetCaseRunStatus())
                if status != previous_status:
                    print(f'Case status={status}', flush=True)
                    previous_status = status
                if status == 1 and seen_running:
                    stopped = True
                    print('Platform stopped the case.', flush=True)
                    break
                if stop_file.exists() and stop_file.read_text(encoding='ascii').strip() == str(os.getpid()):
                    raise RuntimeError('PID-specific graceful stop requested')
                if status != 2:
                    if not seen_running and now > deadline:
                        raise RuntimeError('No Running case after 180 wall seconds')
                    last_gps_wall = last_truth_wall = now
                    time.sleep(LOOP_WALL_S)
                    continue
                seen_running = True
                gps = SimOne_Data_Gps()
                valid = bool(SoGetGps('Ego', gps)) and not gps.isGPSLost
                fresh = valid and int(gps.timestamp) != last_stamp
                if not fresh:
                    if takeover and now-last_gps_wall > 0.35:
                        drive(gps=last_usable)
                    if now-last_gps_wall > 5:
                        raise RuntimeError('GPS missing/stale for 5 wall seconds; no blind throttle')
                    time.sleep(LOOP_WALL_S)
                    continue
                if not all(math.isfinite(v) for v in (gps.posX, gps.posY, gps.oriZ, gps.velX, gps.velY)):
                    raise RuntimeError('Non-finite GPS; braking')
                if last_stamp is not None and gps.timestamp < last_stamp:
                    raise RuntimeError('GPS timestamp reset; refusing to control a new case')
                dt = 0.02 if last_stamp is None else (gps.timestamp-last_stamp)/1000.0
                last_stamp, last_usable, last_gps_wall = int(gps.timestamp), gps, now
                ego = EgoState(float(gps.posX), float(gps.posY), float(gps.oriZ),
                               math.hypot(gps.velX, gps.velY), int(gps.timestamp), int(gps.frame))
                final_distance = math.hypot(active_goal[0]-ego.x, active_goal[1]-ego.y)
                if not case_validated:
                    info = SimOne_Data_CaseInfo()
                    if not SoAPIGetCaseInfo(info):
                        if now > deadline:
                            raise RuntimeError('Case info unavailable')
                        time.sleep(LOOP_WALL_S)
                        continue
                    name = bytes(info.caseName).decode('utf-8', errors='replace')
                    print(f'Startup case={name}; status={status}; API takeover not yet requested.', flush=True)
                    case_number = supported_case(name)
                    if case_number == 13:
                        controller = ObstacleStartController()
                        active_goal, sim_limit = CASE13_GOAL, 95
                        pose_report, pose_reasons = case13_pose_report(ego)
                    elif case_number == 10:
                        controller = TrafficLightController()
                        active_goal, sim_limit = CASE10_GOAL, 90
                        pose_report, pose_reasons = case10_pose_report(ego)
                        case10_lights = Case10LightReader.from_sdk()
                        case10_truth = Case10GroundTruthPolicy()
                        print(f'CASE10_BUILD={CASE10_BUILD}; empty-scene GT gate fixed.', flush=True)
                    elif case_number == 35:
                        controller = VRUYieldController()
                        active_goal, sim_limit = CASE35_GOAL, 90
                        pose_report, pose_reasons = case35_pose_report(ego)
                        case35_signals = Case35SignalWriter.from_sdk()
                        print(f'CASE35_BUILD={CASE35_BUILD}; route=168/-2->216/-1->211/-2; '
                              f'path_samples={len(controller.points)}; live_ped_bike_required=True.', flush=True)
                    elif case_number == 18:
                        controller = CrossTrafficController()
                        active_goal, sim_limit = CASE18_GOAL, 90
                        pose_report, pose_reasons = case18_pose_report(ego)
                        print(f'CASE18_BUILD={CASE18_BUILD}; live_crossing_vehicle_required=True.',flush=True)
                    elif case_number in (15,17,31):
                        controller={15:CurveFollowingController,17:SlowVehicleLaneChangeController,31:JunctionMergeController}[case_number]()
                        active_goal,sim_limit=BATCH_PROFILES[case_number][1],90
                        pose_report,pose_reasons=batch_pose_report(ego,case_number)
                        if case_number==17:batch_signals=BatchSignalWriter.from_sdk(17)
                        if case_number==31:batch_lights=Case31LightReader.from_sdk()
                        print(f'CASE{case_number}_BUILD={BATCH_BUILD}; separate_batch_controller=True.',flush=True)
                    elif case_number in EXPANDED_PROFILES:
                        controller=expanded_controller(case_number)
                        active_goal,sim_limit=controller.goal,90
                        pose_report,pose_reasons=expanded_pose_report(ego,case_number)
                        if case_number in (21,26,30,33):expanded_signals=BatchSignalWriter.from_sdk(case_number)
                        if case_number in (20,37):expanded_lights=ExpandedLightReader.from_sdk(case_number)
                        if case_number==20:expanded_empty_truth=Case10GroundTruthPolicy()
                        print(f'CASE{case_number}_BUILD={EXPANDED_BUILD}; separate_expanded_controller=True.',flush=True)
                        if case_number in (20,32):
                            print('END_CONDITION=STOP_LINE_AND_10S_STANDSTILL; do not drive to the distant route endpoint.',flush=True)
                        if case_number==21:
                            print('CASE21_MAP_CHECK: export contains no map 61ef37be; two-lane geometry inferred from actor/route coordinates. Verify the platform map before scoring.',flush=True)
                    elif case_number==40:
                        controller=SuddenConflictController()
                        case40_signals=BatchSignalWriter.from_sdk(40)
                        active_goal,sim_limit=CASE40_GOAL,91
                        pose_report,pose_reasons=case40_pose_report(ego)
                        print(f'CASE40_BUILD={CASE40_BUILD}; live_sudden_hazard_and_merge=True; no_startup_actor_wait.',flush=True)
                    else:
                        pose_report, pose_reasons = startup_pose_report(ego)
                    print(pose_report, flush=True)
                    if pose_reasons:
                        raise RuntimeError('Wrong/late starting pose; '
                                           f'failed checks={",".join(pose_reasons)}; '
                                           'API takeover NOT requested; see START_POSE; '
                                           'restart a fresh case')
                    task = bytes(info.taskId).decode('ascii', errors='strict')
                    evaluation_path = evaluation_record_path(directory, task)
                    print(f'Case={name}; task={task}; evaluation={evaluation_path}', flush=True)
                    case_validated = True
                    print(f'ROUTED_CASE={case_number}; algorithm={type(controller).__name__}; '
                          f'goal={active_goal}.', flush=True)
                    if case_number == 13:
                        # Unlike moving cut-in traffic, these static obstacles
                        # are very close. Apply a safety brake immediately, even
                        # while GT/wheel data warm up. This is case 13 ONLY.
                        SoSetDriverName('Ego', 'AutoDrive')
                        if not SoSetDriveMode('Ego', ESimOne_Drive_Mode(0)):
                            raise RuntimeError('Case 13 API takeover rejected')
                        takeover, first_stamp = True, ego.stamp
                        drive(brake=1.0, gear=1, gps=gps)
                        print('Case 13 API takeover accepted; INITIAL_BRAKE sent before sensor warm-up.', flush=True)
                        case13_signals = Case13SignalWriter.from_sdk()
                        case13_signals.apply(ego, controller.command(ego, brake=1.0))
                        controller.register_geometry_reader()
                final_distance = math.hypot(active_goal[0]-ego.x, active_goal[1]-ego.y)
                sim_s = 0.0 if first_stamp is None else (ego.stamp-first_stamp)/1000.0
                if sim_s > sim_limit:
                    raise RuntimeError(f'No platform end after {sim_limit} simulation seconds')
                raw = SimOne_Data_Obstacle()
                truth_ok = bool(SoGetGroundTruth('Ego', raw))
                gt_stamp = int(raw.timestamp)
                # Some SDKs leave the obstacle timestamp zero; log it explicitly.
                if gt_stamp > 0 and abs(gt_stamp-ego.stamp) > 500:
                    truth_ok = False
                objects = []
                peers = []
                truth_source = 'LIVE_GT' if truth_ok else 'WAIT_GT'
                if truth_ok:
                    for item in raw.obstacle[:max(0, min(int(raw.obstacleSize), len(raw.obstacle)))]:
                        kind = int(item.type.value)
                        accepted_types = VEHICLE_TYPES if case_number == 24 else CASE13_SOLID_TYPES
                        if kind not in accepted_types:
                            continue
                        values = (item.posX, item.posY, item.velX, item.velY,
                                  item.oriZ, item.length, item.width)
                        if not all(math.isfinite(v) for v in values) or item.length <= 0 or item.width <= 0:
                            raise RuntimeError('Invalid GroundTruth vehicle geometry; braking')
                        objects.append(Obstacle(int(item.id), kind, *map(float, values)))
                    if self_id is None and (not initialized or case_number in (10,15,17,18,31,35,40) or case_number in EXPANDED_PROFILES):
                        # Ego-only case may publish its first GT AFTER startup.
                        # Case 10 uses a tight calibrated body-centre match;
                        # case 13/24 retain their original startup-only logic.
                        self_id = (controller.find_self(ego, objects) if case_number in (10,15,17,18,31,35,40) or case_number in EXPANDED_PROFILES
                                   else find_self_id(ego, objects))
                    peers = [obj for obj in objects if obj.ident != self_id]
                    if case_number == 24 and not initialized and not peers:
                        truth_ok = False  # This case must have two surrounding vehicles.
                if case_number == 10:
                    # Read the lamp even when the empty scene has no obstacle
                    # publication. Previously the GT gate prevented all lamp
                    # reads, API takeover and evaluation initialization.
                    light = case10_lights.read()
                    truth_ok, truth_source = case10_truth.check(
                        truth_ok, int(raw.obstacleSize), gt_stamp, peers, light)
                    if not truth_ok and not takeover:
                        # Never leave a 60 km/h OSC-driven Ego uncontrolled
                        # while required sensor data are unavailable.
                        SoSetDriverName('Ego', 'AutoDrive')
                        if not SoSetDriveMode('Ego', ESimOne_Drive_Mode(0)):
                            raise RuntimeError('Case 10 safety takeover rejected')
                        takeover, first_stamp = True, ego.stamp
                        drive(brake=1.0, gear=1, gps=gps)
                        print('Case 10 missing-data safety takeover; brake applied.', flush=True)
                if case_number==20:
                    expanded_light=expanded_lights.read()
                    truth_ok,truth_source=expanded_empty_truth.check(truth_ok,int(raw.obstacleSize),gt_stamp,peers,expanded_light)
                if not truth_ok:
                    if (case_number in (15,17,18,31,35,40) or case_number in EXPANDED_PROFILES) and not takeover:
                        SoSetDriverName('Ego', 'AutoDrive')
                        if not SoSetDriveMode('Ego', ESimOne_Drive_Mode(0)):
                            raise RuntimeError(f'Case {case_number} missing-GT safety takeover rejected')
                        takeover, first_stamp = True, ego.stamp
                        drive(brake=1.0, gear=1, gps=gps)
                        print(f'Case {case_number} missing-GT safety takeover; brake applied before warm-up.', flush=True)
                    if takeover:
                        drive(gps=gps)
                    if now-last_print >= 1:
                        action = 'neutral brake' if takeover else 'API takeover withheld'
                        print(f'WAIT_GROUNDTRUTH: count={raw.obstacleSize}, stamp={gt_stamp}; {action}.', flush=True)
                        last_print = now
                    if now-last_truth_wall > 5:
                        raise RuntimeError('GroundTruth missing/stale/empty for 5 seconds; no timer fallback')
                    time.sleep(LOOP_WALL_S)
                    continue
                last_truth_wall = now
                if case_number in (13,15,17,18,31,35,40) or case_number in EXPANDED_PROFILES:
                    controller.observe_geometry(gps)
                # Case 24 still calculates risk BEFORE takeover, without a
                # mandatory startup stop. Case 13 has already applied its own
                # static-obstacle startup brake. Stale data still brakes above.
                if case_number == 10:
                    controller.observe_geometry(gps)
                    cmd = controller.update(ego, objects, dt, self_id, light=light)
                elif case_number==31:
                    cmd=controller.update(ego,objects,dt,self_id,light=batch_lights.read())
                elif case_number in (20,37):
                    cmd=controller.update(ego,objects,dt,self_id,light=expanded_light if case_number==20 else expanded_lights.read())
                else:
                    cmd = controller.update(ego, objects, dt, self_id)
                if not takeover:
                    # SDK SetDriverName is void; its wrapper's Boolean is ignored.
                    SoSetDriverName('Ego', 'AutoDrive')
                    if not SoSetDriveMode('Ego', ESimOne_Drive_Mode(0)):
                        raise RuntimeError('API takeover rejected')
                    print('SetDriveMode=True; AutoDrive API takeover accepted.', flush=True)
                    takeover = True
                    first_stamp = ego.stamp
                gear = getattr(cmd, 'gear', 1)
                if case_number == 13:
                    # Confirm the lamp command before any lane-change throttle.
                    # No signal SDK imports/calls are made for case 24.
                    case13_signals.apply(ego, cmd)
                elif case_number == 35:
                    case35_signals.apply(ego, cmd)
                elif case_number==17:
                    batch_signals.apply(ego,cmd)
                elif case_number in (21,26,30,33):
                    expanded_signals.apply(ego,cmd)
                elif case_number==40:
                    case40_signals.apply(ego,cmd)
                drive(cmd.brake, cmd.steering, cmd.throttle, gear, gps)
                lead = cmd.lead
                writer.writerow((round(sim_s, 3), ego.stamp, ego.frame, cmd.mode, ego.x, ego.y,
                                 math.degrees(ego.yaw), ego.speed, cmd.target_speed,
                                 cmd.steering, cmd.throttle, cmd.brake, gear,
                                 lead.ident if lead else '', lead.gap if lead else '',
                                 lead.speed if lead else '', lead.predicted if lead else '',
                                 cmd.ttc, cmd.lateral, cmd.distance, len(objects), gt_stamp,
                                 lead.entry_s if lead else '',
                                 lead.projected_gap(ego.speed) if lead else '',
                                 CASE10_LIGHT_ID if case_number in (10,20,31,37) else '',
                                 getattr(cmd, 'light_status', ''), getattr(cmd, 'light_countdown', ''),
                                 getattr(cmd, 'light_valid', ''), getattr(cmd, 'light_layout', ''),
                                 getattr(cmd, 'stopline_gap', ''),
                                 truth_source if case_number in (10,15,17,18,31,35,40) or case_number in EXPANDED_PROFILES else '',
                                 getattr(cmd, 'vru_blockers', ''), getattr(cmd, 'route_s', ''),
                                 getattr(cmd, 'conflict_id', ''), getattr(cmd, 'cross_enter', ''),
                                 getattr(cmd, 'cross_leave', ''), getattr(cmd, 'ego_enter', ''),
                                 getattr(cmd, 'yield_ids', '')))
                if not initialized:
                    initialized = True
                    csv_file.flush()  # Retain the first real command before SDK service calls.
                    print(f'Startup ready: speed={ego.speed:.3f}m/s; GT objects={len(objects)}; '
                          f'self_id={self_id}; GT timestamp={gt_stamp}; '
                          f'direct control gear={gear}: mode={cmd.mode}, '
                          f'throttle={cmd.throttle:.3f}, brake={cmd.brake:.3f}.', flush=True)
                    evaluation = bool(SoInitEvaluationServiceWithLocalData('Ego', True))
                    print(f'Evaluation init={evaluation}; first risk-based command accepted.', flush=True)
                    if not evaluation:
                        raise RuntimeError('Evaluation initialization failed; no scoring run continued')
                    if case_number in (10,15,17,18,31,35,40) or case_number in EXPANDED_PROFILES:
                        controller.register_geometry_reader()
                decision = (cmd.mode, lead.ident if lead else None)
                if now-last_print >= 1 or decision != last_decision:
                    lead_info = (f'id={lead.ident} gap={lead.gap:.2f}m v={lead.speed:.2f} '
                                 f'predicted={lead.predicted} entry={lead.entry_s:.2f}s '
                                 f'projected_gap={lead.projected_gap(ego.speed):.2f}m') if lead else 'none'
                    if case_number == 13:
                        rear_s, rear_l = case13_coordinates(*controller.rear_pose(ego)[:2])
                        lead_info += (f'; rear_s={rear_s:.3f}m rear_l={rear_l:.3f}m '
                                      f'reverse_target={controller.reverse_target} '
                                      f'signal_flags={cmd.signal_lights}')
                    elif case_number == 10:
                        lead_info += (f'; light_id={CASE10_LIGHT_ID}; light={cmd.light_status}; '
                                      f'valid={cmd.light_valid}; countdown={cmd.light_countdown}; '
                                      f'layout={cmd.light_layout}; bumper_stopline_gap={cmd.stopline_gap:.3f}m; '
                                      f'gt_source={truth_source}')
                    elif case_number == 35:
                        lead_info += (f'; bumper_stopline_gap={cmd.stopline_gap:.3f}m; '
                                      f'vru_blockers={cmd.vru_blockers or "none"}; route_s={cmd.route_s:.3f}m; '
                                      f'stopped_at_line={controller.stopped_at_line}; released={controller.released}; '
                                      f'signal_flags={cmd.signal_lights}')
                    elif case_number == 18:
                        lead_info += (f'; conflict_id={cmd.conflict_id}; cross_window={cmd.cross_enter:.2f}..{cmd.cross_leave:.2f}s; '
                                      f'ego_cross_enter={cmd.ego_enter:.2f}s; yield_ids={cmd.yield_ids or "none"}')
                    elif case_number in (15,17,31):
                        lead_info+=(f'; route_s={cmd.route_s:.3f}; phase={getattr(controller,"phase","none")}; '
                                    f'signal_flags={cmd.signal_lights}; light={cmd.light_status}; '
                                    f'light_valid={cmd.light_valid}; line_gap={cmd.stopline_gap:.3f}')
                    elif case_number in EXPANDED_PROFILES:
                        lead_info+=(f'; route_s={getattr(cmd,"route_s",0):.3f}; phase={getattr(controller,"phase","none")}; '
                                    f'signal_flags={getattr(cmd,"signal_lights",0)}; line_gap={getattr(cmd,"stopline_gap",float("inf")):.3f}; '
                                    f'still_s={getattr(controller,"stopped_s",0):.3f}; blockers={getattr(cmd,"vru_blockers",getattr(cmd,"yield_ids",""))}')
                    elif case_number==40:
                        lead_info+=(f'; conflict_id={cmd.conflict_id}; entry={cmd.cross_enter:.2f}; '
                                    f'leave={cmd.cross_leave:.2f}; ego_entry={cmd.ego_enter:.2f}; yield_ids={cmd.yield_ids or "none"}')
                    print(f'{sim_s:.2f}s {cmd.mode}: v={ego.speed:.2f}/{cmd.target_speed:.2f}m/s '
                          f'lat={cmd.lateral:+.3f}m steer={cmd.steering:+.3f} '
                          f'throttle={cmd.throttle:.3f} brake={cmd.brake:.3f}; gear={gear}; lead={lead_info}', flush=True)
                    csv_file.flush()
                    last_print = now
                    last_decision = decision
                time.sleep(LOOP_WALL_S)
    except (Exception, KeyboardInterrupt) as exc:
        failure = exc
        print(f'AUTODRIVE STOP: {exc}', flush=True)
        traceback.print_exc()
    finally:
        if connected:
            if failure is not None and takeover:
                try:
                    brake_deadline = time.monotonic()+2
                    while time.monotonic() < brake_deadline and int(SoGetCaseRunStatus()) == 2:
                        gps = SimOne_Data_Gps()
                        valid = bool(SoGetGps('Ego', gps))
                        drive(gps=gps if valid else last_usable)
                        if valid and math.hypot(gps.velX, gps.velY) < 0.05:
                            break
                        time.sleep(0.02)
                except Exception as exc:
                    print(f'Brake cleanup: {exc}', flush=True)
            if evaluation:
                try:
                    evaluation_result = finalize_evaluation(SoSaveEvaluationRecord,
                                                              SoGetCaseRunStatus,
                                                              evaluation_path, stopped=stopped)
                except Exception as exc:
                    print(f'Evaluation cleanup error: {exc}', flush=True)
            try:
                SoTerminateSimOneAPI()
            except Exception as exc:
                print(f'Terminate: {exc}', flush=True)
        dll_scope.close()
        os.chdir(old_directory)
    if failure is not None:
        return 1
    if case_number in (20,32) and not controller.stop_confirmed():
        print('Platform ended, but the expected close stop-line position/10s standstill is unconfirmed. Not a score.',flush=True)
        return 2
    if final_distance > 5.5:
        print(f'Platform stopped before goal: distance={final_distance:.2f}m; inspect scoring report.', flush=True)
        return 2
    if not (evaluation_result['saved'] and evaluation_result['end_reason'] == 1):
        print('Driving ended, but evaluation Goal is UNCONFIRMED. Exit code 3 is not a score.', flush=True)
        return 3
    print('Goal/evaluation confirmed. Actual score is determined only by the platform report.', flush=True)
    return 0


def self_test_cutin():
    """Pure regression checks, including a toy feedback model, NOT SimOne."""
    ego = EgoState(*START, ROAD_HEADING, 7.0974954354)

    def vehicle(ident, along, lateral, speed, lateral_speed=0.0):
        return Obstacle(ident, 6,
                        START[0]+along*ROAD_C-lateral*ROAD_S,
                        START[1]+along*ROAD_S+lateral*ROAD_C,
                        speed*ROAD_C-lateral_speed*ROAD_S,
                        speed*ROAD_S+lateral_speed*ROAD_C,
                        ROAD_HEADING+math.atan2(lateral_speed, speed), 4.8, 2.23)

    # Measured 13:46 speeds/gap: first AEB was STILL a predicted cut-in.
    # Old CSV did not contain entry time, so test several SYNTHETIC entry times;
    # this is a logic regression, not a reconstruction of the actual trajectory.
    for entry_s in (0.5, 1.0, 2.5, 5.0):
        recorded = Lead(10815033, 2.5890547649, 6.9302297485, True, entry_s)
        target, mode, _, emergency = lead_speed_plan(ego.speed, recorded)
        assert not emergency and mode == 'CUTIN_PREDICT'
        assert 0 < target < recorded.speed, 'Small cut-in gap was not being restored'
    # Measured next phase: the lead was faster but the old distance latch kept
    # full braking. Both predicted and established opening gaps must release.
    for predicted, entry_s in ((True, 0.1), (True, 1.8), (True, 5.0), (False, 0.0)):
        lead = Lead(10815033, 2.6882527542, 6.6018724918, predicted, entry_s)
        target, _, _, emergency = lead_speed_plan(5.2578819271, lead)
        assert not emergency and target > 0
    stopped = EgoState(*START, ROAD_HEADING, 0.00023464)
    ahead = vehicle(2, 7.5507565206+GPS_TO_FRONT_M+2.4, 0, 5.4978127925)
    recovered = CutInController().update(stopped, [ahead], 0.02)
    assert recovered.mode == 'FOLLOW_GAP' and recovered.throttle > 0
    assert recovered.brake == 0, 'Opening lead still latches Ego at zero'

    early = select_lead(ego, [vehicle(2, 22, 3.5, 6.0, -0.2)])
    assert early and early.predicted and 2.5 < early.entry_s <= CUTIN_LOOKAHEAD_S
    assert select_lead(ego, [vehicle(2, 22, 3.5, 6.0, 0.2)]) is None
    assert select_lead(ego, [vehicle(2, 22, 3.5, 6.0, 0.0)]) is None
    imminent = Lead(2, 1.0, 5.5, True, 0.1)
    assert lead_speed_plan(ego.speed, imminent)[3], 'Imminent cut-in emergency disabled'
    assert lead_speed_plan(ego.speed, Lead(2, 8, 0, False))[3]
    assert lead_speed_plan(ego.speed, Lead(2, 8, 5.5, False))[3]
    assert lead_speed_plan(ego.speed, Lead(2, 1.0, 8.0, False))[3]
    # Low-speed/stationary hazards may legitimately require stopping: never
    # force a 10 km/h scoring floor or manufacture clearance to avoid a penalty.
    assert lead_speed_plan(ego.speed, Lead(2, 8, 0, True, 3.0))[0] == 0

    controller = CutInController()
    opening = vehicle(2, 8+GPS_TO_FRONT_M+2.4, 0, 5.5)
    assert controller.update(ego, [opening], 0.02).brake == 1
    slower = EgoState(*START, ROAD_HEADING, 5.2)
    released = controller.update(slower, [opening], 0.02)
    assert released.mode != 'AEB' and released.brake < 1, 'Emergency did not release'
    cutin = vehicle(2, 2.589055+GPS_TO_FRONT_M+2.4, 3.5, 6.93, -0.65)
    routine = CutInController().update(ego, [cutin], 0.02)
    assert routine.mode == 'CUTIN_PREDICT' and routine.throttle == 0
    assert 0 < routine.brake <= BRAKE_RISE_PER_S*0.02+1e-9
    # A nearby predicted vehicle must not hide a real stopped vehicle ahead.
    blocked = vehicle(3, 15+GPS_TO_FRONT_M+2.4, 0, 0)
    for objects in ([cutin, blocked], [blocked, cutin]):
        command = CutInController().update(ego, objects, 0.02)
        assert command.mode == 'AEB' and command.lead.ident == 3 and command.brake == 1

    # Synthetic physics only. It tests feedback, multiple frame intervals and
    # ordinary cut-in brake recovery; it cannot prove platform DHW or a score.
    for dt in (0.02, 0.05, 0.1):
        controller = CutInController()
        ego_s, speed = 30.0, ego.speed
        lead_s = ego_s+2.589055+GPS_TO_FRONT_M+2.4
        minimum_speed, minimum_gap = speed, float('inf')
        seen_prediction = False
        for step in range(int(12/dt)):
            t = step*dt
            lead_speed = max(5.5, 6.93-1.25*t)
            lateral = max(0.0, 3.5-0.65*t)
            state = EgoState(START[0]+ego_s*ROAD_C, START[1]+ego_s*ROAD_S,
                             ROAD_HEADING, speed)
            obstacle = vehicle(2, lead_s, lateral, lead_speed, -0.65 if lateral > 0 else 0)
            command = controller.update(state, [obstacle], dt)
            seen_prediction |= command.mode == 'CUTIN_PREDICT'
            new_speed = max(0.0, speed+(9*command.throttle-9.5*command.brake-0.15*speed)*dt)
            ego_s += (speed+new_speed)/2*dt
            lead_s += lead_speed*dt
            speed = new_speed
            minimum_speed = min(minimum_speed, speed)
            minimum_gap = min(minimum_gap, lead_s-ego_s-GPS_TO_FRONT_M-2.4)
        assert seen_prediction and minimum_speed > 3.0, f'Toy cut-in still stopped: {dt=}'
        assert minimum_gap > COLLISION_BUFFER_M
        assert lead_s-ego_s-GPS_TO_FRONT_M-2.4 > 12, 'Toy cut-in gap did not recover'


def self_test_case13_recovery_and_signals():
    """Measured failure-pose regression plus mocked signal/staging checks."""
    # Last GPS record from the real failed 16:30 case. Geometry is the wheel
    # calibration printed by that run; this is not a synthetic success score.
    measured = EgoState(-274.5842590332031, -17.506101608276367,
                        0.0025870800018310547, 0.020533588,
                        1791880242201, 539)
    controller = ObstacleStartController()
    controller.wheelbase, controller.gps_to_rear, controller.measured = 2.9186, 0.0007, True
    pose = controller.rear_pose(measured)
    assert controller.plan_forward(pose, quiet=True) is None, 'Measured failed cusp was not reproduced'
    controller.phase = 'REVERSE'
    controller.reverse_origin_s = 3.260
    controller.reverse_lateral = case13_coordinates(*pose[:2])[1]
    controller.reverse_target = 3.260-8.0
    recovered = controller.update(measured, [], 0.02)
    assert recovered.mode == 'OBSTACLE_STAGING_REPLAN' and recovered.gear == 2
    assert recovered.throttle == 0 and recovered.brake > 0
    assert controller.reverse_target < case13_coordinates(*pose[:2])[0]-1.5
    target_pose = (*case13_world(controller.reverse_target, controller.reverse_lateral), CASE13_HEADING)
    points = controller.plan_forward(target_pose, quiet=True)
    assert points and all(controller.safe(p[:3], CASE13_FIXED) for p in points)
    try:
        controller.choose_staging(pose, CASE13_FIXED, max_reverse=1.9)
    except RuntimeError:
        pass
    else:
        raise AssertionError('Reverse distance bound ignored')

    class LightData:
        pass

    sent = []
    def sender(vehicle, data):
        sent.append((vehicle, data.frame, data.timestamp, data.signalLights))
        return True
    writer = Case13SignalWriter(sender, LightData)
    controller.begin_forward(points)
    rear_x, rear_y, yaw = target_pose
    first_throttle_s = None
    first_left_s = None
    for step in range(100):
        ego = EgoState(rear_x+controller.gps_to_rear*math.cos(yaw),
                       rear_y+controller.gps_to_rear*math.sin(yaw), yaw, 0,
                       10000+step*20, step)
        command = controller.update(ego, [], 0.02)
        writer.apply(ego, command)  # Same ordering as run_live: lamp before drive.
        if command.signal_lights & CASE13_LEFT_SIGNAL and first_left_s is None:
            first_left_s = step*0.02
        if command.throttle > 0:
            assert command.signal_lights & CASE13_LEFT_SIGNAL
            if first_throttle_s is None:
                first_throttle_s = step*0.02
    assert first_left_s is not None and first_throttle_s-first_left_s >= 1.0
    before = len(sent)
    writer.apply(ego, command)
    assert len(sent) == before, 'Signal refresh is unnecessarily repeated for one frame'
    assert all((flags & 1) == 0 for _, _, _, flags in sent), 'Wrong right indicator used'
    assert any(flags & CASE13_LEFT_SIGNAL for _, _, _, flags in sent)
    controller.phase = 'CRUISE'
    cleared = controller.command(ego, throttle=0.1, brake=0)
    assert cleared.signal_lights == 0
    writer.apply(ego, cleared)
    assert sent[-1][-1] == 0, 'Left indicator not cancelled after bypass'
    rejected_writer = Case13SignalWriter(lambda *_: False, LightData)
    try:
        rejected_writer.apply(ego, command)
    except RuntimeError:
        pass
    else:
        raise AssertionError('Rejected lamp command accepted')
    # Emergency braking must not shift an already-reversing car into D.
    controller.phase = 'REVERSE'
    off_x, off_y = case13_world(0, 6)
    failed = controller.update(EgoState(off_x, off_y, CASE13_HEADING, 0.8), [], 0.02)
    assert failed.gear == 2 and failed.brake == 1
    assert failed.signal_lights & CASE13_REVERSE_SIGNAL
    print('PASS case 13: measured failure pose, adaptive bounded replan, validated cusp/route, '
          'pre-change left indicator, lamp rejection, refresh/cancel, reverse fail-safe gear.', flush=True)


def self_test_case13():
    """Scene routing, swept geometry and lagged toy-plant tests, no SDK."""
    assert supported_case('24_跟车时邻车道车辆切入-26测试') == 24
    assert supported_case('13.前方障碍物起步-测试') == 13
    for wrong in ('14.稳定跟车-测试', '13.其他场景', '24.其他场景'):
        try:
            supported_case(wrong)
        except RuntimeError:
            pass
        else:
            raise AssertionError('Unsupported scene accepted')
    start = EgoState(*CASE13_START, CASE13_HEADING, 25/3.6)
    assert not case13_pose_report(start)[1]
    late = EgoState(start.x+2.1, start.y, start.yaw, start.speed)
    assert case13_pose_report(late)[1]
    controller = ObstacleStartController()
    assert controller.update(start, [], 0.02).brake == 1
    assert controller.update(start, [], 0.02).throttle == 0
    rear = controller.rear_pose(start)
    blocked_x, blocked_y = case13_world(-8, 0)
    blocked = Obstacle(99, 6, blocked_x, blocked_y, 0, 0, CASE13_HEADING, 4.8, 2.23)
    controller.still_s = 0.5
    try:
        controller.update(EgoState(*CASE13_START, CASE13_HEADING, 0), [blocked], 0.02)
    except RuntimeError as exc:
        assert 'reverse corridor is blocked' in str(exc)
    else:
        raise AssertionError('Blocked reverse corridor was accepted')
    controller = ObstacleStartController()
    ego_x, ego_y = case13_world(20, 3.5)
    moving = EgoState(ego_x, ego_y, CASE13_HEADING, 4.5)
    controller.points = controller.plan_forward(controller.rear_pose(moving))
    controller.phase = 'CRUISE'
    obstacle_x, obstacle_y = case13_world(24, 3.5)
    obstacle = Obstacle(99, 6, obstacle_x, obstacle_y, 0, 0, CASE13_HEADING, 4.8, 2.23)
    guarded = controller.update(moving, [obstacle], 0.02)
    assert guarded.brake == 1 and guarded.throttle == 0
    assert guarded.mode == 'OBSTACLE_RISK_BRAKE'
    # Neither this plant nor its derived times are official SimOne results.
    plants = ((3.1, 3.1, 0.0, 1.0, 1.0, 0.02),
              (2.9187, 2.9187, 25/3.6, 1.0, 1.0, 0.02),
              (3.1, 3.1, 25/3.6, 0.93, 0.85, 0.02),
              (3.1, 3.1, 25/3.6, 1.07, 1.1, 0.02),
              (3.1, 1.45, 25/3.6, 1.0, 1.0, 0.02),
              (3.1, 0.0, 25/3.6, 1.0, 1.0, 0.02),
              (3.1, 3.1, 25/3.6, 1.0, 1.0, 0.05),
              (3.1, 3.1, 25/3.6, 1.0, 1.0, 0.1),
              (2.9186, 0.0007, 7.061, 1.0, 0.70, 0.02),
              (2.9186, 0.0007, 7.061, 0.93, 0.60, 0.02),
              (2.9186, 0.0007, 7.061, 1.07, 0.60, 0.05))
    for wheelbase, offset, initial, steering_scale, brake_scale, dt in plants:
        controller = ObstacleStartController()
        controller.wheelbase, controller.gps_to_rear, controller.measured = wheelbase, offset, True
        pose = (*case13_world(-offset, 0), CASE13_HEADING)
        speed, delta, previous_gear = initial, 0.0, 1
        # Synthetic GT boxes slightly larger than the exported props, to cover
        # geometry uncertainty missed by the original no-GT toy checks.
        reported = [Obstacle(13000+i, o.kind, o.x, o.y, 0, 0, o.yaw,
                             o.length+0.1, o.width+0.1) for i, o in enumerate(CASE13_FIXED)]
        first_left_s = None
        for step in range(math.ceil(80/dt)):
            x, y, yaw = pose
            state = EgoState(x+offset*math.cos(yaw), y+offset*math.sin(yaw), yaw, speed)
            command = controller.update(state, reported, dt)
            assert controller.safe(pose, list(CASE13_FIXED)+reported, margin=0.0), 'Toy collision or outer-line crossing'
            assert not ('FAILSAFE' in command.mode), f'Toy controller failed: {plants=}, {command.mode=}'
            assert not (command.throttle > 0 and command.brake > 0)
            if command.signal_lights & CASE13_LEFT_SIGNAL and first_left_s is None:
                first_left_s = step*dt
            if command.throttle > 0 and command.gear == 1 and controller.phase == 'BYPASS':
                assert first_left_s is not None and step*dt-first_left_s >= 1.0
                assert command.signal_lights & CASE13_LEFT_SIGNAL
            if command.gear != previous_gear:
                assert speed < 0.04, 'Shifting gear while moving'
            previous_gear = command.gear
            if command.distance < 5:
                break
            new_speed = max(0, speed+(9*command.throttle-9.5*brake_scale*command.brake-0.15*speed)*dt)
            delta += (steering_scale*(-command.steering*FULL_STEER_RAD)-delta)*min(1, dt/0.12)
            direction = -1 if command.gear == 2 else 1
            pose = controller.integrate(pose, direction*(speed+new_speed)/2*dt,
                                        math.tan(delta)/wheelbase)
            speed = new_speed
        else:
            raise AssertionError(f'Toy case 13 did not reach goal: {wheelbase=}, {offset=}, {initial=}, {dt=}')
        print(f'PASS case 13 toy plant: offset={offset:.4f}m; wheelbase={wheelbase:.4f}m; '
              f'initial={initial:.3f}m/s; steering={steering_scale}; brake={brake_scale}; '
              f'dt={dt}; simulated_goal_time={step*dt:.2f}s (not platform score).', flush=True)


def self_test_case10():
    """Synthetic observations/toy physics only; no SDK, scene or score writes."""
    import struct
    green, red = Case10Light(2, 20, True, 'mock'), Case10Light(1, 15, True, 'mock')
    for layout, payload in (
        ('packed13', struct.pack('<Biii', 1, CASE10_LIGHT_ID, 15, 1)),
        ('python16', struct.pack('<iiii', 1, CASE10_LIGHT_ID, 20, 2)),
    ):
        decoded = Case10LightReader.decode(payload+b'\xff'*32)
        assert decoded.valid and decoded.layout == layout
        assert decoded.green == (layout == 'python16')
    assert not Case10LightReader.decode(struct.pack('<Biii', 1, 51, 20, 2)).valid
    assert not Case10LightReader.decode(b'\0'*64).valid
    assert not Case10LightReader.decode(struct.pack('<Biii', 1, 52, 20, 99)).valid
    assert Case10LightReader.decode(struct.pack('<Biii', 1, 52, 20000, 2)).green
    def packed_sender(vehicle, ident, data):
        assert vehicle == 'Ego' and ident == 52
        ctypes.memmove(ctypes.addressof(data), struct.pack('<Biii', 1, 52, 18, 2), 13)
        return True
    with contextlib.redirect_stdout(__import__('io').StringIO()):
        assert Case10LightReader(packed_sender).read().green
        assert not Case10LightReader(lambda *_: False).read().valid
        policy = Case10GroundTruthPolicy()
        assert policy.check(False, 0, 0, [], green) == (True, 'EGO_ONLY_NO_GT')
        assert policy.check(False, 0, 0, [], red)[0]
        assert not policy.check(False, 0, 0, [], Case10Light())[0]
        assert not policy.check(False, 0, 0, [], Case10Light(7, 0, True))[0]
        assert not policy.check(False, 1, 0, [], green)[0]
        assert not policy.check(False, 0, 999, [], green)[0]
        assert not policy.check(False, 1, 999, [], green)[0]
        assert policy.check(True, 0, 1200, [], green)[0]
        assert policy.check(True, 1, 1220, ['actual_hazard'], green)[0]
        assert not policy.check(False, 0, 0, [], green)[0], 'Hazard outage hidden by empty-scene fallback'
    assert supported_case('10.机动车信号灯识别及响应-测试(2)') == 10
    assert supported_case('13.前方障碍物起步') == 13
    assert supported_case('24.车辆切入') == 24
    for name in ('11.其他', '10.其他', '未知'):
        try:
            supported_case(name)
        except RuntimeError:
            pass
        else:
            raise AssertionError('Unsupported case accepted')
    ego = EgoState(*CASE10_START, CASE10_HEADING, 16.6667)
    assert not case10_pose_report(ego)[1]
    assert case10_pose_report(EgoState(-40, 2, CASE10_HEADING, 8))[1]
    initial = TrafficLightController().update(ego, [], 0.02, light=green)
    assert initial.brake > 0 and initial.throttle == 0  # Export starts at 60, requests 30 km/h.
    line_s = case10_coordinates(*CASE10_LINE_POINT)[0]
    c = TrafficLightController()
    x, y = case10_world(line_s-CASE10_BODY_FRONT_M-0.29)
    near = EgoState(x, y, CASE10_HEADING, 0)
    assert c.update(near, [], .02, light=red).mode == 'LIGHT_RED_HOLD'
    go = c.update(near, [], .02, light=green)
    assert go.brake == 0 and go.throttle > 0
    assert c.update(near, [], .02, light=Case10Light()).brake > 0
    assert c.update(near, [], .02, light=Case10Light(3, 3, True)).brake > 0
    x, y = case10_world(line_s-CASE10_BODY_FRONT_M-0.1)
    c.update(EgoState(x, y, CASE10_HEADING, 5), [], .02, light=green)
    x, y = case10_world(line_s-CASE10_BODY_FRONT_M+0.1)
    crossed = EgoState(x, y, CASE10_HEADING, 5)
    assert c.update(crossed, [], .02, light=green).throttle > 0
    assert c.entered and c.update(crossed, [], .02, light=red).mode == 'LIGHT_CRUISE'
    assert TrafficLightController().update(crossed, [], .02, light=red).brake == 1
    x, y = case10_world(20)
    cruise = EgoState(x, y, CASE10_HEADING, 8.3333)
    def obj(ident, ahead, lateral):
        ox, oy = case10_world(20+ahead, lateral)
        return Obstacle(ident, 6, ox, oy, 0, 0, CASE10_HEADING, 4.57, 2.21)
    assert TrafficLightController().update(cruise, [obj(9, 15, 0)], .02, light=green).mode == 'LIGHT_AEB'
    assert TrafficLightController().update(cruise, [obj(9, 15, 3.5)], .02, light=green).brake == 0
    owner = obj(0, 1.7, 0)
    owner = Obstacle(owner.ident, owner.kind, owner.x, owner.y,
                     cruise.speed*CASE10_C, cruise.speed*CASE10_S,
                     owner.yaw, owner.length, owner.width)
    assert TrafficLightController().find_self(cruise, [owner]) == 0
    close_peer = Obstacle(9, 6, *case10_world(23), cruise.speed*CASE10_C,
                           cruise.speed*CASE10_S, CASE10_HEADING, 4.57, 2.21)
    assert TrafficLightController().find_self(cruise, [close_peer]) is None
    try:
        TrafficLightController().update(EgoState(x, y, CASE10_HEADING, float('nan')), [], .02)
    except ValueError:
        pass
    else:
        raise AssertionError('Non-finite case 10 state accepted')
    trials = 0
    # Both exported moving-start speeds, update periods, braking uncertainty,
    # response lag, and calibrated GPS offsets. Red is position-triggered as
    # in the export, not assumed to begin at simulation time zero.
    for v0 in (0.0, 30/3.6, 60/3.6):
        for dt in (.02, .05, .1):
            for brake_gain in (4.5, 7.0, 10.0):
                for delay in (0.0, .1):
                    for gps_offset in (0.0, 0.0007, 3.1):
                        c = TrafficLightController()
                        c.gps_to_rear = gps_offset
                        x, y = CASE10_START
                        yaw, v, t = CASE10_HEADING, v0, 0.0
                        trigger = stop_gap = go_time = None
                        min_red_gap, max_lateral = float('inf'), 0.0
                        history = []
                        for i in range(math.ceil(89/dt)):
                            if trigger is None and x >= -45.7:
                                trigger = t
                            is_red = trigger is not None and t < trigger+15.8
                            light = red if is_red else green
                            ego = EgoState(x, y, yaw, v, round(t*1000), i)
                            cmd = c.update(ego, [], dt, light=light)
                            max_lateral = max(max_lateral, abs(cmd.lateral))
                            if is_red:
                                min_red_gap = min(min_red_gap, cmd.stopline_gap)
                                if v < .05 and cmd.stopline_gap < 1:
                                    stop_gap = cmd.stopline_gap
                            if trigger is not None and t >= trigger+15.8 and v > .2 and go_time is None:
                                go_time = t
                            history.append(cmd)
                            held = history[max(0, len(history)-1-round(delay/dt))]
                            acceleration = 5*held.throttle-brake_gain*held.brake-.14-.035*v
                            new_v = max(0.0, v+acceleration*dt)
                            step, v = .5*(v+new_v)*dt, new_v
                            curvature = math.tan(-held.steering*FULL_STEER_RAD)/c.wheelbase
                            x += step*math.cos(yaw)
                            y += step*math.sin(yaw)
                            yaw = wrap(yaw+step*curvature)
                            t += dt
                            if math.dist((x, y), CASE10_GOAL) < 5:
                                break
                        label = f'{v0=}, {dt=}, {brake_gain=}, {delay=}, {gps_offset=}'
                        assert stop_gap is not None and 0.1 < min_red_gap < 0.5, label
                        assert go_time is not None and go_time-trigger-15.8 < 5, label
                        assert t < 89 and math.dist((x, y), CASE10_GOAL) < 5, label
                        assert max_lateral < 0.8, label
                        trials += 1
    # Lost/invalid lights never cause a blind crossing, even after long waits.
    c, t, v = TrafficLightController(), 0.0, 8.3333
    x, y = case10_world(line_s-40)
    for i in range(2000):
        ego = EgoState(x, y, CASE10_HEADING, v, i*20, i)
        cmd = c.update(ego, [], .02, light=Case10Light())
        nv = max(0.0, v+(5*cmd.throttle-7*cmd.brake-.14-.035*v)*.02)
        step, v = .5*(v+nv)*.02, nv
        x += step*CASE10_C
        y += step*CASE10_S
        assert c.front_gap(EgoState(x, y, CASE10_HEADING, v)) > 0
    assert v < .05 and cmd.mode == 'LIGHT_SIGNAL_HOLD' and not c.entered
    print(f'PASS case 10 offline: exact light 52, two ABI layouts, false/invalid reads, '
          f'stop/green release, post-entry clearance, AEB, {trials} toy plants, '
          'bumper gap, 5s launch and 89s goal checks, unknown-light hold, '
          'empty-GT acceptance and real-hazard outage rejection. Not a platform score.')


def self_test_case35():
    """Synthetic position-triggered actors and toy vehicle; never actual control."""
    import io
    assert supported_case('35.行人和非机动车通行-测试') == 35
    assert not case35_pose_report(EgoState(*CASE35_START, CASE35_HEADING, 25/3.6))[1]
    assert case35_pose_report(EgoState(50, -11.4, CASE35_HEADING, 7))[1]
    points = case35_route()
    assert all(a.s < b.s for a,b in zip(points, points[1:]))
    assert max(abs(p.curvature) for p in points) < .2
    assert min(math.dist((p.x,p.y), CASE35_GOAL) for p in points) < .05
    def actors(age=0., *, ped_speed=3/3.6, bike_speed=10/3.6, bike_size=(1.8,.65), moving=True):
        py = max(-18.56587294353564, 6.087367269744453-ped_speed*age)
        by = min(6.246882832794062, -23.36525771316596+bike_speed*age)
        return [Obstacle(101, 8, 61.2, by, 0, bike_speed if moving and by < 6.2468 else 0,
                         math.pi/2, *bike_size),
                Obstacle(102, 4, 59.4, py, 0, -ped_speed if moving and py > -18.5658 else 0,
                         -math.pi/2, .5, .5)]
    c = VRUYieldController()
    start = EgoState(*CASE35_START, CASE35_HEADING, 0)
    with contextlib.redirect_stdout(io.StringIO()):
        begin = c.update(start, actors(moving=False), .02)
        assert begin.throttle > 0 and begin.brake == 0, 'VRU wait stopped Ego before the actor trigger'
        assert set(c.roles) == {'ped','bike'}
        lost = c.update(start, actors()[:1], .02)
        assert lost.mode == 'VRU_WAIT_ACTORS' and lost.throttle == 0 and lost.brake == 1
        blockers, missing, _ = c.crossing_blockers(actors(40), None)
        assert not blockers and not missing
        stationary = actors(0, moving=False)
        assert c.crossing_blockers(stationary, None)[0], 'Static actor assumed cleared by a timer'
    # Exact stop-line normal and calibrated GPS reference.
    for offset in (0., .0007, 3.1):
        c = VRUYieldController(); c.gps_to_rear = offset
        front = CASE35_BODY_FRONT-offset
        line_s = case35_incoming(*CASE35_LINE)[0]
        x = CASE35_START[0]+(line_s-front-.29)*CASE35_C
        y = CASE35_START[1]+(line_s-front-.29)*CASE35_S
        ego = EgoState(x,y,CASE35_HEADING,0)
        with contextlib.redirect_stdout(io.StringIO()):
            cmd = c.update(ego, actors(0), .02)
            assert cmd.mode == 'VRU_YIELD_HOLD' and .27 < cmd.stopline_gap < .3
            for _ in range(20):cmd=c.update(ego, actors(0), .02)
            assert c.stopped_at_line and not c.released
            for _ in range(20):cmd=c.update(ego, actors(40), .02)
            assert c.released and cmd.mode == 'VRU_CLEAR_GO' and cmd.throttle > 0 and cmd.brake == 0
        class Signals: pass
        sent=[]
        writer=Case35SignalWriter(lambda vehicle,data:sent.append(data.signalLights) or True, Signals)
        with contextlib.redirect_stdout(io.StringIO()):writer.apply(ego, cmd)
        assert sent[-1] & CASE35_RIGHT_SIGNAL
        failing=Case35SignalWriter(lambda *_:False, Signals)
        try:failing.apply(ego, cmd)
        except RuntimeError:pass
        else:raise AssertionError('Rejected turn signal ignored')
    # Deliberately very slow or stalled actors must NOT trigger blind release.
    with contextlib.redirect_stdout(io.StringIO()):
        c=VRUYieldController();c.stopped_at_line=True
        for _ in range(300):
            cmd=c.update(ego, actors(0,moving=False), .02)
            assert not c.released and cmd.throttle == 0
    scenarios = [
        (0, .02, 1., 7., 0., .0007, 3/3.6, 10/3.6, (1.8,.65)),
        (25/3.6, .02, 1., 7., 0., .0007, 3/3.6, 10/3.6, (1.8,.65)),
        (25/3.6, .05, .93, 4.5, .1, .0007, .6, 2., (1.8,.65)),
        (25/3.6, .1, 1.07, 10., .1, .0007, 1.1, 4., (1.8,.65)),
        (0, .05, .93, 5., .1, 3.1, .6, 2., (4.8,2.23)),
        (25/3.6, .05, 1.07, 10., .1, 3.1, 1.1, 4., (4.8,2.23)),
        (0, .02, 1., 7., 0., 0., 3/3.6, 10/3.6, (4.8,2.23)),
        (25/3.6, .1, .93, 4.5, .1, 0., .6, 4., (4.8,2.23)),
    ]
    for v0, dt, steer_gain, brake_gain, lag, offset, ped_speed, bike_speed, bike_size in scenarios:
        c=VRUYieldController();c.gps_to_rear=offset;c.wheelbase=2.9186
        rear=(CASE35_START[0]-offset*CASE35_C, CASE35_START[1]-offset*CASE35_S, CASE35_HEADING)
        v,t=v0,0.;trigger=stop_gap=release_time=first_clear=None
        history=[]; max_lateral=0.;modes=set()
        with contextlib.redirect_stdout(io.StringIO()):
            for i in range(math.ceil(89/dt)):
                x,y,yaw=rear
                gx,gy=x+offset*math.cos(yaw),y+offset*math.sin(yaw)
                if trigger is None and gx>=18.6:trigger=t
                age=max(0., t-trigger) if trigger is not None else 0.
                objs=actors(age,ped_speed=ped_speed,bike_speed=bike_speed,bike_size=bike_size,moving=trigger is not None)
                ego=EgoState(gx,gy,yaw,v,round(t*1000),i)
                cmd=c.update(ego,objs,dt)
                modes.add(cmd.mode);max_lateral=max(max_lateral,abs(cmd.lateral))
                if not cmd.vru_blockers and first_clear is None:first_clear=t
                if cmd.mode=='VRU_YIELD_HOLD' and v<.05:stop_gap=cmd.stopline_gap
                if c.released and release_time is None:release_time=t
                body=c.rectangle(*rear,4.57,2.21,1.7)
                assert not any(c.overlap(body,c.rectangle(o.x,o.y,o.yaw,o.length,o.width)) for o in objs), \
                    f'Toy collision at {t=}, {dt=}, {offset=}'
                if not c.released:
                    assert c.front_gap(ego)>0, f'Red/yield stop-line crossing at {t=}'
                # Entire physical body must avoid incoming/outgoing SOLID edges,
                # not merely the rear GPS point staying close to a centreline.
                for bx,by in body:
                    incoming_s, incoming_l=case35_incoming(bx,by)
                    if bx < 57.25:
                        assert abs(incoming_l)<1.75-.04, f'Incoming solid-line touch: {t=}, {incoming_l=}'
                    if by < -24.2:
                        out_h=-1.5651466693913327
                        out_l=-(bx-75.40555650198608)*math.sin(out_h)+(by+24.033500851536544)*math.cos(out_h)
                        # Road 211/-1 is the broken divider beside our -2 lane;
                        # solid boundaries here are outer -7 and centre 0.
                        assert -7+.04 < out_l < -.04, f'Exit solid-line touch: {t=}, {out_l=}'
                if math.dist((gx,gy),CASE35_GOAL)<5:break
                history.append(cmd)
                held=history[max(0,len(history)-1-round(lag/dt))]
                nv=max(0.,v+(5*held.throttle-brake_gain*held.brake-.14-.035*v)*dt)
                step,v=.5*(v+nv)*dt,nv
                rear=c.integrate(rear,step,math.tan(-steer_gain*held.steering*FULL_STEER_RAD)/c.wheelbase)
                t+=dt
        label=f'{v0=}, {dt=}, {steer_gain=}, {brake_gain=}, {lag=}, {offset=}, {ped_speed=}, {bike_size=}'
        assert stop_gap is not None and .05 < stop_gap < .5, label
        assert first_clear is not None and release_time is not None and 0 <= release_time-first_clear < 5, label
        assert t < 89 and cmd.distance < 5 and max_lateral < .65, label
        assert 'VRU_YIELD_HOLD' in modes and 'VRU_RIGHT_TURN' in modes, label
        assert not any(mode.endswith('FAILSAFE') for mode in modes), label
    print('PASS case 35 offline: map-derived lane-centre turn, bumper stop, two live VRUs, '
          'no timer/disappearance release, right signal/rejection, 8 toy plants, actual-body '
          'collision/solid-edge guards, 5s launch and 89s goal checks. Not a platform score.')


def self_test_case18():
    """Synthetic TTC-triggered cross traffic. No live SDK or scene mutation."""
    import io
    assert supported_case('18.无信号灯路口车辆冲突通行-测试') == 18
    assert not case18_pose_report(EgoState(*CASE18_START,CASE18_HEADING,CASE18_CRUISE))[1]
    assert case18_pose_report(EgoState(-20,2,CASE18_HEADING,7))[1]
    def npc(y=27.907306408210108,speed=0.,ident=101):
        return Obstacle(ident,6,-3.3,y,0,-speed,-math.pi/2,4.8,2.23)
    start=EgoState(*CASE18_START,CASE18_HEADING,CASE18_CRUISE)
    with contextlib.redirect_stdout(io.StringIO()):
        c=CrossTrafficController()
        cmd=c.update(start,[npc()],.02)
        assert cmd.mode=='CROSS_CRUISE' and cmd.brake==0 and cmd.throttle>0
        assert not c.yielding, 'Startup stop prevented the TTC-triggered NPC from moving'
        assert c.forecast(start,npc()) is None
        assert c.update(start,[],.02).brake==1, 'Missing actor assumed clear'
        c=CrossTrafficController()
        close=EgoState(-24.2,case18_world(case18_coordinates(-24.2,2)[0])[1],CASE18_HEADING,CASE18_CRUISE)
        forecast=c.forecast(close,npc(27.,12.5))
        assert forecast and forecast.risk
        planned=c.update(close,[npc(27.,12.5)],.02)
        assert planned.mode=='CROSS_PREDICT_RESERVE' and planned.brake==0
        assert c.yielding
        # A stopped vehicle IN the crossing is not cleared by Ego's speed=0.
        stopped=EgoState(-11.2,case18_world(case18_coordinates(-11.2,2)[0])[1],CASE18_HEADING,0)
        for _ in range(100):
            hold=c.update(stopped,[npc(2.,0)],.02)
            assert hold.throttle==0 and hold.brake>0 and c.yielding
        for _ in range(20):go=c.update(stopped,[npc(-12.,5.5)],.02)
        assert not c.yielding and go.throttle>0 and go.brake==0
        c=CrossTrafficController()
        unexpected=Obstacle(202,6,*case18_world(9),0,0,CASE18_HEADING,4.8,2.23)
        assert c.update(start,[npc(),unexpected],.02).mode=='CROSS_AEB'
    scenarios = [
        (0.,.02,7.,0.,0.,5.5555556,False),
        (CASE18_CRUISE,.02,7.,0.,.0007,5.5555556,False),
        (CASE18_CRUISE,.02,7.,0.,.0007,8.,False),
        (CASE18_CRUISE,.02,7.,0.,.0007,12.5,False),
        (CASE18_CRUISE,.1,4.5,.1,0.,12.5,False),
        (0.,.05,4.5,.1,3.1,8.,False),
        (CASE18_CRUISE,.05,10.,.1,3.1,15.,False),
        (CASE18_CRUISE,.02,7.,0.,.0007,8.,True),
    ]
    for v0,dt,brake_gain,lag,offset,npc_speed,pause_crossing in scenarios:
        c=CrossTrafficController();c.gps_to_rear=offset;c.wheelbase=2.9186
        pose=(CASE18_START[0]-offset*CASE18_C,CASE18_START[1]-offset*CASE18_S,CASE18_HEADING)
        v,t=v0,0.;trigger=None;history=[];modes=set();max_lateral=0.;cleared=None;released=None
        with contextlib.redirect_stdout(io.StringIO()):
            for i in range(math.ceil(89/dt)):
                rx,ry,yaw=pose;gx,gy=rx+offset*math.cos(yaw),ry+offset*math.sin(yaw)
                # Export triggers the other car with Ego TTC<2.5, NOT a fixed
                # simulation time. The controller never sees this fixture clock.
                freespace=max(0.,math.hypot(-3-gx,2-gy)-(CASE18_BODY_FRONT-offset))
                if trigger is None and v>.1 and freespace/v<2.5:trigger=t
                age=t-trigger if trigger is not None else 0.
                if pause_crossing:
                    # Stop its centre on Ego's route for three seconds, then
                    # resume. A current-speed-only collision check would release
                    # Ego into this stationary conflict; yielding must persist.
                    arrival=(27.907306408210108-2.)/npc_speed
                    if arrival < age < arrival+3:
                        oy,ov=2.,0.
                    else:
                        motion_age=age-(3 if age>=arrival+3 else 0)
                        oy=max(-35.653221609090245,27.907306408210108-motion_age*npc_speed)
                        ov=npc_speed if trigger is not None and oy>-35.6532 else 0.
                else:
                    oy=max(-35.653221609090245,27.907306408210108-age*npc_speed)
                    ov=npc_speed if trigger is not None and oy>-35.6532 else 0.
                other=npc(oy,ov)
                ego=EgoState(gx,gy,yaw,v,round(t*1000),i)
                cmd=c.update(ego,[other],dt)
                modes.add(cmd.mode);max_lateral=max(max_lateral,abs(cmd.lateral))
                assert not c.overlap(c.rectangle(*pose,4.57,2.21,1.7),
                                     c.rectangle(other.x,other.y,other.yaw,other.length,other.width)), \
                    f'Toy collision at {t=}, {npc_speed=}, {dt=}, {offset=}'
                for bx,by in c.rectangle(*pose,4.57,2.21,1.7):
                    assert abs(case18_coordinates(bx,by)[1])<1.7, f'Outer/inner lane edge touch at {t=}'
                if c.physical_clear(ego,other,-1) and cleared is None:cleared=t
                if cleared is not None and not c.yielding and cmd.throttle>0 and released is None:released=t
                if math.dist((gx,gy),CASE18_GOAL)<5:break
                history.append(cmd);held=history[max(0,len(history)-1-round(lag/dt))]
                nv=max(0.,v+(5*held.throttle-brake_gain*held.brake-.14-.035*v)*dt)
                step,v=.5*(v+nv)*dt,nv
                pose=c.integrate(pose,step,math.tan(-held.steering*FULL_STEER_RAD)/c.wheelbase)
                t+=dt
        label=f'{v0=}, {dt=}, {brake_gain=}, {lag=}, {offset=}, {npc_speed=}, {pause_crossing=}'
        assert trigger is not None and t<89 and cmd.distance<5 and max_lateral<.8,label
        assert not any(mode.endswith('FAILSAFE') for mode in modes),label
        if npc_speed>=8:
            assert 'CROSS_YIELD_APPROACH' in modes,label
            assert cleared is not None and released is not None and released-cleared<2.,label
        if pause_crossing:assert 'CROSS_YIELD_HOLD' in modes,label
    print('PASS case 18 offline: moving TTC trigger without startup deadlock, live crossing windows, '
          'physical-clear yield latch, stationary crossing hold, unexpected-car AEB, actor-loss braking, '
          '8 toy plants, collision/lane/89s goal checks. Not a platform score.')


def batch_test_merge_route():
    """Synthetic other-car path for tests ONLY; driving uses live GT instead."""
    vertices=((3.570044,-14.017035,1.563581),(3.616228,-7.61602,1.628941),
              (3.643067,-3.896103,1.342797),(7.015375,.375286,.512697),
              (12.465297,1.762855,.062348),(18.100401,1.748625,-.026),
              (29.401339,1.720087,.003332),(57.600907,1.648876,-.002525))
    coords=[]
    for (x0,y0,h0),(x1,y1,h1) in zip(vertices,vertices[1:]):
        d=math.hypot(x1-x0,y1-y0);count=math.ceil(d/.1)
        for i in range(count):
            u=i/count;a=2*u**3-3*u*u+1;b=u**3-2*u*u+u;c=-2*u**3+3*u*u;e=u**3-u*u
            coords.append((a*x0+b*d*math.cos(h0)+c*x1+e*d*math.cos(h1),
                           a*y0+b*d*math.sin(h0)+c*y1+e*d*math.sin(h1),2.))
    coords.append((*vertices[-1][:2],2.))
    return BatchRoute(coords)


def self_test_batch():
    """Three new scene algorithms, synthetic actors/physics; no SDK or files."""
    import io,struct
    for number,title in ((15,'弯道内跟车'),(17,'避让低速行驶车辆变道'),(31,'路口车辆冲突通行')):
        assert supported_case(f'{number}.{title}-测试')==number
        start,goal,h,speed=BATCH_PROFILES[number]
        assert not batch_pose_report(EgoState(*start,h,speed),number)[1]
        assert batch_pose_report(EgoState(start[0]+20,start[1],h,speed),number)[1]
    route=case15_route()
    assert max(abs(p.curvature) for p in route.points)<.15
    assert all(a.s<b.s for a,b in zip(route.points,route.points[1:]))
    assert min(math.dist((p.x,p.y),BATCH_PROFILES[15][1]) for p in route.points)<.06
    class Signals:pass
    writer=BatchSignalWriter(lambda *_:False,Signals,17)
    try:writer.apply(EgoState(0,0,0,0),BatchCommand(0,.3,0,4,'test',0,1,signal_lights=2))
    except RuntimeError:pass
    else:raise AssertionError('Rejected lane-change signal ignored')
    def lamps(_,ident,data):
        assert ident==52
        payload=struct.pack('<Biii',1,52,300,2);ctypes.memmove(ctypes.addressof(data),payload,len(payload));return True
    with contextlib.redirect_stdout(io.StringIO()):assert Case31LightReader(lamps).read().green
    # Missing main vehicles never become an automatic permission to drive.
    for cls in (CurveFollowingController,SlowVehicleLaneChangeController,JunctionMergeController):
        c=cls();start,_,h,_=BATCH_PROFILES[c.number];ego=EgoState(*start,h,0)
        with contextlib.redirect_stdout(io.StringIO()):
            cmd=c.update(ego,[],.02,light=Case10Light(2,300,True)) if c.number==31 else c.update(ego,[],.02)
        assert cmd.brake==1 and cmd.throttle==0
    scenarios=(
        # case, start speed, dt, brake, steering, lag, GPS offset, actor speed, extra
        (15,0.,.02,7.,1.,0.,.0007,20/3.6,''),
        (15,25/3.6,.02,7.,1.,0.,.0007,20/3.6,''),
        (15,25/3.6,.1,4.5,.93,.1,0.,20/3.6,''),
        (15,0.,.05,10.,1.07,.1,3.1,3.,''),
        (15,25/3.6,.05,4.5,1.,.1,.0007,20/3.6,'lead_pause'),
        (17,0.,.02,7.,1.,0.,.0007,5/3.6,''),
        (17,25/3.6,.02,7.,1.,0.,.0007,5/3.6,''),
        (17,25/3.6,.1,4.5,.93,.1,0.,.5,''),
        (17,0.,.05,10.,1.07,.1,3.1,5/3.6,''),
        (17,25/3.6,.05,7.,1.,.1,.0007,5/3.6,'left_rear'),
        (31,0.,.02,7.,1.,0.,.0007,15/3.6,''),
        (31,40/3.6,.02,7.,1.,0.,.0007,15/3.6,''),
        (31,40/3.6,.1,4.5,.93,.1,0.,15/3.6,''),
        (31,40/3.6,.02,4.5,1.,.1,.0007,15/3.6,''),
        (31,0.,.05,10.,1.07,.1,3.1,15/3.6,''),
        (31,0.,.05,10.,1.07,.1,3.1,10/3.6,''),
        (31,40/3.6,.02,7.,1.,0.,.0007,15/3.6,'red'),
    )
    merge_route=batch_test_merge_route()
    for number,v0,dt,brake_gain,steer_gain,lag,offset,actor_speed,extra in scenarios:
        c={15:CurveFollowingController,17:SlowVehicleLaneChangeController,31:JunctionMergeController}[number]()
        c.gps_to_rear=offset;c.wheelbase=2.9186
        start,goal,h,_=BATCH_PROFILES[number]
        pose=(start[0]-offset*math.cos(h),start[1]-offset*math.sin(h),h)
        v,t=v0,0.;trigger=None;history=[];modes=set();max_lat=0.;min_gap=float('inf')
        lead_route=case15_route() if number==15 else None
        lead_s=lead_route.project(99.9504826977,-3.5120802444)[0] if number==15 else 0.
        min_warning_speed=float('inf');signal_start=change_time=None
        with contextlib.redirect_stdout(io.StringIO()):
            for i in range(math.ceil(89/dt)):
                rx,ry,yaw=pose;gx,gy=rx+offset*math.cos(yaw),ry+offset*math.sin(yaw)
                if trigger is None and (number==15 and gx>=72.75 or number==17 and gx>=-499.5 or number==31 and gx>=-31.55):trigger=t
                age=max(0.,t-trigger) if trigger is not None else 0.
                if number==15:
                    delay=1.5 if extra=='lead_pause' and age>7.5 else max(0.,age-6.) if extra=='lead_pause' and age>6 else 0.
                    point=lead_route.sample(lead_s+max(0.,age-delay)*actor_speed)
                    speed=0. if trigger is None or extra=='lead_pause' and 6.<age<7.5 else actor_speed
                    actor=Obstacle(101,6,point.x,point.y,speed*math.cos(point.yaw),speed*math.sin(point.yaw),point.yaw,4.8,2.23)
                elif number==17:
                    x,y=c.world(24.3015+age*actor_speed,.005)
                    speed=actor_speed if trigger is not None else 0.
                    actor=Obstacle(101,6,x,y,speed*math.cos(h),speed*math.sin(h),h,4.8,2.23)
                else:
                    point=merge_route.sample(max(0.,age-.824)*actor_speed)
                    speed=actor_speed if age>.824 else 0.
                    actor=Obstacle(101,6,point.x,point.y,speed*math.cos(point.yaw),speed*math.sin(point.yaw),point.yaw,4.8,2.23)
                objects=[actor]
                if extra=='left_rear':
                    x,y=c.world(-18+12*t,3.5)
                    objects.append(Obstacle(102,6,x,y,12*math.cos(h),12*math.sin(h),h,4.8,2.23))
                ego=EgoState(gx,gy,yaw,v,round(t*1000),i)
                light=Case10Light(1 if extra=='red' and t<17 else 2,300,True,'mock')
                cmd=c.update(ego,objects,dt,light=light) if number==31 else c.update(ego,objects,dt)
                modes.add(cmd.mode);max_lat=max(max_lat,abs(cmd.lateral))
                body=c.rectangle(*pose,4.57,2.21,1.7)
                assert not any(c.overlap(body,c.rectangle(o.x,o.y,o.yaw,o.length,o.width)) for o in objects),f'Batch collision: {number=}, {t=}, {extra=}'
                if number==15:
                    lead_progress=lead_route.project(actor.x,actor.y)[0]
                    ego_progress=c.route.project(rx,ry)[0]
                    physical_gap=lead_progress-ego_progress-(1.7+4.57/2)-actor.length/2
                    min_gap=min(min_gap,physical_gap)
                    assert physical_gap>10.,f'Case15 DHW reserve lost: {physical_gap=}, {t=}, {extra=}'
                    cx,cy=rx+1.7*math.cos(yaw),ry+1.7*math.sin(yaw)
                    assert abs(c.route.project(cx,cy,key='body_center')[1])<.5,f'Case15 body-centre warning at {t=}'
                    for j,(bx,by) in enumerate(body):
                        _,bl,p,_=c.route.project(bx,by,key=('corner',j))
                        assert abs(bl)<p.half_width-.025,f'Case15 solid-line touch at {t=}, {bl=}'
                    if 83.9<gx<164.2 and extra!='lead_pause':min_warning_speed=min(min_warning_speed,v)
                elif number==17:
                    for bx,by in body:assert -1.75<c.coordinates(bx,by)[1]<5.25,f'Case17 edge touch: {t=}'
                    if cmd.signal_lights&2 and signal_start is None:signal_start=t
                    if c.phase=='LANE_CHANGE' and change_time is None:change_time=t
                else:
                    for bx,by in body:assert abs(c.coordinates(bx,by)[1])<1.7,f'Case31 line touch: {t=}'
                    if not light.green and not c.entered:assert c.line_gap(ego)>0.,f'Case31 red-line crossing at {t=}'
                if math.dist((gx,gy),goal)<5:break
                history.append(cmd);held=history[max(0,len(history)-1-round(lag/dt))]
                nv=max(0.,v+(5*held.throttle-brake_gain*held.brake-.14-.035*v)*dt)
                if number==31 and extra!='red' and actor_speed==15/3.6 and -47.6<gx<19.4:
                    assert nv<=v+1e-6,f'Case31 positive-acceleration warning: {t=}, {v=}, {nv=}, {dt=}, {lag=}'
                step,v=.5*(v+nv)*dt,nv
                pose=c.integrate(pose,step,math.tan(-steer_gain*held.steering*FULL_STEER_RAD)/c.wheelbase);t+=dt
        label=f'{number=}, {v0=}, {dt=}, {brake_gain=}, {steer_gain=}, {lag=}, {offset=}, {extra=}'
        assert t<89 and cmd.distance<5 and trigger is not None,label
        assert not any(mode.endswith('FAILSAFE') for mode in modes),label
        if number==15:assert max_lat<.45 and min_warning_speed>2.7777797,label
        if number==17:
            assert c.phase=='PASS' and change_time is not None and change_time-signal_start>=.9,label
            assert 'CHANGE_LANE_CHANGE' in modes and not cmd.signal_lights&2,label
            if extra=='left_rear':assert change_time>2.,label
        if number==31:
            assert c.entered and 'MERGE_FOLLOW' in modes,label
            if extra=='red':assert 'MERGE_SIGNAL_HOLD' in modes,label
    print('PASS new batch offline: 15 map-centre U-turn/16.5m following reserve, 17 left signal/swept '
          'lane change/rear-car wait, 31 live green light/turning-car merge-to-ACC; '
          '17 toy plants, physical collision/line/curve-centre/DHW/min-speed/positive-acceleration/89s checks. Not a platform score.')


def self_test_expanded():
    """New export: synthetic observations/grade/actuator models, no SDK/files."""
    import io
    from ParkingAPA import rectangle,overlap,integrate
    for number,title in EXPANDED_TITLES.items():
        assert supported_case(f'{number}.{title}-测试')==number
        start,_,h,speed=EXPANDED_PROFILES[number]
        assert not expanded_pose_report(EgoState(*start,h,speed),number)[1]
        assert expanded_pose_report(EgoState(start[0]+20,start[1],h,speed),number)[1]
    hill=expanded_hill_route()
    assert abs(hill.project(*EXPANDED_PROFILES[22][0])[1])<.03
    assert abs(hill.project(*EXPANDED_PROFILES[22][1])[1])<.03
    assert max(abs(p.curvature) for p in hill.points)<.005
    # Missing actors and falsely treated partial pedestrian groups must brake.
    with contextlib.redirect_stdout(io.StringIO()):
        for number in (21,22,26,30,32,33,37):
            c=expanded_controller(number);start,_,h,_=EXPANDED_PROFILES[number]
            cmd=c.update(EgoState(*start,h,0),[],.02,light=Case10Light(2,300,True)) if number==37 else c.update(EgoState(*start,h,0),[],.02)
            assert cmd.brake==1 and cmd.throttle==0,number
    def bike_route():
        # Fixture only: exported southbound, right-curving bicycle trajectory.
        vertices=((60.2,7.7034452901,-math.pi/2),(60.25,-5.7006972611,-math.pi/2),
                  (60.25,-10.9528505674,-math.pi/2),(60.3,-13.7595565784,-math.pi/2),
                  (64.15,-17.705619104,5.373651069),(66.75,-21.7072597184,5.046853835))
        coords=[]
        for a,b in zip(vertices,vertices[1:]):
            d=math.dist(a[:2],b[:2]);n=math.ceil(d/.06)
            for i in range(n):
                u=i/n
                h00,h10,h01,h11=2*u**3-3*u*u+1,u**3-2*u*u+u,-2*u**3+3*u*u,u**3-u*u
                x=h00*a[0]+h10*d*math.cos(a[2])+h01*b[0]+h11*d*math.cos(b[2])
                y=h00*a[1]+h10*d*math.sin(a[2])+h01*b[1]+h11*d*math.sin(b[2])
                coords.append((x,y,1.75))
        coords.append((*vertices[-1][:2],1.75));return BatchRoute(coords)
    bike=bike_route();lead_path=expanded_hill_route();lead_s=lead_path.project(-70.3968641093,.5690871051)[0]
    scenarios=[(n,v0,dt,bg,sg,lag,offset,'') for n in EXPANDED_PROFILES
               for v0,dt,bg,sg,lag,offset in ((0.,.02,7.,1.,0.,.0007),
                   (25/3.6,.02,7.,1.,0.,.0007),(25/3.6,.1,4.5,.93,.1,0.))]
    scenarios.extend(((21,0.,.05,10.,1.07,.1,3.1,''),(26,0.,.05,10.,1.07,.1,3.1,'no_sign_gt'),
                      (30,0.,.05,7.,1.,.1,.0007,'large_bike'),(33,0.,.05,7.,1.,.1,.0007,'ped_pause'),
                      (37,0.,.05,7.,1.,.1,.0007,'unknown_light'),(22,0.,.05,10.,1.07,.1,3.1,'')))
    for number,v0,dt,brake_gain,steer_gain,lag,offset,extra in scenarios:
        c=expanded_controller(number);c.gps_to_rear=offset;c.wheelbase=2.9187
        start,goal,h,_=EXPANDED_PROFILES[number];cc,ss=math.cos(h),math.sin(h)
        pose=(start[0]-offset*cc,start[1]-offset*ss,h);v,t=v0,0.;trigger=None;history=[];modes=set()
        max_lat=0.;min_gap=float('inf');minimum_speed=float('inf');stop_gap=None;clear_at=launch_at=None
        signal_start=change_at=None
        with contextlib.redirect_stdout(io.StringIO()):
            for i in range(math.ceil(89/dt)):
                rx,ry,yaw=pose;gx,gy=rx+offset*math.cos(yaw),ry+offset*math.sin(yaw)
                if trigger is None and (number in (21,22) or number in (30,33) and gx>=18.4 or number==37 and gx>=-64.7):trigger=t
                age=max(0.,t-trigger) if trigger is not None else 0.
                def ob(ident,kind,x,y,sp=0.,heading=h,length=4.8,width=2.23):
                    return Obstacle(ident,kind,x,y,sp*math.cos(heading),sp*math.sin(heading),heading,length,width)
                if number==21:
                    objects=[ob(101,6,-210.8503089115,-17.3397053856),
                             ob(102,6,-240.6494453993+age*25/3.6*cc,-13.9174979895+age*25/3.6*ss,25/3.6 if trigger is not None else 0.)]
                elif number==22:
                    p=lead_path.sample(lead_s+age*20/3.6)
                    objects=[ob(101,6,p.x,p.y,20/3.6 if trigger is not None else 0.,p.yaw)]
                elif number==26:
                    objects=[ob(101,6,-191.8042405865,-17.2899767477)]
                    if extra!='no_sign_gt':objects.append(ob(102,7,-241.65,-17.6135350681,heading=0.,length=1.2,width=.5))
                elif number==30:
                    p=bike.sample(age*10/3.6);size=(4.8,2.23) if extra=='large_bike' else (1.8,.65)
                    sp=10/3.6 if trigger is not None and age*10/3.6<bike.points[-1].s else 0.
                    objects=[ob(101,8,p.x,p.y,sp,p.yaw,*size)]
                elif number==33:
                    delay=min(2.,max(0.,age-14.)) if extra=='ped_pause' else 0.
                    py=max(-18.5658729435,6.0873672697-age*3/3.6)
                    by=max(-18.0656678667,5.3939502347-age*3/3.6)
                    ny=min(4.8437246502,-17.2653397439+(age-delay)*3/3.6)
                    moving=trigger is not None
                    objects=[ob(101,4,59.4,py,3/3.6 if moving and py>-18.5658 else 0.,-math.pi/2,.5,.5),
                             ob(102,4,61.4,by,3/3.6 if moving and by>-18.0656 else 0.,-math.pi/2,.5,.5),
                             ob(103,4,60.3,ny,3/3.6 if moving and ny<4.8437 and not(extra=='ped_pause' and 14<age<16) else 0.,math.pi/2,.5,.5)]
                elif number==32:
                    objects=[ob(101+j,6,x,y) for j,(x,y) in enumerate(((18.3441082766,1.7480095024),(28.3513401924,1.7227387147),(38.6996220684,1.6966066898)))]
                elif number==37:
                    py=max(-32.5613817618,17.4705603595-age*3/3.6)
                    objects=[ob(101,4,-16.415349041,py,3/3.6 if trigger is not None and py>-32.5613 else 0.,-math.pi/2,.5,.5)]
                else:objects=[]
                ego=EgoState(gx,gy,yaw,v,round(t*1000),i)
                light=Case10Light(1 if number==20 else 2,300,not(extra=='unknown_light' and t<20),'mock')
                cmd=c.update(ego,objects,dt,light=light) if number in (20,37) else c.update(ego,objects,dt)
                modes.add(cmd.mode);max_lat=max(max_lat,abs(cmd.lateral))
                assert all(math.isfinite(x) for x in (cmd.steering,cmd.throttle,cmd.brake))
                assert 0<=cmd.throttle<=1 and 0<=cmd.brake<=1 and not(cmd.throttle>0 and cmd.brake>0)
                body=rectangle(*pose,4.57,2.21,1.7)
                assert not any(overlap(body,rectangle(o.x,o.y,o.yaw,o.length,o.width)) for o in objects),f'Expanded collision: {number=}, {t=}, {extra=}'
                if number==22:
                    os=lead_path.project(objects[0].x,objects[0].y,key='test_lead')[0]
                    es=c.route.project(rx,ry,key='test_ego')[0]
                    gap=os-es-(1.7+4.57/2)-objects[0].length/2;min_gap=min(min_gap,gap)
                    assert gap>10.,f'Hill DHW lost at {t=}, {gap=}'
                    cx,cy=rx+1.7*math.cos(yaw),ry+1.7*math.sin(yaw)
                    assert abs(c.route.project(cx,cy,key='test_center')[1])<.5
                    for j,(x,y) in enumerate(body):assert abs(c.route.project(x,y,key=('edge',j))[1])<1.72
                    if -81<gx<102.9:minimum_speed=min(minimum_speed,v)
                elif number in (21,26):
                    for x,y in body:assert -1.75<c.coordinates(x,y)[1]<5.25
                    if cmd.signal_lights&2 and signal_start is None:signal_start=t
                    if c.phase=='LANE_CHANGE' and change_at is None:change_at=t
                    if number==21 and c.phase!='WAIT_LEFT':assert c.coordinates(objects[1].x,objects[1].y)[0]-c.coordinates(rx,ry)[0]>10
                elif number in (30,33,37):
                    if c.stopped_at_line and stop_gap is None:stop_gap=c.front_gap(ego) if number in (30,33) else c.line_gap(ego)
                    blockers=cmd.vru_blockers if number in (30,33) else cmd.yield_ids
                    if not blockers and c.stopped_at_line and clear_at is None and light.green:clear_at=t
                    if c.released and cmd.throttle>0 and launch_at is None:launch_at=t
                    if not c.released:assert not(c.stopped_at_line and not .0<getattr(cmd,'stopline_gap')<.5)
                    if number==37 and not light.green:assert c.line_gap(ego)>0
                if number in (20,32):
                    assert .0<c.line_gap(ego),f'Stop line crossed: {number=}, {t=}'
                    if c.stopped_s>=10.1:break
                elif math.dist((gx,gy),goal)<5:break
                history.append(cmd);held=history[max(0,len(history)-1-round(lag/dt))]
                gravity=9.81*expanded_hill_grade(rx,ry) if number==22 else 0.
                nv=max(0.,v+(5*held.throttle-brake_gain*held.brake-.14-.035*v-gravity)*dt)
                step,v=.5*(v+nv)*dt,nv
                pose=integrate(pose,step,math.tan(-steer_gain*held.steering*FULL_STEER_RAD)/2.9187);t+=dt
        label=f'{number=}, {v0=}, {dt=}, {brake_gain=}, {lag=}, {offset=}, {extra=}'
        assert t<89 and not any(m.endswith('FAILSAFE') for m in modes),label
        if number in (20,32):assert c.stop_confirmed() and .03<c.line_gap(ego)<.5,label
        else:assert cmd.distance<5,label
        if number==22:assert max_lat<.45 and min_gap>10 and minimum_speed>2.7777777,label
        if number in (21,26):assert c.phase=='PASS' and change_at is not None and change_at-signal_start>=.9 and not(cmd.signal_lights&2),label
        if number in (30,33,37):assert stop_gap is not None and 0<stop_gap<.5 and launch_at is not None and clear_at is not None and launch_at-clear_at<5,label
    print('PASS expanded export offline: 20/32 close-line 10s stop endings, 21 left-car pass then signal/shift, '
          '22 real map grade/18m following reserve, 26 sign+broken-car avoidance, 30 southbound bike, '
          '33 all three pedestrians including northbound, 37 live green+pedestrian clearance; '
          '30 toy plants and missing-actor guards. No SDK connection or platform score.')



def case40_test_objects(car_age,bike_age,*,large_bike=False,continuing_car=False):
    """Offline-only fixture from the export; never called by live control."""
    import bisect
    car=(
        (0,-128.7510479665697,5.6194730767022945,6.280660060022073,),
        (12.411807074868396,-20.530916933976087,5.346189917528974,6.2812105928974376,),
        (13.427693759656789,-12.369711373352503,5.325580693368382,6.278715290123784,),
        (14.557569363847309,-4.096260082173242,5.304688139602754,0.008422526390435212,),
        (15.417800871778011,1.366190938281839,5.2908940713692685,6.238978921811886,),
        (16.49111883243232,7.170488938674725,4.8755676412495745,6.238043747916645,),
        (17.640025019402778,12.949747998505224,4.662659198766295,6.18355390525762,),
        (18.821748961948597,19.010909400012412,3.5471547111557924,6.057838147566388,),
        (20.144080302413137,26.499463718872903,2.1296902074601634,6.202243674598055,),
        (21.404570621732137,34.489360505649266,2.21410017032627,0.013855344183868638,),
        (22.578875059609977,42.197550288455986,2.0963617060218724,6.282919875279494,),
        (23.627201510069824,48.749682901999,2.181119558214243,6.279952421251804,),
        (25.078094541467294,57.67251360515195,1.7505948131885418,6.190335996817227,),
        (25.91641383913354,63.28132143709224,1.3269236612719622,6.2806600600220746,),
    )
    bike=(
        (0,-9.97405273438007,-8.554175371335054,1.5299206804967138,),
        (2.310809693327018,-1.170039062500018,1.5887140336701124,0.29663224158908835,),
        (4.722395651903491,13.769960937499977,1.615261419300353,6.215168887756996,),
        (5.3643705888802735,16.85,1.2037323975755272,5.954904641832326,),
        (6.519969793818825,20.4,-0.6470263865469633,5.903985734558369,),
        (8.429541072459594,24.3,-1.2472724786947993,0.004063779563288473,),
        (11.457635994174058,28.5,-1.0471904479788463,0.009880879682176973,),
        (15.82756440525919,33.35,-1.2472724786947993,6.281421548679747,),
        (22.37923397687466,38.8,-0.9471494326208812,0.03565846847561094,),
        (37.89632114260221,50.4,-1.447354509410752,6.252806668344126,),
        (50.84998945983299,59,-1.447354509410752,6.271971205547987,),
        (60.50013774395799,65.4,-1.347313494052787,0.11283795640789318,),
    )
    objects=[]
    for ident,vertices,age,kind,size in (
        (101,car,car_age,6,(4.8,2.23)),(102,bike,bike_age,8,(4.8,2.23) if large_bike else (1.8,.65))):
        i=max(0,min(len(vertices)-2,bisect.bisect_right([p[0] for p in vertices],age)-1))
        a,b=vertices[i:i+2];u=clip((age-a[0])/(b[0]-a[0]),0.,1.)
        x,y=a[1]+u*(b[1]-a[1]),a[2]+u*(b[2]-a[2]);yaw=wrap(a[3]+wrap(b[3]-a[3])*u)
        moving=0<=age<vertices[-1][0]
        vx,vy=((b[1]-a[1])/(b[0]-a[0]),(b[2]-a[2])/(b[0]-a[0])) if moving else (0.,0.)
        if ident==101 and continuing_car and age>=vertices[-1][0]:
            elapsed=age-vertices[-1][0];yaw=CASE40_HEADING
            vx,vy=9.072784515956252*math.cos(yaw),9.072784515956252*math.sin(yaw)
            x,y=x+vx*elapsed,y+vy*elapsed
        objects.append(Obstacle(ident,kind,x,y,vx,vy,yaw,*size))
    return objects


def self_test_case40():
    """Feedback plants + sudden/merged/stopped objects; no SDK or file writes."""
    import io
    assert supported_case('40.冲突对象突然出现-测试')==40
    for n in (10,13,15,17,18,20,21,22,24,26,30,31,32,33,35,37):assert n!=40
    start=EgoState(*CASE40_START,CASE40_HEADING,0.)
    assert not case40_pose_report(start)[1]
    assert case40_pose_report(EgoState(start.x+20,start.y,start.yaw,0))[1]
    with contextlib.redirect_stdout(io.StringIO()):
        # Empty but valid GT is compatible with a not-yet-visible conflict.
        c=SuddenConflictController();cmd=c.update(start,[],.02)
        assert cmd.throttle>0 and cmd.brake==0
        near=EgoState(-35.,CASE40_START[1]+(-35.-CASE40_START[0])*math.tan(CASE40_HEADING),CASE40_HEADING,7.)
        x,y=c.world(c.coordinates(near.x,near.y)[0]+18)
        stopped=Obstacle(201,8,x,y,0,0,CASE40_HEADING,1.8,.65)
        c.update(near,[stopped],.02)
        lost=c.update(near,[],.02);assert lost.brake==1 and lost.throttle==0
        # Reappearing and already opening the gap must not preserve a brake latch.
        opening=Obstacle(201,8,x+25,y,9*math.cos(CASE40_HEADING),9*math.sin(CASE40_HEADING),CASE40_HEADING,1.8,.65)
        released=c.update(near,[opening],.02);assert released.brake==0 and released.throttle>0
        invalid=Obstacle(999,8,float('nan'),0,0,0,0,1,.5)
        try:SuddenConflictController().update(start,[invalid],.02)
        except ValueError:pass
        else:raise AssertionError('Invalid case 40 obstacle accepted')
        # The bypass is forbidden on the approach/junction, and when occupied.
        c=SuddenConflictController();c.bypass_pending=101
        assert not c.plan_bypass(near,[stopped])
    scenarios=(
        # speed, period, braking, steering gain, latency, GPS offset, variant
        (9.51836121342708,.02,7.,1.,0.,.0007,''),
        (9.51836121342708,.02,7.,1.,0.,.0007,'large'),
        (0.,.02,7.,1.,0.,.0007,''),
        (9.51836121342708,.1,4.5,.93,.1,0.,'large'),
        (0.,.05,10.,1.07,.1,3.1,'large'),
        (9.51836121342708,.02,4.5,1.,.1,.0007,'hidden'),
        (9.51836121342708,.05,4.5,1.,.1,.0007,'late'),
        (9.51836121342708,.05,7.,1.,.1,.0007,'pause'),
        (0.,.05,7.,1.,0.,.0007,'continue'),
        (9.51836121342708,.05,7.,1.,.1,.0007,'left_rear'),
    )
    for v0,dt,bg,sg,lag,offset,extra in scenarios:
        c=SuddenConflictController();c.wheelbase=2.9187;c.gps_to_rear=offset
        pose=(CASE40_START[0]-offset*math.cos(CASE40_HEADING),CASE40_START[1]-offset*math.sin(CASE40_HEADING),CASE40_HEADING)
        v,t=v0,0.;car=bike=None;history=[];modes=set();signal_at=plan_at=None;min_clear=float('inf')
        with contextlib.redirect_stdout(io.StringIO()):
            for i in range(math.ceil(90/dt)):
                x,y,yaw=pose;gx,gy=x+offset*math.cos(yaw),y+offset*math.sin(yaw)
                if car is None and gx>=-134.5:car=t
                bike_gate=-35. if extra in ('late','pause') else -76.4
                if bike is None and gx>=bike_gate:bike=t
                car_age=t-car if car is not None else -.1;bike_age=t-bike if bike is not None else -.1
                if extra=='pause':bike_age-=min(3.,max(0.,bike_age-2.310809693327018))
                objects=case40_test_objects(car_age,bike_age,large_bike=extra in ('large','late'),continuing_car=extra=='continue')
                if extra=='pause' and 2.310809693327018<=t-(bike or 0)<5.310809693327018:
                    o=objects[1];objects[1]=Obstacle(o.ident,o.kind,o.x,o.y,0,0,o.yaw,o.length,o.width)
                if extra=='hidden' and gx< -35.:objects=objects[:1]
                if extra=='left_rear' and car_age>=24:
                    sx,sy=c.world(c.coordinates(30.,0)[0]+7*(car_age-24),3.5)
                    objects.append(Obstacle(301,6,sx,sy,7*c.c,7*c.s,CASE40_HEADING,4.8,2.23))
                ego=EgoState(gx,gy,yaw,v,round(t*1000),i)
                cmd=c.update(ego,objects,dt);modes.add(cmd.mode)
                assert 0<=cmd.throttle<=1 and 0<=cmd.brake<=1 and not(cmd.throttle>0 and cmd.brake>0)
                body=c.rectangle(*pose,4.57,2.21,1.7)
                assert not any(c.overlap(body,c.rectangle(o.x,o.y,o.yaw,o.length,o.width)) for o in objects),f'Case40 collision: {extra=}, {t=}, {dt=}'
                for bx,by in body:assert -1.75<c.coordinates(bx,by)[1]<5.25,f'Case40 outer/centre solid touch: {t=}, {extra=}'
                if cmd.signal_lights&2 and signal_at is None:signal_at=t
                if c.bypass is not None and plan_at is None:
                    plan_at=t
                    assert x>=28. and signal_at is not None and t-signal_at>=.9
                if cmd.distance<5:break
                history.append(cmd);held=history[max(0,len(history)-1-round(lag/dt))]
                nv=max(0.,v+(5*held.throttle-bg*held.brake-.14-.035*v)*dt)
                step,v=.5*(v+nv)*dt,nv
                pose=c.integrate(pose,step,math.tan(-sg*held.steering*FULL_STEER_RAD)/c.wheelbase);t+=dt
        label=f'{v0=}, {dt=}, {bg=}, {lag=}, {offset=}, {extra=}'
        assert t<90 and cmd.distance<5 and car is not None and bike is not None,label
        assert not any(mode.endswith('FAILSAFE') for mode in modes),label
        if extra!='continue':assert plan_at is not None and 'HAZARD_BYPASS_RETURN' in modes,label
        if extra=='left_rear':assert plan_at>car+30,label
    print('PASS case 40 offline: 10 feedback plants, timed actor endpoint stop/continuation, hidden/late/paused bicycle, '
          'merge-to-follow release, observed-hazard loss braking, exit-only swept left/return bypass and signals, '
          'left-lane rear-car wait, physical collision/solid-edge/90s checks. Not a platform score.')


def self_test():
    executable = Path(powershell_executable())
    assert executable.is_absolute() and executable.name.lower() == 'powershell.exe'
    ego = EgoState(*START, ROAD_HEADING, CRUISE_MPS)
    report, reasons = startup_pose_report(ego)
    assert not reasons and 'result=ACCEPT' in report and '25.000km/h' in report
    for dx, yaw_delta, expected in (
        (8.0, 0, ()),
        (8.01, 0, ('position_offset',)),
        (0, 0.14, ()),
        (0, 0.16, ('heading_offset',)),
        (0, -0.16, ('heading_offset',)),
        (8.01, 0.16, ('position_offset', 'heading_offset')),
        (0, 2*math.pi, ()),
    ):
        sample = EgoState(ego.x+dx, ego.y, ego.yaw+yaw_delta, ego.speed, 1234, 56)
        report, reasons = startup_pose_report(sample)
        assert reasons == expected, f'Start-pose guard changed: {dx=}, {yaw_delta=}'
        assert f'result={"REJECT" if expected else "ACCEPT"}' in report
        assert 'frame=56; timestamp_ms=1234' in report
    def obj(ident, along, lateral, speed=8.333, lateral_speed=0, length=4.8):
        x = ego.x+along*ROAD_C-lateral*ROAD_S
        y = ego.y+along*ROAD_S+lateral*ROAD_C
        return Obstacle(ident, 6, x, y, speed*ROAD_C-lateral_speed*ROAD_S,
                        speed*ROAD_S+lateral_speed*ROAD_C, ROAD_HEADING, length, 2.23)
    assert select_lead(ego, [obj(2, 8, 3.5)]) is None, 'Adjacent car falsely selected'
    lead = select_lead(ego, [obj(2, 24, 3.5, lateral_speed=-0.8)])
    assert lead and lead.predicted, 'Cut-in velocity not anticipated'
    owner = obj(0, 1.7, 0, CRUISE_MPS, length=4.57)
    assert find_self_id(ego, [owner, obj(1, 30, 0)]) == 0
    assert select_lead(ego, [owner, obj(1, 30, 0)], 0).ident == 1
    rolling = CutInController().update(
        ego, [owner, obj(1, 30, 0, speed=30/3.6), obj(2, 0.3, 3.5, speed=30/3.6)],
        0.02, self_id=0,
    )
    assert rolling.mode == 'FOLLOW' and rolling.lead.ident == 1
    assert rolling.target_speed == CRUISE_MPS and rolling.brake == 0 and rolling.throttle > 0, \
        'Safe moving start with a faster lead was unnecessarily braked'
    for gap in (12.0, 14.0, 18.0, 23.0):
        opening = CutInController().update(
            ego, [obj(1, gap+GPS_TO_FRONT_M+2.4, 0, speed=30/3.6)], 0.02,
        )
        assert opening.mode == 'FOLLOW' and opening.target_speed == CRUISE_MPS
        assert opening.brake == 0 and opening.throttle > 0, \
            f'Faster lead caused unnecessary slowdown at {gap=}m'
    clear_cutin = CutInController().update(
        ego, [obj(2, 21.0, 3.5, speed=30/3.6, lateral_speed=-0.8)], 0.02,
    )
    assert clear_cutin.mode == 'CUTIN_PREDICT' and clear_cutin.brake == 0
    assert clear_cutin.target_speed == CRUISE_MPS
    closing_lead = CutInController().update(
        ego, [obj(1, 15.0+GPS_TO_FRONT_M+2.4, 0, speed=20/3.6)], 0.02,
    )
    assert closing_lead.mode == 'FOLLOW_GAP' and 0 < closing_lead.target_speed < CRUISE_MPS
    assert closing_lead.brake > 0 and closing_lead.throttle == 0
    near_goal = EgoState(ego.x+(ROUTE_LENGTH-10)*ROAD_C,
                        ego.y+(ROUTE_LENGTH-10)*ROAD_S, ego.yaw, ego.speed)
    arrival = CutInController().update(near_goal, [], 0.02)
    assert arrival.mode == 'CRUISE' and arrival.target_speed == CRUISE_MPS and arrival.brake == 0
    endpoint = EgoState(ego.x+(ROUTE_LENGTH-0.5)*ROAD_C,
                       ego.y+(ROUTE_LENGTH-0.5)*ROAD_S, ego.yaw, ego.speed)
    assert CutInController().update(endpoint, [], 0.02).mode == 'GOAL_HOLD'
    far = CutInController().update(ego, [obj(1, 80, 0)], 0.05)
    assert far.throttle >= 0 and far.brake == 0 and abs(far.steering) < 1e-6
    urgent = CutInController().update(ego, [obj(1, 15, 0, speed=0)], 0.05)
    assert urgent.mode == 'AEB' and urgent.throttle == 0 and urgent.brake == 1
    offroad = EgoState(ego.x-2*ROAD_S, ego.y+2*ROAD_C, ego.yaw, ego.speed)
    assert CutInController().update(offroad, [], 0.05).mode == 'LANE_FAILSAFE'
    target = EgoState(ego.x-0.25*ROAD_S, ego.y+0.25*ROAD_C, ego.yaw, ego.speed)
    assert CutInController().update(target, [], 0.05).steering > 0, 'Steering sign is reversed'
    for bad in (float('nan'), float('inf')):
        try:
            CutInController().update(EgoState(ego.x, ego.y, ego.yaw, bad), [], 0.05)
        except ValueError:
            pass
        else:
            raise AssertionError('Non-finite GPS accepted')
    self_test_cutin()
    self_test_case13()
    self_test_case13_recovery_and_signals()
    self_test_case10()
    self_test_case35()
    self_test_case18()
    self_test_batch()
    self_test_expanded()
    self_test_case40()
    from ParkingAPA import self_test_evaluation
    self_test_evaluation()  # Pure mocks; does not run ParkingAPA's main/SDK.
    assert not any(name in sys.modules for name in
                   ('SimOneIOStruct', 'SimOneSensorAPI', 'SimOnePNCAPI', 'SimOneServiceAPI'))
    print('PASS offline: absolute PowerShell path, startup pose diagnostics/guard, cruise priority, safe moving start, LKA steering sign, ACC, AEB, adjacent/cut-in selection, '
          'self-ID, invalid data, evaluation cleanup, measured cut-in regressions, '
          'early prediction, emergency release, multi-vehicle risks, toy feedback at 3 frame intervals. '
          'Case 13 adaptive staging, measured failure regression, indicators, '
          'safe shifts, swept collision guards and 11 toy plants passed. '
          'Case 10 live-light decoding, empty GT policy, stop/launch and 162 toy plants passed. '
          'Case 35 live VRU yield and map-derived right turn passed. '
          'Case 18 live cross-traffic prediction and yield-release regressions passed. '
          'New cases 15/17/31 batch checks passed. '
          'Expanded cases 20/21/22/26/30/32/33/37 checks passed. '
          'Case 40 sudden conflict/merge/exit bypass checks passed. '
          'No SDK connection or score.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--stop', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return 0
    directory = Path(__file__).resolve().parent
    lock, stop = directory/'.AutoDrive.lock', directory/'.AutoDrive.stop'
    if args.stop:
        if lock.exists() and pid_alive(pid := int(lock.read_text(encoding='ascii').strip())):
            stop.write_text(str(pid), encoding='ascii')
            print(f'Graceful AutoDrive stop requested for PID {pid}; no force-kill.')
        else:
            print('No active AutoDrive process.')
        return 0
    # Check every prerequisite before SDK import/connection or taking API control.
    if not (directory/'ParkingAPA.py').is_file():
        raise RuntimeError('ParkingAPA.py evaluation helpers missing; no vehicle control started')
    ensure_no_other_controller(directory)
    if lock.exists():
        pid = int(lock.read_text(encoding='ascii').strip())
        if pid_alive(pid):
            raise RuntimeError(f'AutoDrive already active: PID={pid}')
        lock.unlink()  # Own confirmed stale PID file only.
    fd = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
    try:
        os.write(fd, str(os.getpid()).encode('ascii'))
    finally:
        os.close(fd)
    stem = time.strftime('AutoDrive-%Y%m%d-%H%M%S')+f'-{os.getpid()}-{time.time_ns()%1000000:06d}'
    try:
        with (directory/(stem+'.log')).open('x', encoding='utf-8') as log:
            with contextlib.redirect_stdout(Tee(sys.stdout, log)), contextlib.redirect_stderr(Tee(sys.stderr, log)):
                print(f'AutoDrive cases 10/13/15/17/18/20/21/22/24/26/30/31/32/33/35/37/40; PID={os.getpid()}; Python={sys.executable}', flush=True)
                print(f'LOG={directory/(stem+".log")}; CSV={directory/(stem+".csv")}', flush=True)
                try:
                    return run_live(directory, directory/(stem+'.csv'), stop)
                except (Exception, KeyboardInterrupt):
                    traceback.print_exc()
                    print('AutoDrive startup failed; traceback retained in this log.', flush=True)
                    return 1
    finally:
        if lock.exists() and lock.read_text(encoding='ascii').strip() == str(os.getpid()):
            lock.unlink()
        if stop.exists() and stop.read_text(encoding='ascii').strip() == str(os.getpid()):
            stop.unlink()


if __name__ == '__main__':
    raise SystemExit(main())
