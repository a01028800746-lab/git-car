# -*- coding: utf-8 -*-  # 声明文件编码为UTF-8，避免中文注释乱码

import math  # 引入数学库，用于三角函数、角度换算、开方等运算
import time  # 引入时间库，用于控制循环节拍、计算帧间隔dt

from SimOneIOStruct import *  # 引入SimOne平台的数据结构定义（如SimOne_Data_Gps、SimOne_Data_Control等）
from SimOnePNCAPI import *  # 引入规划与控制类API（Planning and Control），如SoSetDrive等下发控制指令的接口
from SimOneSensorAPI import SoGetGps  # 单独引入传感器API中的SoGetGps，用于获取主车GPS/位姿数据
from SimOneServiceAPI import *  # 引入服务类API，如SoInitSimOneAPI（初始化连接）、SoGetCaseRunStatus（查询案例运行状态）等


MAIN_VEHICLE_ID = "Ego"  # 主车ID，需与平台/控制器里配置的主车标识一致，这里用字符串"Ego"而非示例手册里的"0"
TARGET_SPEED_MPS = 8.333333333333332  # 目标巡航速度，单位米/秒，约等于30km/h（8.333*3.6≈30）
CONTROL_PERIOD_S = 0.05  # 控制循环周期，单位秒，0.05s即20Hz，比手册示例的10Hz更快、控制更精细
WHEELBASE_M = 2.85  # 车辆轴距，单位米，用于Pure Pursuit（纯追踪）算法计算转向角，需与主车实际轴距匹配
MAX_ROAD_WHEEL_ANGLE_RAD = math.radians(32.0)  # 前轮最大转角，32度转换成弧度，用于将物理转角换算成百分比控制量

# Fitted-clothoid vertices shown in the NEVC LKA case editor.  # 注释：以下路径点是从NEVC平台LKA案例编辑器里读出的拟合曲线关键点
# Each item is (x, y, heading in radians).  # 注释：每个元组是(x坐标, y坐标, 航向角弧度)
ROUTE_VERTICES = [  # 定义参考路径的关键点列表，作为车道保持要跟随的几何路径骨架
    (-27.852001192621447, -124.3186791351911, 1.5852880658493673),  # 第1个路径点：起点附近坐标与航向角
    (-28.855777400204243, -55.05812075917717, 1.5852880658493536),  # 第2个路径点
    (-29.70837380339802, 3.771031106043393, 1.585288065849348),  # 第3个路径点
    (-22.892132262370914, 56.33067469280025, 1.3000805809133036),  # 第4个路径点，航向角开始变化，说明这里进入弯道
    (-4.537285403912831, 103.08089961071268, 1.105993292358437),  # 第5个路径点，继续弯曲
    (17.378466605521172, 141.04161667071665, 0.995217664847178),  # 第6个路径点：终点坐标与航向角
]


def clamp(value, minimum, maximum):  # 定义通用限幅函数，把value限制在[minimum, maximum]区间内
    return max(minimum, min(value, maximum))  # 先取value和maximum的较小值，再和minimum取较大值，实现双向限幅


def normalize_angle(angle):  # 定义角度归一化函数，把任意角度换算到(-π, π]区间，避免角度突变（如359度和1度的跳变问题）
    return (angle + math.pi) % (2.0 * math.pi) - math.pi  # 经典的角度归一化公式


def build_reference_path(vertices, spacing_m=0.75):  # 定义函数：把稀疏的路径关键点插值成密集的参考路径点，采样间距默认0.75米
    """Interpolate the route vertices with heading-aware cubic Hermite curves."""  # 文档字符串：用考虑航向角的三次Hermite曲线对路径点进行插值
    path = []  # 初始化空列表，用于存放插值后的密集路径点
    for segment_index in range(len(vertices) - 1):  # 遍历每一段相邻路径点之间的区间（点数-1段）
        x0, y0, heading0 = vertices[segment_index]  # 取出当前段起点的x、y坐标和航向角
        x1, y1, heading1 = vertices[segment_index + 1]  # 取出当前段终点的x、y坐标和航向角
        segment_length = math.hypot(x1 - x0, y1 - y0)  # 计算起点到终点的直线距离，作为该段长度的近似值
        sample_count = max(2, int(math.ceil(segment_length / spacing_m)))  # 根据段长和采样间距计算需要插值的采样点数，至少2个

        tangent0_x = math.cos(heading0) * segment_length  # 计算起点切向量的x分量（用航向角和段长构造，用于Hermite插值控制曲线走向）
        tangent0_y = math.sin(heading0) * segment_length  # 计算起点切向量的y分量
        tangent1_x = math.cos(heading1) * segment_length  # 计算终点切向量的x分量
        tangent1_y = math.sin(heading1) * segment_length  # 计算终点切向量的y分量

        for sample_index in range(sample_count):  # 在当前段内按采样点数进行循环插值
            u = sample_index / sample_count  # 计算归一化插值参数u，范围[0,1)，表示在本段内的相对位置
            u2 = u * u  # 预计算u的平方，供下面Hermite基函数使用
            u3 = u2 * u  # 预计算u的立方
            h00 = 2.0 * u3 - 3.0 * u2 + 1.0  # 三次Hermite插值的第一个基函数（控制起点位置权重）
            h10 = u3 - 2.0 * u2 + u  # 第二个基函数（控制起点切向量权重）
            h01 = -2.0 * u3 + 3.0 * u2  # 第三个基函数（控制终点位置权重）
            h11 = u3 - u2  # 第四个基函数（控制终点切向量权重）
            x = h00 * x0 + h10 * tangent0_x + h01 * x1 + h11 * tangent1_x  # 用四个基函数加权组合，算出插值点的x坐标
            y = h00 * y0 + h10 * tangent0_y + h01 * y1 + h11 * tangent1_y  # 同理算出插值点的y坐标
            path.append((x, y))  # 把插值出的(x, y)点加入路径列表

    path.append((vertices[-1][0], vertices[-1][1]))  # 循环结束后，把最后一个原始路径点补充进去，确保路径终点精确
    return path  # 返回插值后的密集参考路径点列表


def find_nearest_index(path, x, y, previous_index):  # 定义函数：在参考路径上找到离当前车辆位置(x,y)最近的点的索引
    start_index = max(0, previous_index - 8)  # 从上一次最近点索引往前回退8个点作为搜索起点，避免全路径搜索、提高效率（车辆只会前进）
    return min(  # 返回搜索范围内使距离最小的索引
        range(start_index, len(path)),  # 搜索区间：从start_index到路径末尾
        key=lambda index: (path[index][0] - x) ** 2 + (path[index][1] - y) ** 2,  # 以到(x,y)的欧氏距离平方作为比较依据（省去开方运算）
    )


def find_lookahead_index(path, nearest_index, lookahead_distance):  # 定义函数：从最近点开始沿路径累加距离，找到预瞄距离对应的目标点索引
    distance = 0.0  # 初始化累计距离为0
    target_index = nearest_index  # 目标索引初始化为最近点索引
    while target_index < len(path) - 1 and distance < lookahead_distance:  # 只要还没到路径末尾且累计距离未达到预瞄距离，就继续前进
        x0, y0 = path[target_index]  # 取当前目标点坐标
        x1, y1 = path[target_index + 1]  # 取下一个点坐标
        distance += math.hypot(x1 - x0, y1 - y0)  # 累加这一小段的长度到总距离
        target_index += 1  # 索引前移一位
    return target_index  # 返回达到预瞄距离时对应的路径点索引


def pure_pursuit_steering(path, nearest_index, x, y, yaw, speed):  # 定义纯追踪（Pure Pursuit）转向角计算函数，输入路径、最近点、车辆位姿和速度
    lookahead_distance = clamp(3.5 + 0.35 * speed, 4.0, 7.0)  # 根据车速动态计算预瞄距离：车速越快看得越远，限制在4~7米之间
    target_index = find_lookahead_index(path, nearest_index, lookahead_distance)  # 调用前面的函数，找到预瞄目标点索引
    target_x, target_y = path[target_index]  # 取出预瞄目标点的坐标

    target_heading = math.atan2(target_y - y, target_x - x)  # 计算车辆当前位置指向目标点的方位角
    heading_error = normalize_angle(target_heading - yaw)  # 计算目标方位角与车辆当前航向角之间的误差，并归一化
    actual_lookahead = max(1.0, math.hypot(target_x - x, target_y - y))  # 计算车辆到目标点的实际直线距离，最小限制为1米（防止除零/过小导致转角异常）
    road_wheel_angle = math.atan2(  # 按Pure Pursuit经典公式计算前轮转角（弧度）
        2.0 * WHEELBASE_M * math.sin(heading_error),  # 公式分子：2倍轴距乘以航向误差的正弦
        actual_lookahead,  # 公式分母：实际预瞄距离
    )

    # SimOne percent mode uses negative for left and positive for right.  # 注释：SimOne平台的百分比转向模式中，负值代表左转，正值代表右转
    steering = -road_wheel_angle / MAX_ROAD_WHEEL_ANGLE_RAD  # 把物理转角换算成[-1,1]的百分比控制量，并取负号以匹配平台左右符号约定
    return clamp(steering, -0.65, 0.65), target_index  # 对转向量做限幅（最大用到65%转向幅度，留有余量），并返回目标点索引供调用方使用


class SpeedController:  # 定义速度控制器类，负责根据目标速度与当前速度算出油门/刹车
    def __init__(self):  # 类的构造函数
        self.integral_error = 0.0  # 初始化积分误差项为0，用于PI控制器的积分部分

    def update(self, target_speed, current_speed, dt):  # 定义更新方法：输入目标速度、当前速度、本帧时间间隔dt
        error = target_speed - current_speed  # 计算速度误差：目标速度减去当前速度
        self.integral_error = clamp(  # 更新积分误差，并做限幅防止积分饱和（windup）
            self.integral_error + error * dt,  # 误差乘以dt累加到积分项（离散积分）
            -4.0,  # 积分误差下限
            4.0,  # 积分误差上限
        )
        command = 0.16 * error + 0.025 * self.integral_error  # 计算PI控制输出：比例项(0.16*error)加积分项(0.025*integral_error)

        if target_speed < 0.1:  # 特殊情况：如果目标速度接近0（比如需要停车/到达终点）
            return 0.0, clamp(0.25 + 0.12 * current_speed, 0.25, 0.8)  # 油门给0，刹车按当前速度比例增大（车速越快刹车越大），限制在0.25~0.8之间，保证能停住
        if command >= 0.0:  # 如果PI输出为正，说明需要加速
            return clamp(0.12 + command, 0.0, 0.70), 0.0  # 油门 = 基础油门0.12加上控制量，限幅到0~0.70；刹车为0
        return 0.0, clamp(-0.35 * command, 0.0, 0.55)  # 如果PI输出为负，说明需要减速：油门为0，刹车 = 控制量的绝对值乘以0.35，限幅到0~0.55


def target_speed_for_goal(distance_to_goal):  # 定义函数：根据离终点的剩余距离，动态计算目标速度（实现临近终点前减速的效果）
    if distance_to_goal <= 2.0:  # 如果离终点距离小于等于2米
        return 0.0  # 目标速度直接设为0，即准备停车
    if distance_to_goal < 18.0:  # 如果距离在2~18米之间，属于减速区
        return min(  # 目标速度取以下两者的较小值
            TARGET_SPEED_MPS,  # 正常巡航速度上限
            max(1.5, math.sqrt(2.0 * 1.2 * (distance_to_goal - 2.0))),  # 按匀减速运动学公式v=√(2as)反推允许速度，1.2是假设的减速度，最低1.5m/s防止过早停滞
        )
    return TARGET_SPEED_MPS  # 距离终点还很远（≥18米），直接使用正常巡航目标速度


def make_control(steering, throttle, brake):  # 定义函数：把转向、油门、刹车数值打包成SimOne平台需要的控制指令结构体
    control = SimOne_Data_Control()  # 创建一个空的控制指令数据结构实例
    control.EThrottleMode = ESimOne_Throttle_Mode.ESimOne_Throttle_Mode_Percent  # 设置油门控制模式为"百分比模式"（而非具体扭矩等其他单位）
    control.EBrakeMode = ESimOne_Brake_Mode.ESimOne_Brake_Mode_Percent  # 设置刹车控制模式为"百分比模式"
    control.ESteeringMode = ESimOne_Steering_Mode.ESimOne_Steering_Mode_Percent  # 设置转向控制模式为"百分比模式"
    control.steering = steering  # 把计算出的转向百分比赋值给控制结构体
    control.throttle = throttle  # 把计算出的油门百分比赋值给控制结构体
    control.brake = brake  # 把计算出的刹车百分比赋值给控制结构体
    control.handbrake = False  # 不使用手刹
    control.isManualGear = False  # 不使用手动挡，采用自动挡模式
    control.gear = ESimOne_Gear_Mode.ESimOne_Gear_Mode_Drive  # 挡位设置为前进挡(D挡)
    return control  # 返回填好的控制指令结构体，供SoSetDrive下发使用


def run_lka():  # 定义主函数：运行车道保持(LKA)算法的完整流程
    reference_path = build_reference_path(ROUTE_VERTICES)  # 调用插值函数，把稀疏关键点转换成密集参考路径，供后续每一帧查找最近点/预瞄点使用

    # This SDK version can return 0 even after the connection succeeds.  # 注释：当前这版SDK即使连接成功，返回值也可能是0（即不能仅凭返回值判断是否成功）
    # The SDK log is the reliable initialization result.  # 注释：需要以SDK自身打印的日志作为初始化是否成功的可靠依据
    SoInitSimOneAPI(MAIN_VEHICLE_ID, 0, "127.0.0.1")  # 初始化与仿真平台的连接：传入主车ID、保留参数0、本机回环地址
    SoSetDriverName(MAIN_VEHICLE_ID, "LKA")  # 设置本次驾驶名称为"LKA"，便于在平台界面/日志中识别对应关系
    SoSetDriveMode(MAIN_VEHICLE_ID, ESimOne_Drive_Mode(0))  # 设置驾驶模式为API模式（模式0），即由外部算法接管车辆，而非平台内置逻辑
    SimoneAPI.InitEvaluationServiceWithLocalData(MAIN_VEHICLE_ID.encode("utf-8"))  # 初始化评价服务（注意：变量名是SimoneAPI，与前面import的SimOneServiceAPI要保证实际可用，否则本局不会被云端计分），主车ID需编码为字节串

    speed_controller = SpeedController()  # 实例化速度控制器对象，供每帧调用update()计算油门/刹车
    nearest_index = 0  # 初始化"最近路径点索引"为0，作为第一帧查找最近点的起始猜测值
    previous_steering = 0.0  # 初始化"上一帧转向量"为0，用于后面做转向变化率限制（防止转向突变）
    previous_time = time.monotonic()  # 记录当前单调时钟时间，作为计算帧间隔dt的基准
    last_status_time = 0.0  # 初始化"上次打印状态信息的时间"为0，用于控制日志打印频率
    gps_failure_count = 0  # 初始化"GPS获取失败计数器"为0，用于统计连续获取不到GPS数据的次数

    try:  # 开始异常捕获块，保证程序被中断时依然能执行清理逻辑
        while True:  # 主控制循环，每次循环处理一帧
            if SoGetCaseRunStatus() == ESimOne_Case_Status.ESimOne_Case_Status_Stop:  # 查询当前案例（场景）运行状态，如果已经是"停止"状态
                break  # 跳出主循环，准备结束程序、保存评价结果

            loop_start = time.monotonic()  # 记录本帧循环开始的时间戳
            dt = clamp(loop_start - previous_time, 0.01, 0.20)  # 计算本帧与上一帧的时间间隔dt，并限幅在0.01~0.20秒之间（防止异常大/小的dt影响控制器计算）
            previous_time = loop_start  # 更新"上一帧时间"为本帧开始时间，供下一次循环计算dt使用

            gps = SimOne_Data_Gps()  # 创建一个空的GPS/位姿数据结构实例，用于接收本帧车辆状态
            if not SoGetGps(MAIN_VEHICLE_ID, gps):  # 调用API获取主车GPS数据，若返回False表示获取失败
                gps_failure_count += 1  # 失败计数器加1
                if gps_failure_count % 20 == 1:  # 每失败20次打印一次提示（避免日志刷屏）
                    print("Waiting for GPS data...")  # 打印等待GPS数据的提示信息
                time.sleep(CONTROL_PERIOD_S)  # 按控制周期休眠，等待下一次重试
                continue  # 跳过本次循环剩余部分，直接进入下一次while循环
            gps_failure_count = 0  # 成功获取GPS数据后，把失败计数器重置为0

            x = float(gps.posX)  # 提取车辆当前x坐标（后轴中心位置），并转换为浮点数
            y = float(gps.posY)  # 提取车辆当前y坐标
            yaw = float(gps.oriZ)  # 提取车辆当前偏航角（朝向），oriZ对应偏航角
            speed = math.sqrt(gps.velX ** 2 + gps.velY ** 2 + gps.velZ ** 2)  # 用三个方向的速度分量合成车辆的标量速度（速度大小）

            nearest_index = find_nearest_index(reference_path, x, y, nearest_index)  # 调用函数，基于当前位置更新"最近路径点索引"
            steering, target_index = pure_pursuit_steering(  # 调用Pure Pursuit函数，计算本帧应打的转向量，以及使用的预瞄目标点索引
                reference_path,  # 传入参考路径
                nearest_index,  # 传入最近点索引
                x,  # 传入车辆x坐标
                y,  # 传入车辆y坐标
                yaw,  # 传入车辆航向角
                speed,  # 传入车辆当前速度
            )
            steering = clamp(  # 对本帧转向量做"变化率限制"，限制其相对上一帧转向量的变化幅度
                steering,  # 待限幅的本帧转向量
                previous_steering - 0.07,  # 允许的最小值：上一帧转向减0.07
                previous_steering + 0.07,  # 允许的最大值：上一帧转向加0.07（防止方向盘打得过猛，保证平顺性）
            )
            previous_steering = steering  # 更新"上一帧转向量"为限幅后的本帧转向量，供下一帧使用

            goal_x, goal_y, _ = ROUTE_VERTICES[-1]  # 取出路径终点的x、y坐标（忽略航向角，用下划线占位）
            distance_to_goal = math.hypot(goal_x - x, goal_y - y)  # 计算车辆当前位置到终点的直线距离
            target_speed = target_speed_for_goal(distance_to_goal)  # 根据到终点的距离，计算本帧目标速度（实现临近终点减速停车）
            throttle, brake = speed_controller.update(target_speed, speed, dt)  # 调用速度控制器，计算本帧油门和刹车量

            control = make_control(steering, throttle, brake)  # 把转向、油门、刹车打包成平台需要的控制指令结构体
            if not SoSetDrive(MAIN_VEHICLE_ID, control):  # 调用API把控制指令下发给仿真引擎，若返回False表示下发失败
                print("Warning: SoSetDrive failed")  # 打印下发失败的警告信息，便于排查通信问题

            if loop_start - last_status_time >= 1.0:  # 如果距离上次打印状态信息已经过去至少1秒
                print(  # 打印一行状态日志，方便实时观察算法运行情况
                    "pos=({:.1f}, {:.1f}) speed={:.2f}/{:.2f} m/s "  # 格式化字符串第一部分：位置、当前速度/目标速度
                    "steer={:.3f} target={}/{} goal={:.1f} m".format(  # 格式化字符串第二部分：转向量、目标点索引/路径总点数、离终点距离
                        x,  # 填入x坐标
                        y,  # 填入y坐标
                        speed,  # 填入当前速度
                        target_speed,  # 填入目标速度
                        steering,  # 填入本帧转向量
                        target_index,  # 填入预瞄目标点索引
                        len(reference_path) - 1,  # 填入参考路径最后一个点的索引（作为总点数参照）
                        distance_to_goal,  # 填入到终点的剩余距离
                    )
                )
                last_status_time = loop_start  # 更新"上次打印时间"为本帧时间，重新开始计时下一次打印间隔

            elapsed = time.monotonic() - loop_start  # 计算本帧从开始到现在实际消耗的时间
            time.sleep(max(0.0, CONTROL_PERIOD_S - elapsed))  # 用控制周期减去已消耗时间，得到剩余需要休眠的时间（不小于0），保证循环节拍稳定在CONTROL_PERIOD_S左右
    except KeyboardInterrupt:  # 捕获用户手动中断（Ctrl+C）的异常
        print("LKA stopped by user")  # 打印提示信息，说明是被用户手动停止的
    finally:  # 无论是正常break结束、异常结束还是被中断，都会执行finally块，保证收尾动作一定被执行
        SoSetDrive(MAIN_VEHICLE_ID, make_control(0.0, 0.0, 1.0))  # 结束前下发一帧"转向0、油门0、刹车拉满"的安全停车指令，避免车辆带着惯性乱冲
        SimoneAPI.SaveEvaluationRecord()  # 保存本局的评价记录，提交给平台计分（这一步遗漏会导致本局分数丢失）


if __name__ == "__main__":  # Python标准写法：判断当前脚本是否作为主程序直接运行（而非被其他模块import）
    run_lka()  # 是主程序运行时，调用run_lka()函数，正式启动整个LKA算法流程
