"""
JAKA机器人 SDK 绘图示例 - 常量配置模块
"""
import math
# ========== 运动模式 ==========
ABS = 0     # 绝对运动
INCR = 1    # 增量运动
CONT = 2    # 连续运动（仅 jog）

# ========== 阻塞模式 ==========
BLOCK = True
NON_BLOCK = False

# ========== 坐标系 ==========
COORD_BASE = 0   # 用户坐标系设置
COORD_TOOL = 3   # 工具坐标系设置
COORD_JOINT = 1  # 关节空间

# ========== 机器人默认参数 ==========
DEFAULT_ROBOT_IP = "192.168.56.101"   # 请根据实际修改
DEFAULT_LINEAR_SPEED = 150           # 直线运动默认速度 (mm/s)
DEFAULT_JOINT_SPEED = 75            # 关节运动默认速度 (mm/s)
DEFAULT_ARC_SPEED = 150              # 圆弧运动默认速度 (mm/s)
DEFAULT_ARC_ACC = 75                 # 圆弧运动默认加速度 (mm/s²)

# ========== 直线绘画参数 ==========
joint_pose_1 = [-48.187*math.pi/180,
                 71.607*math.pi/180,
                 -0.192*math.pi/180,
                 26.315*math.pi/180,
                -12.874*math.pi/180,
                  6.240*math.pi/180]

joint_pose_2 = [-33.676*math.pi/180,
                 85.372*math.pi/180,
                 -5.429*math.pi/180,
                    0.0*math.pi/180,
                -79.944*math.pi/180,
                 33.676*math.pi/180]

joint_pose_3 = [  21.576*math.pi/180,
                 105.618*math.pi/180,
                 -80.081*math.pi/180,
                 227.229*math.pi/180,
                 -40.189*math.pi/180,
                -236.722*math.pi/180]