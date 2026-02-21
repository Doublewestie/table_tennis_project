"""
全局配置参数
"""

# ========== 机器人运动模式 ==========
ABS = 0     # 绝对运动
INCR = 1    # 增量运动
BLOCK = True
NON_BLOCK = False

# ========== 坐标系 ==========
COORD_BASE = 0   # 基坐标系/当前用户坐标系
COORD_JOINT = 1  # 关节空间
COORD_TOOL = 2   # 工具坐标系

# ========== 机器人网络参数 ==========
ROBOT_IP = "192.168.56.101"   # 请根据实际修改

# ========== 运动速度参数 ==========
LINEAR_SPEED = 200          # 直线运动速度 (mm/s)
JOINT_SPEED = 100           # 关节运动速度 (rad/s? 实际为 rad/s，但文档中 joint_move speed 单位 rad/s)
ARC_SPEED = 200             # 圆弧速度 (mm/s)
ARC_ACC = 100               # 圆弧加速度 (mm/s²)

# ========== 击球相关参数 ==========
TARGET_X = 400              # 拦截位置的 X 坐标 (mm)，需在击球区内
RACKET_RADIUS = 50          # 球拍半径 (mm)，用于判断是否命中
HIT_Z = 20                  # 击球时球拍在 Z 方向的高度 (mm) 桌面以上

# ========== 轨迹拟合参数 ==========
SPLIT_THRESHOLD = 500       # 轨迹分割的 X 突变阈值
FIT_METHOD = 'quadratic'    # 拟合方法：linear / quadratic / cubic
FIT_X_LIMIT = 1500          # 只使用 x < 1500 的数据进行拟合（避免远端噪声）

# ========== 电机参数 ==========
'''真实电机参数,仿真时无效'''
MOTOR_PORT = "COM3"         # 翎控电机串口号（Windows 示例）
MOTOR_BAUDRATE = 115200     # 默认波特率
MOTOR_ID = 1                # 电机 ID（默认 1）

# ========== 仿真模式设置 ==========
SIMULATION_MODE = True          # True 表示在仿真环境中运行，False 表示连接真实硬件