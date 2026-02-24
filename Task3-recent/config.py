"""
全局配置参数
"""
import numpy as np

# ========== 机器人运动模式 ==========
ABS = 0     # 绝对运动
INCR = 1    # 增量运动
BLOCK = True
NON_BLOCK = False

# ========== 坐标系 ==========
COORD_BASE = 4   # 用户坐标系设置
COORD_TOOL = 2   # 工具坐标系设置
COORD_JOINT = 1  # 关节空间

# ========== 机器人网络参数 ==========
ROBOT_IP = "192.168.56.101"   # 请根据实际修改

# ========== 运动速度参数 ==========
LINEAR_SPEED = 200          # 直线运动速度 (mm/s)
JOINT_SPEED = 100           # 关节运动速度 (rad/s? 实际为 rad/s，但文档中 joint_move speed 单位 rad/s)
ARC_SPEED = 200             # 圆弧速度 (mm/s)
ARC_ACC = 100               # 圆弧加速度 (mm/s²)

# ========== 击球相关参数 ==========
TARGET_X = 225              # 拦截位置的 X 坐标 (mm)，需在击球区内
RACKET_RADIUS = 50          # 球拍半径 (mm)，用于判断是否命中
HIT_Z = 40                  # 击球时球拍在 Z 方向的高度 (mm) 桌面以上

# ========== 零点相关参数 ==========
JOINT_ORIGIN=[0, 0, 0, 0, 0, 0]   # 全局关节零点设置
JOINT_SET=[  -0.871*np.pi/180, 
             63.672*np.pi/180, 
             15.442*np.pi/180,
              0.811*np.pi/180, 
             52.936*np.pi/180, 
           -181.071*np.pi/180]

# ========== 轨迹分割参数 ==========
SPLIT_THRESHOLD = 500       # X 突变阈值，用于区分不同轨迹

# ========== 轨迹预测v2参数==========
WEIGHT_QUAD = 0.825          # 二次回归的权重（线性回归权重为 1-WEIGHT_QUAD）

# 三轮精确定位的 X 范围（单位：mm）
STAGE1_RANGE = (1600, 2150)   # 初步调整阶段
STAGE2_RANGE = (1000, 1550)   # 第二轮调整
STAGE3_RANGE = (400, 700)     # 第三轮最终调整

# ========== 电机参数 ==========
'''真实电机参数,仿真时无效'''
MOTOR_PORT = "COM3"         # 翎控电机串口号
MOTOR_BAUDRATE = 115200     # 默认波特率
MOTOR_ID = 1                # 电机 ID（默认 1）

# ========== 仿真模式设置 ==========
SIMULATION_MODE = True          # True 表示在仿真环境中运行，False 表示连接真实硬件