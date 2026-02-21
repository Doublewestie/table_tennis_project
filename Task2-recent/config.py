"""
JAKA机器人 SDK 绘图示例 - 常量配置模块
"""

# ========== 运动模式 ==========
ABS = 0     # 绝对运动
INCR = 1    # 增量运动
CONT = 2    # 连续运动（仅 jog）

# ========== 阻塞模式 ==========
BLOCK = True
NON_BLOCK = False

# ========== 坐标系 ==========
COORD_BASE = 0   # 基坐标系/当前用户坐标系
COORD_JOINT = 1  # 关节空间
COORD_TOOL = 2   # 工具坐标系

# ========== 机器人默认参数 ==========
DEFAULT_ROBOT_IP = "192.168.56.101"   # 请根据实际修改
DEFAULT_LINEAR_SPEED = 200           # 直线运动默认速度 (mm/s)
DEFAULT_JOINT_SPEED = 100            # 关节运动默认速度 (mm/s)
DEFAULT_ARC_SPEED = 200              # 圆弧运动默认速度 (mm/s)
DEFAULT_ARC_ACC = 100                 # 圆弧运动默认加速度 (mm/s²)