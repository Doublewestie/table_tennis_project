"""
机器人基础控制模块
依赖: jkrc, config
"""

import jkrc
import time
import math
import config
import numpy as np

 # 安全连接并上电使能
def secure_boot(ip=config.ROBOT_IP):
    robot = jkrc.RC(ip)
    for act, name in [(robot.login, "登录"),
                      (robot.power_on, "电源打开"),
                      (robot.enable_robot, "上使能")]:
        ret = act()
        if ret[0] == 0:
            print(f"[OK] {name}成功")
        else:
            raise Exception(f"[ERROR] {name}失败，错误码：{ret[0]}")
    print(f"--机器人 [{ip}] 已连接并上电使能--")
    return robot

 # 安全下电并登出
def safe_shutdown(robot):
    for act, name in [(robot.disable_robot, "下使能"),
                      (robot.power_off, "断电"),
                      (robot.logout, "登出")]:
        ret = act()
        if ret[0] == 0:
            print(f"[OK] {name}成功")
        else:
            raise Exception(f"[ERROR] {name}失败，错误码：{ret[0]}")
    print("--机器人已登出--")

 # 回归全局零点
def return_to_origin(robot,origin=config.JOINT_ORIGIN,joint_speed=config.JOINT_SPEED):
    ret = robot.joint_move(origin, config.ABS, config.BLOCK, joint_speed)
    if ret[0] != 0:
        raise Exception(f"--原点回归失败，错误码：{ret[0]}--")
    else:
        print("--全局零点回归就位--")

# 回归运动零点
def return_to_set_point(robot, set_point=config.JOINT_SET,joint_speed=config.JOINT_SPEED):
    ret = robot.joint_move(set_point, config.ABS, config.BLOCK, joint_speed)
    if ret[0] != 0:
        raise Exception(f"--原点回归失败，错误码：{ret[0]}--")
    else:
        print("--运动零点回归就位--")

"""
:param robot: 已登录且上电使能的机器人对象
:param target_pose: 目标笛卡尔位姿 [x, y, z, rx, ry, rz] (单位：mm, rad)
:param ref_joint: 参考关节角度（可选，默认使用当前关节角度）
:return: 成功返回关节角度列表（6个元素，单位 rad），失败返回 None
"""
 # 计算目标位姿在当前工具和用户坐标系下的逆解
def compute_inverse_kinematics(robot, target_pose):
    ret, ref_joint = robot.get_joint_position()
    if ret != 0:
        print("[ERROR] 获取当前关节角度失败，错误码：", ret)
        return None

    try:
        result = robot.kine_inverse(ref_joint, target_pose)   # 调用逆解，处理不同返回值格式
    except Exception as e:
        print(f"[ERROR] 逆解调用异常: {e}")
        return None

    if isinstance(result, tuple) and len(result) == 2:
        ret, target_joint = result
        if ret == 0:
            print("[OK] 逆解成功，关节角度：", [f"{j*180/np.pi:.2f}°" for j in target_joint])
            return target_joint
        else:
            print(f"[ERROR] 逆解失败，错误码：{ret}")
            return None

"""
:param robot: 机器人对象
:param x, y, z: 目标位置 (mm)
:param angle_rad: 绕Z轴旋转角度 (弧度)
:param joint_speed: 关节运动速度 (rad/s)，默认从配置读取
:raises Exception: 逆解失败或运动失败时抛出e
"""
 # 移动到击球点，使用关节运动（逆解后关节移动）
def move_to_hit_pose(robot, target_joint, joint_speed=config.JOINT_SPEED):
    ret = robot.joint_move(target_joint, config.ABS, config.BLOCK, joint_speed)  # 关节运动到目标角度s
    if ret[0] != 0:
        raise Exception(f"关节运动到击球点失败，错误码：{ret[0]}")
    else:
        print(f"已到达击球点")


 # 设置工具坐标系和用户坐标系 (实际设置在main中)
def set_tool_and_user_frame(robot, user_id, tool_id):
    ret = robot.set_user_frame_id(user_id)
    if ret[0] != 0:
        print("设置用户坐标系失败")
    ret = robot.set_tool_id(tool_id)
    if ret[0] != 0:
        print("设置工具坐标系失败")
    print(f"工具坐标系 ID={tool_id}，用户坐标系 ID={user_id} 已激活")

