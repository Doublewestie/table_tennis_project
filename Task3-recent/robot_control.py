"""
机器人基础控制模块
依赖: jkrc, config
"""

import jkrc
import time
import math
import config

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

 # 回归关节零点
def return_to_origin(robot, joint_speed=config.JOINT_SPEED):
    origin = [0, 0, 0, 0, 0, 0]
    ret = robot.joint_move(origin, config.ABS, config.BLOCK, joint_speed)
    if ret[0] != 0:
        raise Exception(f"--原点回归失败，错误码：{ret[0]}--")
    else:
        print("--原点回归就位--")

def compute_inverse_kinematics(robot, target_pose, ref_joint=None):
    """
    计算目标位姿在当前工具和用户坐标系下的逆解
    :param robot: 已登录且上电使能的机器人对象
    :param target_pose: 目标笛卡尔位姿 [x, y, z, rx, ry, rz] (单位：mm, rad)
    :param ref_joint: 参考关节角度（可选，默认使用当前关节角度）
    :return: 成功返回关节角度列表（6个元素，单位 rad），失败返回 None
    """
    if ref_joint is None:
        ret, ref_joint = robot.get_joint_position()
        if ret != 0:
            print("[ERROR] 获取当前关节角度失败，错误码：", ret)
            return None

    # 调用逆解，处理不同返回值格式
    try:
        result = robot.kine_inverse(ref_joint, target_pose)
    except Exception as e:
        print(f"[ERROR] 逆解调用异常: {e}")
        return None

    # 情况1：返回 (errcode, joint_pos) 元组
    if isinstance(result, tuple) and len(result) == 2:
        ret, joint_pos = result
        if ret == 0:
            print("[OK] 逆解成功，关节角度：", [round(j, 4) for j in joint_pos])
            return joint_pos
        else:
            print(f"[ERROR] 逆解失败，错误码：{ret}")
            return None

    # 情况2：直接返回关节角度列表（6个值）
    elif isinstance(result, (list, tuple)) and len(result) == 6:
        print("[OK] 逆解成功，关节角度：", [round(j, 4) for j in result])
        return list(result)

    # 未知格式
    else:
        print(f"[ERROR] 逆解返回未知格式: {result}")
        return None

def move_to_hit_pose(robot, x, y, z, angle_rad, joint_speed=config.JOINT_SPEED):
    """
    移动到击球点，使用关节运动（逆解后关节移动）
    :param robot: 机器人对象
    :param x, y, z: 目标位置 (mm)
    :param angle_rad: 绕Z轴旋转角度 (弧度)
    :param joint_speed: 关节运动速度 (rad/s)，默认从配置读取
    :raises Exception: 逆解失败或运动失败时抛出e
    """
    # 构造目标位姿（绕Z轴旋转，其他轴为0）
    target_pose = [x, y, z, 0, 0, angle_rad]
    
    # 逆解得到关节角度
    joint_angles = compute_inverse_kinematics(robot, target_pose)
    if joint_angles is None:
        raise Exception(f"逆解失败，无法到达击球点 ({x:.1f}, {y:.1f}, {z:.1f})")
    
    # 关节运动到目标角度
    ret = robot.joint_move(joint_angles, config.ABS, config.BLOCK, joint_speed)
    if ret[0] != 0:
        raise Exception(f"关节运动到击球点失败，错误码：{ret[0]}")
    else:
        print(f"已到达击球点，关节角度: {[round(j, 4) for j in joint_angles]}")

def set_tool_and_user_frame(robot, tool_id=1, user_id=1):
    """
    设置工具坐标系和用户坐标系（示例）
    需提前标定好，此处仅调用 API 切换
    """
    ret = robot.set_tool_id(tool_id)
    if ret[0] != 0:
        print("设置工具坐标系失败")
    ret = robot.set_user_frame_id(user_id)
    if ret[0] != 0:
        print("设置用户坐标系失败")
    print(f"工具坐标系 ID={tool_id}，用户坐标系 ID={user_id} 已激活")

