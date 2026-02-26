"""
节卡机器人 SDK 绘图示例 - 绘图功能库
依赖: jkrc, config
"""

import jkrc
import time
import math
import config

 # 安全连接→登录→上电→上使能，每一步均检查状态并打印结果
def secure_boot(ip=config.DEFAULT_ROBOT_IP):
    robot = jkrc.RC(ip)
    for act, name in [(robot.login, "登录"),
                      (robot.power_on, "电源打开"),
                      (robot.enable_robot, "上使能")]:
        ret = act()
        if ret[0] == 0:
            print(f"*{name}成功")
        else:
            raise Exception(f"*{name}失败，错误码：{ret[0]}")
    print(f"--机器人 [{ip}] 已连接并上电使能--")
    return robot

def set_tool_and_user_frame(robot, user_id, tool_id):
    ret = robot.set_user_frame_id(user_id)
    if ret[0] != 0:
        print("设置用户坐标系失败")
    ret = robot.set_tool_id(tool_id)
    if ret[0] != 0:
        print("设置工具坐标系失败")
    print(f"工具坐标系 ID={tool_id}，用户坐标系 ID={user_id} 已激活")

 # 安全关机：下使能→断电→登出，每一步均检查状态并打印结果
def safe_shutdown(robot):
    for act, name in [(robot.disable_robot, "下使能"),
                      (robot.power_off, "断电"),
                      (robot.logout, "登出")]:
        ret = act()
        if ret[0] == 0:
            print(f"*{name}成功")
        else:
            raise Exception(f"*{name}失败，错误码：{ret[0]}")
    print("*机器人已登出")

 # 关节运动至原点
def return_to_the_origin(robot,joint_speed=config.DEFAULT_LINEAR_SPEED):
    origin_point = [0,0,0,0,0,0]
    ret = robot.joint_move(origin_point,config.ABS, config.BLOCK, joint_speed)
    if ret[0] != 0:
        raise Exception("--原点回归失败，错误码：{ret[0]}--")
    else:
        print("--原点回归就位--")

 # 绘制直线
def draw_horizontal_line(robot, x_fixed, y_start, y_end, z_fixed,r_x,r_y,start_pose,
                         linear_speed=config.DEFAULT_LINEAR_SPEED,
                         joint_speed=config.DEFAULT_LINEAR_SPEED,):
    """
    绘制一段水平横线（沿 Y 轴）
    """
     # 关节运动至初始位置，并检查状态
    ret = robot.joint_move(start_pose, config.ABS, config.BLOCK, joint_speed)
    if ret[0] != 0:
        raise Exception("*移动到该段横线起点失败")
    else:
        print("*成功移动到该段横线起点")
    
     # 直线运动，并检查状态
    end_pose = [x_fixed, y_end, z_fixed, r_x, r_y, 0]
    ret = robot.linear_move(end_pose, config.ABS, config.BLOCK, linear_speed)
    if ret[0] != 0:
        print("*该段横线绘制失败")
    else:
        print(f"--该段横线绘制过程完成： X={x_fixed} , Z={z_fixed} ; 从 Y={y_start} 到 Y={y_end}--")

 # 绘制正方形
def draw_square(robot, x, center_y, center_z, side_len,
                linear_speed=config.DEFAULT_LINEAR_SPEED,
                joint_speed=config.DEFAULT_LINEAR_SPEED, 
                mode=config.ABS):
    """
    绘制正方形（绝对运动阻塞模式）
    参数含义同单文件版本
    """
    half = side_len / 2.0

     # 导入初始关节位置
    start_pose =[-16.890*math.pi/180,
                 119.508*math.pi/180,
                 -60.103*math.pi/180,
                   0.000*math.pi/180,
                 -59.406*math.pi/180,
                  16.890*math.pi/180,]
    
     # 正方形四点数据
    points = [
        [x,center_y + half, center_z - half,  0, 0, 0],
        [x,center_y + half, center_z + half,  0, 0, 0],
        [x,center_y - half, center_z + half,  0, 0, 0],
        [x,center_y - half, center_z - half,  0, 0, 0],
    ]

     # 关节运动至初始位置
    ret = robot.joint_move(start_pose,config.ABS, config.BLOCK, joint_speed)
    if ret[0] != 0:
        raise Exception("--正方形绘图初始化失败--")
    else:
        print("--正方形绘图初始化成功--")
        
     # 状态检查
    for i, p in enumerate(points):
        ret = robot.linear_move(p, mode, config.BLOCK, linear_speed)
        if ret[0] != 0:
            print(f"*正方形第{i+1}段运动失败，错误码：{ret[0]}")
        else:
            print(f"*正方形顶点 {i+1} :{p} 到位")
    print("--正方形绘制过程完成--")

def draw_circle_arc_origin(robot,joint_speed=config.DEFAULT_ARC_SPEED):
    # 移动到起点（角度 0）：起点位于 (center_x, center_y+radius, z)
    start_angle = 0.0
    start_pos = [ 15.001*math.pi/180,
                  99.974*math.pi/180,
                 -51.689*math.pi/180,
                   0.000*math.pi/180,
                 -48.285*math.pi/180,
                 -15.001*math.pi/180,]
    ret = robot.joint_move(start_pos, config.ABS, config.BLOCK, joint_speed)
    if ret[0] != 0:
        raise Exception(f"*移动到圆形起点失败,错误码: {ret[0]}")
    print("*已移动到圆形起点")
    return start_angle

 # 使用末端圆弧指令绘制圆形（平行于 YZ 平面）
def draw_circle_arc_segment(robot, center_x, center_y, z, radius,segments,
                            joint_speed=config.DEFAULT_ARC_SPEED, 
                            arc_speed=config.DEFAULT_ARC_SPEED,
                            arc_acc=config.DEFAULT_ARC_ACC):
    print(f"--开始分{segments}段绘制圆形--")
    start_angle=draw_circle_arc_origin(robot,joint_speed)
    # 分段绘制圆弧
    angle_step = 2 * math.pi / segments
    for i in range(1, segments + 1):
        end_angle = i * angle_step
        mid_angle = start_angle + angle_step / 2.0

        mid_pos = [center_x,
                   center_y + radius * math.cos(mid_angle),
                   z + radius * math.sin(mid_angle),
                   0, 0, 0]
        end_pos = [center_x,
                   center_y + radius * math.cos(end_angle),
                   z + radius * math.sin(end_angle),
                   0, 0, 0]

        ret = robot.circular_move(end_pos, mid_pos,
                                  config.ABS, config.BLOCK,
                                  arc_speed, arc_acc, tol=0.1)
        if ret[0] != 0:
            raise Exception(f"*圆弧段 {i} 绘制失败,错误码: {ret[0]}")
        else:
            print(f"*圆弧段 {i} 绘制成功")
        start_angle = end_angle
    print("*圆形绘制过程完成")





'''以下为废案，暂时不想改，又弃之可惜'''
'''
 # 使用扩展圆弧指令绘制圆形（平行于 YZ 平面）
def draw_circle_arc_whole(robot, center_x, center_y, z, radius,
                         joint_speed=config.DEFAULT_ARC_SPEED, 
                         arc_speed=config.DEFAULT_ARC_SPEED,
                         arc_acc=config.DEFAULT_ARC_ACC):
    print(f"--开始一次性绘制整个圆形--")
    start_angle=draw_circle_arc_origin(robot,joint_speed)
    angle_step = 2 * math.pi / 4

    # 整段段绘制圆弧
    end_angle = 4 * angle_step
    mid_angle = start_angle + angle_step / 2.0

    mid_pos = [center_x,
               center_y + radius * math.cos(mid_angle),
               z + radius * math.sin(mid_angle),
               0, 0, 0]
    end_pos = [center_x,
               center_y + radius * math.cos(end_angle),
               z + radius * math.sin(end_angle),
               0, 0, 0]

    ret = robot.circular_move_extend(end_pos, mid_pos,
                                     config.ABS, config.BLOCK,
                                     arc_speed, arc_acc, tol=0.1,circle_cnt=1)
    if ret[0] != 0:
        raise Exception(f"*整段圆弧绘制失败,错误码: {ret[0]}")
    else:
        print(f"*整段圆弧绘制成功")
    start_angle = end_angle
    print("-圆形绘制过程完成-")
'''