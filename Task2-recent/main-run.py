"""
节卡机器人仿真绘图主程序
功能：连续绘制正方形、圆形及一条平行于球网的横线
"""

import config
import drawing_lib as dl
import time

def main():
    # 1. 连接机器人
    print("\n========== 连接机器人ing ==========")
    robot = dl.secure_boot(config.DEFAULT_ROBOT_IP)
        # 初始化工具和用户坐标系    
    dl.set_tool_and_user_frame(robot, user_id=config.COORD_BASE, tool_id=config.COORD_TOOL)
    time.sleep(2)   # 等待系统稳定
    print("\n========== 原点初始化ing ==========")
    dl.return_to_the_origin(robot)
    time.sleep(1)

    # 2-1. 分段绘制平行于球网的横线（假设球网法向为 X 轴，横线沿 Y 轴）
    print("\n========== 任务1-1：分段绘制横线 ==========")
    dl.draw_horizontal_line(robot, 
                            x_fixed=350, y_start=-405, y_end=-226,z_fixed=400,r_x=45,r_y=45,
                            start_pose=config.joint_pose_1)
    time.sleep(1)
    dl.draw_horizontal_line(robot, 
                            x_fixed=350, y_start=226, y_end=226,z_fixed=400,r_x=0,r_y=0,
                            start_pose=config.joint_pose_2)
    time.sleep(1)
    dl.draw_horizontal_line(robot, 
                            x_fixed=350, y_start=226, y_end=400,z_fixed=400,r_x=-45,r_y=45,
                            start_pose=config.joint_pose_3)
    time.sleep(1)
    dl.return_to_the_origin(robot)
    time.sleep(1)
    # 2-2. 整段绘制平行于球网的横线（假设球网法向为 X 轴，横线沿 Y 轴）
    print("\n========== 任务1-2：整段绘制横线 ==========")
    dl.draw_horizontal_line(robot, 
                            x_fixed=350, y_start=-405, y_end=400,z_fixed=400,r_x=-45,r_y=45,
                            start_pose=config.joint_pose_1)
    time.sleep(1)
    dl.return_to_the_origin(robot)
    time.sleep(1)

    # 3. 绘制正方形（中心位于 [400,50,350]，边长 80mm）
    print("\n========== 任务2：绘制正方形 ==========")
    dl.draw_square(robot, x=350, center_y=0, center_z=450, side_len=200)
    time.sleep(1)
    dl.return_to_the_origin(robot)
    time.sleep(1)

    # 4-1. 绘制圆形（圆心位于 [350,0,450]，半径100，4段圆弧）
    print("\n========== 任务3-1：分段绘制圆形 ==========")
    dl.draw_circle_arc_segment(robot, center_x=350, center_y=0, z=450, radius=100,segments=4)
    time.sleep(1)
    dl.return_to_the_origin(robot)
    time.sleep(1)

    # 4-2. 绘制圆形（圆心位于 [350,0,450]，半径100，4段圆弧）
    print("\n========== 任务3-2：整段绘制圆形 ==========")
    dl.draw_circle_arc_whole(robot, center_x=350, center_y=0, z=450, radius=100)
    time.sleep(1)
    dl.return_to_the_origin(robot)
    time.sleep(1)

    # 5. 断开连接
    print("\n========== 断连机器人ing ==========")
    dl.safe_shutdown(robot)

if __name__ == "__main__":
    main()