"""
节卡机器人仿真绘图主程序（交互式菜单版本）
功能：通过终端菜单选择连续绘制正方形、圆形或平行于球网的横线
"""

import config
import drawing_lib as dl
import time

def print_menu():
    """打印功能菜单"""
    print("\n" + "="*40)
    print("          请选择要执行的任务")
    print("="*40)
    print(" 1. 绘制平行于球网的横线")
    print(" 2. 绘制正方形")
    print(" 3. 绘制圆形")
    print(" 0. 退出程序")
    print("="*40)

def main():
    # 1. 连接机器人
    print("\n========== 连接机器人ing ==========")
    robot = dl.secure_boot(config.DEFAULT_ROBOT_IP)
    time.sleep(2)   # 等待系统稳定
    print("\n========== 原点初始化ing ==========")
    dl.return_to_the_origin(robot)
    time.sleep(1)

    # 2. 交互菜单循环
    while True:
        print_menu()
        choice = input("请输入数字选择任务: ").strip()

        if choice == '1':
            print("\n========== 任务1：绘制横线 ==========")
            dl.draw_horizontal_line(robot, x_fixed=350, y_start=-226.6, y_end=226.6, z_fixed=400)
            time.sleep(1)
            dl.return_to_the_origin(robot)
            time.sleep(1)

        elif choice == '2':
            print("\n========== 任务2：绘制正方形 ==========")
            dl.draw_square(robot, x=350, center_y=0, center_z=450, side_len=200)
            time.sleep(1)
            dl.return_to_the_origin(robot)
            time.sleep(1)

        elif choice == '3':
            dl.draw_circle_arc_segment(robot, center_x=350, center_y=0, z=450, radius=100, segments=4)
            time.sleep(1)
            dl.return_to_the_origin(robot)
            time.sleep(1)

        elif choice == '0':
            print("\n========== 退出程序 ==========")
            break

        else:
            print("输入无效，请重新选择 (0-3)")

    # 3. 断开连接
    print("\n========== 断连机器人ing ==========")
    dl.safe_shutdown(robot)

if __name__ == "__main__":
    main()





'''drawing_lib的伴生废案'''
'''
elif choice == '3':
            print("\n========== 任务3：绘制圆形 ==========")
            print("            请选择绘制方法")
            print(" 1. 分段绘制")
            print(" 2. 整段绘制")
            print("="*40)
            option=input("请输入数字选择方法: ")
            if option == '1':
                dl.draw_circle_arc_segment(robot, center_x=350, center_y=0, z=450, radius=100, segments=4)
                time.sleep(1)
                dl.return_to_the_origin(robot)
                time.sleep(1)
            else:
                dl.draw_circle_arc_whole(robot, center_x=350, center_y=0, z=450, radius=100)
                time.sleep(1)
                dl.return_to_the_origin(robot)
                time.sleep(1)
'''