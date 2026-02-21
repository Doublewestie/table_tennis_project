#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
乒乓球机器人拦截主程序
功能：读取模拟数据，分割轨迹，预测落点，控制机器人移动到击球位置并模拟挥拍
支持仿真模式（无需真实电机和串口）
"""

import time
import config
import robot_control as rc
import motor_control as mc
import trajectory_analysis as ta
import numpy as np  # 仅用于角度打印，已安装

def main():
    # 1. 连接机器人
    print("\n========== 连接机器人 ==========")
    robot = rc.secure_boot(config.ROBOT_IP)
    time.sleep(2)
    rc.return_to_origin(robot)
    # 设置工具和用户坐标系（需提前标定好，ID 与示教器一致）
    rc.set_tool_and_user_frame(robot, tool_id=2, user_id=4)  # 根据实际情况修改
    time.sleep(1)

    # 2. 初始化电机（仿真模式根据配置决定是否真实连接）
    print("\n========== 初始化末端电机 ==========")
    motor = mc.LingKongMotor(
        port=config.MOTOR_PORT,
        baudrate=config.MOTOR_BAUDRATE,
        motor_id=config.MOTOR_ID,
        simulation=config.SIMULATION_MODE   # 从配置读取仿真模式
    )
    motor.enable()
    time.sleep(1)

    # 3. 读取并分析轨迹
    print("\n========== 加载模拟数据 ==========")
    points = ta.load_points('模拟数据.txt')
    print(f"总点数: {len(points)}")

    trajectories = ta.split_trajectories(points)
    print(f"分割出 {len(trajectories)} 条轨迹")

    # 4. 对每条轨迹进行拦截
    print("\n========== 开始拦截 ==========")
    for idx, traj in enumerate(trajectories):
        print(f"\n--- 轨迹 {idx+1} (点数: {len(traj)}) ---")

        # 预测落点和速度方向角
        y_pred, angle = ta.fit_and_predict(traj, config.TARGET_X)
        if y_pred is None:
            print("  预测失败，跳过")
            continue

        # 获取实际最近点（仅用于显示误差）
        actual = ta.find_actual_point(traj, config.TARGET_X)
        if actual:
            actual_x, actual_y = actual
            error = abs(y_pred - actual_y)
            print(f"  目标 X = {config.TARGET_X} mm")
            print(f"  预测 Y = {y_pred:.2f} mm")
            print(f"  实际 Y = {actual_y:.2f} mm")
            print(f"  误差 = {error:.2f} mm")
            print(f"  速度方向角 = {np.degrees(angle):.2f}°")
            if error <= config.RACKET_RADIUS:
                print("  ✅ 理论可拦截")
            else:
                print("  ❌ 理论不可拦截")

        # 构造目标位姿并计算机器人逆解（可选，仅用于验证）
        target_pose = [config.TARGET_X, y_pred, config.HIT_Z, 0, 0, angle]
        joint_angles = rc.compute_inverse_kinematics(robot, target_pose)
        if joint_angles is None:
            print("  逆解失败，跳过本次拦截")
            continue

        # 移动机器人到击球点（关节运动）
        print("  移动机器人至击球点...")
        rc.move_to_hit_pose(robot, config.TARGET_X, y_pred, config.HIT_Z, angle)

        # 模拟挥拍动作（电机控制）
        print("  挥拍...")
        motor.set_single_turn_angle(30, direction=0, max_speed_dps=360)
        time.sleep(0.1)
        motor.set_single_turn_angle(0, direction=1, max_speed_dps=360)
        time.sleep(0.1)

        # 回归原点准备下一次
        rc.return_to_origin(robot)
        time.sleep(1)

    # 5. 清理
    print("\n========== 任务完成，断开连接 ==========")
    motor.disable()
    motor.close()
    rc.safe_shutdown(robot)

if __name__ == '__main__':
    main()