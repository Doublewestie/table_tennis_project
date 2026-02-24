"""
乒乓球机器人拦截主程序
功能：读取模拟数据，分割轨迹，通过三轮精确定位预测落点，控制机器人移动到击球位置并模拟挥拍
支持仿真模式（无真实电机和串口)
"""

import time
import numpy as np
import config
import robot_control as rc
import motor_control as mc
import trajectory_analysis as ta

def main():
    # 1. 连接机器人
    print("\n========== 连接机器人 ==========")
    robot = rc.secure_boot(config.ROBOT_IP)
    time.sleep(2)
    rc.return_to_origin(robot)
    rc.set_tool_and_user_frame(robot, user_id=config.COORD_BASE, tool_id=config.COORD_TOOL)
    time.sleep(1)

    # 2. 初始化电机（仿真模式根据配置决定是否真实连接）
    print("\n========== 初始化末端电机 ==========")
    motor = mc.LingKongMotor(
        port=config.MOTOR_PORT,
        baudrate=config.MOTOR_BAUDRATE,
        motor_id=config.MOTOR_ID,
        simulation=config.SIMULATION_MODE
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

    # 三轮精确定位的阶段定义
    stages = [
        ("初步调整", config.STAGE1_RANGE),
        ("第二轮调整", config.STAGE2_RANGE),
        ("第三轮最终", config.STAGE3_RANGE)
    ]

    for idx, traj in enumerate(trajectories):
        print(f"\n--- 轨迹 {idx+1} (点数: {len(traj)}) ---")

        # 三轮精确定位预测
        y_pred = None
        angle_pred = None
        for stage_name, x_range in stages:
            print(f"  执行{stage_name}预测 (x范围 {x_range})...")
            y_stage, angle_stage = ta.fit_and_predict(traj, config.TARGET_X, x_range=x_range)
            if y_stage is not None:
                y_pred, angle_pred = y_stage, angle_stage
                print(f"    预测 Y = {y_pred:.2f} mm, 角度 = {np.degrees(angle_pred):.2f}°")
            else:
                print(f"    {stage_name}无有效数据，跳过")

        if y_pred is None:
            print("  所有阶段预测均失败，跳过本次拦截")
            continue

        # 获取实际最近点（仅用于显示误差）
        actual = ta.find_actual_point(traj, config.TARGET_X)
        if actual:
            actual_x, actual_y = actual
            error = abs(y_pred - actual_y)
            print(f"  目标 X = {config.TARGET_X} mm")
            print(f"  最终预测 Y = {y_pred:.2f} mm")
            print(f"  实际 Y = {actual_y:.2f} mm")
            print(f"  误差 = {error:.2f} mm")
            if error <= config.RACKET_RADIUS:
                print("  ✅ 理论可拦截")
            else:
                print("  ❌ 理论不可拦截")

        # 构造目标位姿（绕X轴旋转90°，绕Z轴旋转预测角度）
        target_pose = [config.TARGET_X, y_pred, config.HIT_Z,
                       90 * np.pi / 180, 0, angle_pred]

        # 逆解得到关节角度
        target_joint = rc.compute_inverse_kinematics(robot, target_pose)
        if target_joint is None:
            print("  逆解失败，跳过本次拦截")
            continue

        # 移动机器人到击球点
        print("  移动机器人至击球点...")
        rc.move_to_hit_pose(robot, target_joint)

        # 模拟挥拍动作（电机控制）
        print("  挥拍...")
        motor.set_single_turn_angle(30, direction=0, max_speed_dps=360)
        time.sleep(0.1)
        motor.set_single_turn_angle(0, direction=1, max_speed_dps=360)
        time.sleep(0.1)

        # 回归运动零点准备下一次
        rc.return_to_set_point(robot)
        time.sleep(1)

    # 5. 清理
    print("\n========== 任务完成，断开连接 ==========")
    motor.disable()
    motor.close()
    rc.safe_shutdown(robot)

if __name__ == '__main__':
    main()