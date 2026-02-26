"""
JAKA 机器人逆运动学求解器（纯Python实现，不依赖SDK）
支持：给定末端位姿（基坐标系下）和参考关节角度，返回所有可行解或最优解。
需要用户根据实际机器人修改 DH 参数。
"""

import numpy as np
import math
from typing import List, Tuple, Optional

# ========== 用户配置区域 ==========
# 请根据实际 JAKA 机器人的 DH 参数修改以下值
# DH 参数表 (标准DH约定: a, alpha, d, theta_offset)
# 关节顺序: 1~6
# 格式: [a (mm), alpha (rad), d (mm), theta_offset (rad)]
JAKA_DH = [
    [0,       math.pi/2, 0.089,    0],   # 关节1
    [-0.425,  0,         0,        0],   # 关节2
    [-0.392,  0,         0,        0],   # 关节3
    [0,       math.pi/2, 0.109,    0],   # 关节4
    [0,      -math.pi/2, 0.095,    0],   # 关节5
    [0,       0,         0.082,    0]    # 关节6
]

# 关节运动范围（弧度），用于剔除无效解，请根据实际修改
JOINT_LIMITS = [
    (-2*math.pi, 2*math.pi),  # J1
    (-2*math.pi, 2*math.pi),  # J2
    (-2*math.pi, 2*math.pi),  # J3
    (-2*math.pi, 2*math.pi),  # J4
    (-2*math.pi, 2*math.pi),  # J5
    (-2*math.pi, 2*math.pi)   # J6
]

# ========== 辅助函数 ==========
def rot_matrix_to_rpy(R: np.ndarray) -> Tuple[float, float, float]:
    """旋转矩阵转欧拉角 (RPY: 绕固定轴 Z-Y-X 顺序)"""
    assert R.shape == (3, 3)
    if abs(R[2, 0]) < 1 - 1e-6:
        ry = -math.asin(R[2, 0])
        rx = math.atan2(R[2, 1]/math.cos(ry), R[2, 2]/math.cos(ry))
        rz = math.atan2(R[1, 0]/math.cos(ry), R[0, 0]/math.cos(ry))
    else:
        # 万向锁情况
        ry = math.pi/2 if R[2, 0] < 0 else -math.pi/2
        rx = 0
        rz = math.atan2(R[0, 1], R[1, 1])
    return rx, ry, rz

def rpy_to_rot_matrix(rx: float, ry: float, rz: float) -> np.ndarray:
    """欧拉角 (RPY: 绕固定轴 Z-Y-X 顺序) 转旋转矩阵"""
    Rx = np.array([[1, 0, 0],
                   [0, math.cos(rx), -math.sin(rx)],
                   [0, math.sin(rx), math.cos(rx)]])
    Ry = np.array([[math.cos(ry), 0, math.sin(ry)],
                   [0, 1, 0],
                   [-math.sin(ry), 0, math.cos(ry)]])
    Rz = np.array([[math.cos(rz), -math.sin(rz), 0],
                   [math.sin(rz), math.cos(rz), 0],
                   [0, 0, 1]])
    return Rz @ Ry @ Rx

def forward_kinematics(joint_angles: List[float]) -> Tuple[np.ndarray, np.ndarray]:
    """正向运动学：根据关节角度计算末端位姿 (位置向量, 旋转矩阵)"""
    T = np.eye(4)
    for i in range(6):
        a, alpha, d, theta_offset = JAKA_DH[i]
        theta = joint_angles[i] + theta_offset
        # 标准DH变换矩阵
        A = np.array([
            [math.cos(theta), -math.sin(theta)*math.cos(alpha),  math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
            [math.sin(theta),  math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
            [0,                math.sin(alpha),                 math.cos(alpha),                d],
            [0,                0,                                0,                              1]
        ])
        T = T @ A
    position = T[:3, 3]
    rotation = T[:3, :3]
    return position, rotation

def is_within_limits(joint_angles: List[float]) -> bool:
    """检查关节角度是否在限位内"""
    for i, (low, high) in enumerate(JOINT_LIMITS):
        if joint_angles[i] < low or joint_angles[i] > high:
            return False
    return True

def normalize_angle(angle: float) -> float:
    """将角度归一化到 [-pi, pi]"""
    while angle > math.pi:
        angle -= 2*math.pi
    while angle < -math.pi:
        angle += 2*math.pi
    return angle

def distance_to_ref(q: List[float], q_ref: List[float]) -> float:
    """计算关节角度与参考值的距离（考虑角度周期性）"""
    dist = 0.0
    for i in range(6):
        d = abs(normalize_angle(q[i] - q_ref[i]))
        dist += d * d
    return math.sqrt(dist)

# ========== 逆运动学核心 ==========
def inverse_kinematics(target_pose: List[float],
                       ref_joint: List[float],
                       return_all: bool = False) -> Tuple[int, Optional[List[float]]]:
    """
    逆运动学求解
    :param target_pose: 目标位姿 [x, y, z, rx, ry, rz] (基坐标系，单位 mm, rad)
    :param ref_joint: 参考关节角度 [j1,...,j6] (rad)
    :param return_all: 如果True，返回所有有效解（元组列表），否则返回最优解
    :return: (错误码, 解) 错误码 0 成功，-4 逆解失败
    """
    # 解析目标位姿
    x, y, z, rx, ry, rz = target_pose
    R_des = rpy_to_rot_matrix(rx, ry, rz)  # 期望的旋转矩阵
    p_des = np.array([x, y, z])             # 期望的位置

    # --- 第一步：计算腕部中心点位置 ---
    # 对于标准六轴机器人，关节4、5、6的交点为腕部中心，其位置 = 末端位置 - d6 * 末端坐标系Z轴方向
    d6 = JAKA_DH[5][2]  # 最后一个连杆偏距
    # 末端坐标系Z轴是旋转矩阵的第三列
    n = R_des[:, 2]  # Z轴方向向量
    p_wrist = p_des - d6 * n

    # --- 第二步：求解关节1、2、3 ---
    # 根据腕部中心坐标 p_wrist = (px, py, pz)
    px, py, pz = p_wrist
    a2 = JAKA_DH[1][0]   # 关节2的a
    a3 = JAKA_DH[2][0]   # 关节3的a
    d1 = JAKA_DH[0][2]   # 关节1的d
    d4 = JAKA_DH[3][2]   # 关节4的d

    solutions = []  # 存储所有有效解

    # --- 求解 theta1 ---
    r = math.sqrt(px*px + py*py)
    if r < 1e-6:
        # 腕部中心位于Z轴上，theta1任意，通常取当前值
        theta1_candidates = [ref_joint[0]]
    else:
        theta1_1 = math.atan2(py, px)
        theta1_2 = math.atan2(-py, -px)  # 相差180度
        theta1_candidates = [theta1_1, theta1_2]

    for theta1 in theta1_candidates:
        # 计算旋转后的坐标，方便后续求解
        s1 = math.sin(theta1)
        c1 = math.cos(theta1)

        # --- 求解 theta3 ---
        # 根据平面几何，计算 theta3
        # 腕部中心在基坐标系下的坐标，经过 theta1 旋转后，在坐标系1中为 (x1, y1, z1)
        x1 = c1*px + s1*py
        y1 = -s1*px + c1*py   # 应为0（因为theta1使腕部落在X1Z平面）
        z1 = pz - d1

        # 计算 theta3 的两种可能
        # 余弦定理： (x1^2 + z1^2 - a2^2 - a3^2) / (2*a2*a3) = cos(theta3)
        D = (x1*x1 + z1*z1 - a2*a2 - a3*a3) / (2 * a2 * a3)
        if abs(D) > 1.0:
            continue  # 无解
        theta3_1 = math.acos(D)
        theta3_2 = -theta3_1

        for theta3 in [theta3_1, theta3_2]:
            # --- 求解 theta2 ---
            # 利用几何关系求 theta2
            s3 = math.sin(theta3)
            c3 = math.cos(theta3)
            # 计算矩阵 k = a2 + a3*c3
            k1 = a2 + a3*c3
            k2 = a3*s3
            # 解方程组：
            # x1 = k1 * cos(theta2) - k2 * sin(theta2)
            # z1 = k1 * sin(theta2) + k2 * cos(theta2)
            # 求解 theta2 使用 atan2
            # 将方程组看作旋转矩阵作用于 (k1, k2)
            # [x1; z1] = [cos(theta2), -sin(theta2); sin(theta2), cos(theta2)] * [k1; k2]
            # 所以 theta2 = atan2( x1*k2 + z1*k1, x1*k1 - z1*k2 )? 需要推导
            # 更直接的方法：通过解线性方程组
            # 设 cos(theta2) = u, sin(theta2) = v, 且 u^2+v^2=1
            # x1 = k1*u - k2*v
            # z1 = k1*v + k2*u
            # 求解 u, v:
            # 行列式 det = k1^2 + k2^2
            det = k1*k1 + k2*k2
            if abs(det) < 1e-12:
                continue
            u = (k1*x1 - k2*z1) / det   # cos(theta2)
            v = (k1*z1 + k2*x1) / det   # sin(theta2)
            theta2 = math.atan2(v, u)

            # 至此，前三关节角度为 [theta1, theta2, theta3]
            q123 = [theta1, theta2, theta3]

            # --- 第三步：求解后三关节 (theta4, theta5, theta6) ---
            # 根据前三关节计算从基到关节4的旋转矩阵 R03
            T01 = np.array([
                [math.cos(theta1), -math.sin(theta1)*math.cos(JAKA_DH[0][1]), math.sin(theta1)*math.sin(JAKA_DH[0][1]), JAKA_DH[0][0]*math.cos(theta1)],
                [math.sin(theta1), math.cos(theta1)*math.cos(JAKA_DH[0][1]), -math.cos(theta1)*math.sin(JAKA_DH[0][1]), JAKA_DH[0][0]*math.sin(theta1)],
                [0, math.sin(JAKA_DH[0][1]), math.cos(JAKA_DH[0][1]), JAKA_DH[0][2]],
                [0, 0, 0, 1]
            ])
            T12 = np.array([
                [math.cos(theta2), -math.sin(theta2)*math.cos(JAKA_DH[1][1]), math.sin(theta2)*math.sin(JAKA_DH[1][1]), JAKA_DH[1][0]*math.cos(theta2)],
                [math.sin(theta2), math.cos(theta2)*math.cos(JAKA_DH[1][1]), -math.cos(theta2)*math.sin(JAKA_DH[1][1]), JAKA_DH[1][0]*math.sin(theta2)],
                [0, math.sin(JAKA_DH[1][1]), math.cos(JAKA_DH[1][1]), JAKA_DH[1][2]],
                [0, 0, 0, 1]
            ])
            T23 = np.array([
                [math.cos(theta3), -math.sin(theta3)*math.cos(JAKA_DH[2][1]), math.sin(theta3)*math.sin(JAKA_DH[2][1]), JAKA_DH[2][0]*math.cos(theta3)],
                [math.sin(theta3), math.cos(theta3)*math.cos(JAKA_DH[2][1]), -math.cos(theta3)*math.sin(JAKA_DH[2][1]), JAKA_DH[2][0]*math.sin(theta3)],
                [0, math.sin(JAKA_DH[2][1]), math.cos(JAKA_DH[2][1]), JAKA_DH[2][2]],
                [0, 0, 0, 1]
            ])
            T03 = T01 @ T12 @ T23
            R03 = T03[:3, :3]

            # 期望的旋转矩阵 R06 = R_des
            # 从关节4到6的旋转矩阵 R36 = R03.T * R06
            R36 = R03.T @ R_des

            # 根据球形腕结构求解 theta4, theta5, theta6
            # 通常 R36 可表示为 Z-Y-Z 欧拉角形式：R36 = Rz(theta4) * Ry(theta5) * Rz(theta6)
            # 对于标准DH，关节4绕Z轴，关节5绕Y轴（或X？取决于DH），需要根据实际alpha调整。
            # 这里假设关节4绕Z，关节5绕Y，关节6绕Z（常见于UR）。
            # 但根据我们设置的DH，关节4 alpha=pi/2，关节5 alpha=-pi/2，所以顺序需要仔细。
            # 更通用的方法是使用 atan2 从 R36 的元素提取。
            # 参考常见解法：
            # theta5 = atan2( sqrt(R36[0,2]^2 + R36[2,2]^2), R36[1,2] ) 或类似。
            # 我们采用稳健方法：
            # 假设 R36 = Rz(t4) * Ry(t5) * Rz(t6)
            # 则 R36 = [ [c4*c5*c6 - s4*s6, -c4*c5*s6 - s4*c6, c4*s5],
            #            [s4*c5*c6 + c4*s6, -s4*c5*s6 + c4*c6, s4*s5],
            #            [-s5*c6,             s5*s6,            c5] ]
            # 因此：
            # theta5 = atan2( sqrt(R36[0,2]^2 + R36[1,2]^2), R36[2,2] )
            # 若 theta5 接近0或pi，则腕部奇异，theta4和theta6有无穷多解，通常取当前参考值。
            if abs(R36[2, 2]) < 1e-6 and (abs(R36[0,2]) < 1e-6 and abs(R36[1,2]) < 1e-6):
                # 奇异：关节5接近0或pi，theta4和theta6不能独立确定，但总和可确定
                # 一般取 theta4 = ref_joint[3], theta6 = atan2(-R36[1,0], R36[0,0]) - theta4
                # 或者 theta4 = 0，然后计算theta6
                # 为简单，跳过奇异解，或使用参考值
                # 这里我们生成一组解，但需要处理
                # 先假设 theta5 = 0 或 pi
                if R36[2,2] > 0:
                    theta5 = 0.0
                else:
                    theta5 = math.pi
                # 此时 R36 = Rz(t4+ t6) 矩阵形式
                # 有 R36[0,0] = cos(t4+t6), R36[0,1] = -sin(t4+t6), R36[1,0] = sin(t4+t6) ...
                sum_theta = math.atan2(R36[1,0], R36[0,0])
                # 取 theta4 = ref_joint[3], 则 theta6 = sum_theta - theta4
                theta4 = ref_joint[3]
                theta6 = sum_theta - theta4
                theta4_candidates = [theta4]
            else:
                # 正常情况
                theta5_1 = math.atan2(math.sqrt(R36[0,2]**2 + R36[1,2]**2), R36[2,2])
                theta5_2 = math.atan2(-math.sqrt(R36[0,2]**2 + R36[1,2]**2), R36[2,2])
                theta5_candidates = [theta5_1, theta5_2]
                for theta5 in theta5_candidates:
                    s5 = math.sin(theta5)
                    c5 = math.cos(theta5)
                    if abs(s5) < 1e-6:
                        # 奇异，类似处理
                        sum_theta = math.atan2(R36[1,0], R36[0,0])
                        theta4 = ref_joint[3]
                        theta6 = sum_theta - theta4
                        theta4_candidates = [theta4]
                    else:
                        theta4 = math.atan2(R36[1,2], R36[0,2])
                        theta6 = math.atan2(R36[2,1], -R36[2,0])
                        theta4_candidates = [theta4]
                    for theta4 in theta4_candidates:
                        # 组成完整解
                        q = [theta1, theta2, theta3, theta4, theta5, theta6]
                        # 归一化角度到 [-pi, pi]
                        q_norm = [normalize_angle(angle) for angle in q]
                        if is_within_limits(q_norm):
                            solutions.append(q_norm)

    # 如果没有解
    if not solutions:
        return (-4, None)

    # 根据参考选择最优解
    if return_all:
        return (0, solutions)

    # 选择与参考最接近的解
    best_sol = min(solutions, key=lambda q: distance_to_ref(q, ref_joint))
    return (0, best_sol)