"""
轨迹分析与预测模块（增强版）
支持指定数据范围拟合，并实现线性与二次回归的加权组合。
"""

import re
import numpy as np
from typing import List, Tuple, Optional
import config

def load_points(filename: str) -> List[Tuple[float, float]]:
    """从文件加载所有坐标点"""
    with open(filename, 'r', encoding='utf-8') as f:
        content = f.read()
    matches = re.findall(r'\(([^,]+),([^)]+)\)', content)
    points = [(float(x), float(y)) for x, y in matches]
    return points

def split_trajectories(points: List[Tuple[float, float]],
                       threshold: float = config.SPLIT_THRESHOLD) -> List[List[Tuple[float, float]]]:
    """根据 X 突变分割轨迹"""
    if not points:
        return []
    trajectories = []
    current = [points[0]]
    for i in range(1, len(points)):
        prev_x = points[i-1][0]
        curr_x = points[i][0]
        if abs(curr_x - prev_x) > threshold:
            trajectories.append(current)
            current = [points[i]]
        else:
            current.append(points[i])
    trajectories.append(current)
    return trajectories

def fit_and_predict(traj: List[Tuple[float, float]], target_x: float,
                    x_range: Optional[Tuple[float, float]] = None):
    """
    对轨迹进行拟合，返回预测的 y 值和切线角度 (弧度)
    使用线性与二次回归的加权组合（权重 config.WEIGHT_QUAD）

    :param traj: 轨迹点列表
    :param target_x: 目标 X 坐标
    :param x_range: 可选，指定拟合数据范围 (x_min, x_max)，若为 None 则使用全部数据
    :return: (y_pred, angle_pred) 或 (None, None)
    """
    # 按 x 排序
    sorted_traj = sorted(traj, key=lambda p: p[0])
    xs = np.array([p[0] for p in sorted_traj])
    ys = np.array([p[1] for p in sorted_traj])

    # 数据筛选
    if x_range is not None:
        x_min, x_max = x_range
        mask = (xs >= x_min) & (xs <= x_max)
        if np.sum(mask) >= 3:
            xs_fit = xs[mask]
            ys_fit = ys[mask]
        else:
            print(f"  警告: 范围 {x_range} 内点数不足 (实际 {np.sum(mask)} 个)，无法拟合")
            return None, None
    else:
        xs_fit = xs
        ys_fit = ys

    if len(xs_fit) < 2:
        return None, None

    try:
        # 线性回归
        coeffs_lin = np.polyfit(xs_fit, ys_fit, 1)
        a_lin, b_lin = coeffs_lin
        y_lin = a_lin * target_x + b_lin
        slope_lin = a_lin

        # 二次回归
        coeffs_quad = np.polyfit(xs_fit, ys_fit, 2)
        a_quad, b_quad, c_quad = coeffs_quad
        y_quad = a_quad * target_x**2 + b_quad * target_x + c_quad
        slope_quad = 2 * a_quad * target_x + b_quad

        # 加权组合
        w = config.WEIGHT_QUAD
        y_pred = w * y_quad + (1 - w) * y_lin
        slope = w * slope_quad + (1 - w) * slope_lin

        # 计算速度方向与 X 轴夹角 (弧度)
        # 速度向量近似为 ( -1, slope )，因为 x 递减
        angle_pred = np.arctan2(slope, -1)

        return y_pred, angle_pred
    except Exception as e:
        print(f"  拟合失败: {e}")
        return None, None

def find_actual_point(traj: List[Tuple[float, float]], target_x: float) -> Optional[Tuple[float, float]]:
    """在轨迹中找到 X 最接近 target_x 的实际点"""
    if not traj:
        return None
    return min(traj, key=lambda p: abs(p[0] - target_x))