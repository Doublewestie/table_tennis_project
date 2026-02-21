"""
轨迹分析与预测模块
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
                    method: str = config.FIT_METHOD,
                    x_limit: Optional[float] = config.FIT_X_LIMIT):
    """
    对轨迹进行拟合，返回预测的 y 值和切线角度 (弧度)
    """
    # 按 x 排序
    sorted_traj = sorted(traj, key=lambda p: p[0])
    xs = np.array([p[0] for p in sorted_traj])
    ys = np.array([p[1] for p in sorted_traj])

    # 数据筛选
    if x_limit is not None:
        mask = xs < x_limit
        if np.sum(mask) >= 3:
            xs_fit = xs[mask]
            ys_fit = ys[mask]
        else:
            xs_fit = xs
            ys_fit = ys
    else:
        xs_fit = xs
        ys_fit = ys

    if len(xs_fit) < 2:
        return None, None

    try:
        if method == 'linear':
            coeffs = np.polyfit(xs_fit, ys_fit, 1)
            a, b = coeffs
            y_pred = a * target_x + b
            slope = a
        elif method == 'quadratic':
            coeffs = np.polyfit(xs_fit, ys_fit, 2)
            a, b, c = coeffs
            y_pred = a * target_x**2 + b * target_x + c
            slope = 2 * a * target_x + b
        elif method == 'cubic':
            coeffs = np.polyfit(xs_fit, ys_fit, 3)
            a, b, c, d = coeffs
            y_pred = a * target_x**3 + b * target_x**2 + c * target_x + d
            slope = 3 * a * target_x**2 + 2 * b * target_x + c
        else:
            raise ValueError(f"未知拟合方法: {method}")

        # 计算速度方向与 X 轴夹角 (弧度)
        # 速度向量近似为 ( -1, slope )，因为 x 递减
        angle = np.arctan2(slope, -1)   # 实际方向角
        return y_pred, angle
    except Exception as e:
        print(f"拟合失败: {e}")
        return None, None

def find_actual_point(traj: List[Tuple[float, float]], target_x: float) -> Optional[Tuple[float, float]]:
    """在轨迹中找到 X 最接近 target_x 的实际点"""
    if not traj:
        return None
    return min(traj, key=lambda p: abs(p[0] - target_x))