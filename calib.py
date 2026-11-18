# calib.py  -  픽셀 -> 작업판 mm -> (높이 보정) -> 로봇 좌표, 그리고 블록 각도
import os
import numpy as np
import config as C


def load_H():
    if not os.path.exists(C.H_PATH):
        raise FileNotFoundError("data/H.npy가 없습니다. tools/calib_homography.py에서 s를 눌러 저장하세요.")
    return np.load(C.H_PATH)


def pixel_to_mm(H, u, v):
    """픽셀 (u, v) -> 작업판 (x, y) mm"""
    p = H @ np.array([u, v, 1.0])
    return float(p[0] / p[2]), float(p[1] / p[2])


def height_fix(x, y, obj_h):
    """높이 obj_h인 점이 바깥으로 밀려 보이는 만큼 카메라 바로 아래 점 쪽으로 당김"""
    k = (C.CAM_H_MM - obj_h) / C.CAM_H_MM
    cx, cy = C.NADIR_MM
    return cx + (x - cx) * k, cy + (y - cy) * k


def table_to_robot(x, y):
    """작업판 좌표 -> 로봇 좌표 (x 정면, y 왼쪽)"""
    dx, dy = x - C.ROBOT_BASE[0], y - C.ROBOT_BASE[1]
    t = np.radians(C.ROBOT_THETA_DEG)
    return float(dx * np.cos(t) + dy * np.sin(t)), float(-dx * np.sin(t) + dy * np.cos(t))


def to_robot(H, u, v, obj_h=None):
    """픽셀 -> 로봇 좌표 mm. obj_h = 그 점의 바닥에서의 높이 (기본: 블록 윗면 25mm)"""
    x, y = pixel_to_mm(H, u, v)
    if C.USE_HEIGHT_FIX:
        x, y = height_fix(x, y, C.BLOCK_MM if obj_h is None else obj_h)
    return table_to_robot(x, y)


def block_angle(H, corners, obj_h=None):
    """블록 윗면 꼭짓점 4개(픽셀)를 로봇 좌표로 바꾼 뒤 변의 방향을 0~90도로 돌려줌
    화면에서 바로 재면 원근 때문에 틀어지므로 mm로 바꾼 다음에 잽니다."""
    pts = np.array([to_robot(H, u, v, obj_h) for u, v in corners])
    e = pts[1] - pts[0]
    return float(np.degrees(np.arctan2(e[1], e[0])) % 90.0)
