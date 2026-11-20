# calib_height.py  -  높이 보정 값(카메라 높이 CAM_H_MM, 카메라 바로 아래 점 NADIR_MM) 구하기  (비전 전용, 로봇 안 씀)
# 카메라가 비스듬히 위에서 보므로, 높이 25mm인 블록 윗면은 실제 위치보다 카메라 반대쪽으로 밀려 보입니다.
# 블록을 작업판 격자의 정해진 칸에 정확히 맞춰 놓고, 카메라가 잰 위치와 비교해 밀림을 계산합니다.
#   python tools\calib_height.py          카메라 높이도 계산
#   python tools\calib_height.py 400      카메라 렌즈 높이를 자로 잰 값(mm)으로 고정하고 바로 아래 점만 계산 (더 안정적)
# 잰 값은 data\height_points.csv 에 저장됩니다
# 칸 놓는 법: "왼쪽 아래 모서리 (30, 30)" 이면, 블록 아랫면의 두 변을 x=30, y=30 격자선에 맞춤 -> 블록이 (30~55, 30~55) 칸을 덮음
import _path  # noqa
import os
import sys
import csv
import numpy as np
import config as C
import camera
import calib
from detector import detect

CORNERS = [(30, 30), (100, 25), (170, 30), (30, 90), (100, 95), (170, 90), (65, 60), (135, 60)]   # 작업판 mm
N_FRAMES = 15
H_TOP = C.BLOCK_MM                       # 블록 윗면 높이


def measure(cap, H):
    pts = []
    for f in camera.grab(cap, N_FRAMES):
        found = detect(f)
        if len(found) == 1:
            pts.append(calib.pixel_to_mm(H, found[0]["u"], found[0]["v"]))   # 높이 보정 없이 작업판 mm
    return np.median(pts, axis=0) if len(pts) >= N_FRAMES // 2 else None


def fit_fixed(meas, true, cam_h):
    """카메라 높이를 알 때: g = H / (H - h) 고정, meas = n + g * (true - n) 에서 n 만 구함"""
    g = cam_h / (cam_h - H_TOP)
    meas, true = np.array(meas, float), np.array(true, float)
    n = ((meas - g * true) / (1 - g)).mean(axis=0)
    pred = n + g * (true - n)
    return (n[0], n[1]), cam_h, g, np.linalg.norm(pred - meas, axis=1)


def fit(meas, true):
    """meas = a + g * true  ->  카메라 바로 아래 점 n = a / (1 - g),  카메라 높이 H = h * g / (g - 1)"""
    A, b = [], []
    for m, p in zip(meas, true):
        A += [[1, 0, p[0]], [0, 1, p[1]]]
        b += [m[0], m[1]]
    (ax, ay, g), *_ = np.linalg.lstsq(np.array(A, float), np.array(b, float), rcond=None)
    pred = np.array([[ax + g * p[0], ay + g * p[1]] for p in true])
    res = np.linalg.norm(pred - np.array(meas), axis=1)
    return (ax / (1 - g), ay / (1 - g)), H_TOP * g / (g - 1), g, res


def main():
    H = calib.load_H()
    cap = camera.open_camera()
    meas, true = [], []
    print("화면에 블록 하나만 두세요. 손과 로봇팔은 화면 밖으로.")
    try:
        for cx, cy in CORNERS:
            a = input(f"\n블록 아랫면 왼쪽 아래 모서리를 ({cx}, {cy}) 에 맞춰 놓고 Enter  (건너뛰기 s, 그만 q) ").strip().lower()
            if a == "q":
                break
            if a == "s":
                continue
            m = measure(cap, H)
            if m is None:
                print("  블록을 안정적으로 못 찾았습니다. 다시 놓고 시도하세요. (이번 칸은 버림)")
                continue
            p = (cx + C.BLOCK_MM / 2, cy + C.BLOCK_MM / 2)
            meas.append(m); true.append(p)
            print(f"  실제 중심 ({p[0]:.1f}, {p[1]:.1f})  카메라 ({m[0]:.1f}, {m[1]:.1f})  밀림 ({m[0]-p[0]:+.1f}, {m[1]-p[1]:+.1f}) mm")
    finally:
        cap.release()
    if len(meas) < 4:
        print("4칸 이상 필요합니다.")
        return
    with open(os.path.join(C.DATA_DIR, "height_points.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["true_x", "true_y", "meas_x", "meas_y"])
        for p, m in zip(true, meas):
            w.writerow([round(p[0], 2), round(p[1], 2), round(float(m[0]), 2), round(float(m[1]), 2)])
    raw = np.linalg.norm(np.array(meas) - np.array(true), axis=1)
    known = float(sys.argv[1]) if len(sys.argv) > 1 else None
    nadir, cam_h, g, res = fit_fixed(meas, true, known) if known else fit(meas, true)
    print(f"\n보정 전 오차: 평균 {raw.mean():.1f}mm, 최대 {raw.max():.1f}mm")
    print(f"보정 후 남는 오차: 평균 {res.mean():.1f}mm, 최대 {res.max():.1f}mm")
    if not known and (not (1.0 < g < 1.3) or not (150 < cam_h < 2000)):
        print(f"계산이 이상합니다 (g={g:.3f}, 높이 {cam_h:.0f}mm). 블록을 칸에 정확히 맞췄는지 확인하고 다시 하세요.")
        return
    print("\nconfig.py에 붙여 넣기:")
    print("USE_HEIGHT_FIX = True")
    print(f"CAM_H_MM = {cam_h:.1f}")
    print(f"NADIR_MM = ({nadir[0]:.1f}, {nadir[1]:.1f})")


if __name__ == "__main__":
    main()
