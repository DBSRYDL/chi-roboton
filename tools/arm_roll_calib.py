# arm_roll_calib.py  -  손목 회전(wrist_roll)의 방향·영점 맞추기 (지금까지 한 번도 재지 않은 값)
# 증상: 집게가 블록 각도와 비뚤어지게 내려와서 고정 집게가 블록 윗면을 때림, 놓은 블록 각도가 들쭉날쭉
# 방법: 사람이 블록을 집게에 반듯하게 끼움 -> 로봇이 0/30/60도로 돌려서 놓음 -> 카메라가 놓인 각도를 잼
#       -> "명령한 각도"와 "실제 놓인 각도"의 관계로 wrist_roll의 부호와 영점을 계산
#   python tools\arm_roll_calib.py
import _path  # noqa
import math
import numpy as np
import config as C
import so101_ik as K

SAMPLES = [(160, -20, 0), (160, -20, 30), (200, -30, 60), (200, -30, 0), (180, 10, 30), (180, 10, 60), (170, -50, 15), (190, -10, 45)]
HOVER_Z = C.BLOCK_MM / 2 + 60.0
RELEASE_Z = C.BLOCK_MM / 2 + 8.0
READY = (150.0, 0.0, 110.0)


def wrap45(d):
    return (d + 45.0) % 90.0 - 45.0


def jaw_angle(m, sign, off):
    q = np.radians(np.array(sign, float) * (np.asarray(m, float) - np.array(off, float)))
    a = K.fk(q)[:3, K.JAW_AXIS_COL]
    return math.degrees(math.atan2(a[1], a[0])) % 90.0


def fit_roll(motors, angles):
    """wrist_roll 부호 s와 영점 o를 격자 탐색. 나머지 관절 매핑은 config 그대로"""
    best = None
    for s in (1, -1):
        for o in np.arange(-45.0, 45.01, 0.25):
            sign = list(C.ARM_SIGN[:4]) + [s]
            off = list(C.ARM_OFFSET_DEG[:4]) + [float(o)]
            e = np.array([wrap45(jaw_angle(m, sign, off) - a) for m, a in zip(motors, angles)])
            rms = float(np.sqrt(np.mean(e ** 2)))
            if best is None or rms < best[0]:
                best = (rms, s, round(float(o), 2), e)
    return best


def main():
    print("손목 회전 맞추기: 블록을 매번 손으로 집게에 반듯하게 끼웁니다 (블록 면이 두 집게 면에 딱 붙게).")
    if input("한 명이 전원 옆에 있으면 yes: ").strip().lower() != "yes":
        return
    C.ARM_SPEED_DEG_S = min(C.ARM_SPEED_DEG_S, 15.0)
    from arm import Arm
    from vision import get_blocks, close
    arm = Arm().connect()
    motors, angles = [], []
    try:
        for i, (x, y, yaw) in enumerate(SAMPLES):
            print(f"\n[{i + 1}/{len(SAMPLES)}] ({x}, {y}) 에 {yaw}도로 놓기")
            arm.grip_open()
            if input("  블록을 집게에 반듯하게 끼우고 Enter (q: 끝내기) ").strip().lower() == "q":
                break
            arm.grip_close()
            if arm.move_to(*READY) is None or arm.move_to(x, y, HOVER_Z, yaw_deg=yaw) is None \
                    or arm.move_to(x, y, RELEASE_Z, yaw_deg=yaw, speed=8) is None:
                print("  갈 수 없습니다. 건너뜁니다.")
                arm.move_to(*READY); arm.park()
                continue
            m = np.median([arm.read_motor() for _ in range(5)], axis=0)[:5]
            arm.grip_open()
            arm.move_to(x, y, HOVER_Z, yaw_deg=yaw, speed=8)
            arm.move_to(*READY)
            arm.park()
            found = get_blocks()
            if not found:
                print("  블록을 못 찾았습니다. (이번 점은 버림)")
                continue
            b = min(found, key=lambda c: math.hypot(c["x"] - x, c["y"] - y))
            now = jaw_angle(m, C.ARM_SIGN, C.ARM_OFFSET_DEG)
            print(f"  명령 {yaw}도 -> 놓인 각도 {b['angle']:.1f}도  (지금 매핑이 생각한 집게 각도 {now:.1f}도, 차이 {wrap45(b['angle'] - now):+.1f}도)")
            motors.append(m); angles.append(b["angle"])
    finally:
        close()
        try:
            arm.park()
        except Exception:
            pass
        arm.close()
    if len(motors) < 4:
        print("4점 이상 필요합니다.")
        return
    cur = np.array([wrap45(jaw_angle(m, C.ARM_SIGN, C.ARM_OFFSET_DEG) - a) for m, a in zip(motors, angles)])
    rms, s, o, e = fit_roll(motors, angles)
    print(f"\n지금 값의 각도 오차: 평균 {np.mean(np.abs(cur)):.1f}도, 최대 {np.max(np.abs(cur)):.1f}도")
    print(f"맞춘 값의 각도 오차: 평균 {np.mean(np.abs(e)):.1f}도, 최대 {np.max(np.abs(e)):.1f}도")
    sign = list(C.ARM_SIGN[:4]) + [s]
    off = list(C.ARM_OFFSET_DEG[:4]) + [o]
    print("\nconfig.py에 붙여 넣기:")
    print(f"ARM_SIGN = {sign}")
    print(f"ARM_OFFSET_DEG = {off}")


if __name__ == "__main__":
    main()
