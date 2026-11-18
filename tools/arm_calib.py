# arm_calib.py  -  관절 영점·방향 매핑 구하기 (팔은 스스로 움직이지 않음)
# 토크를 끈 채, 고정 집게 손끝을 작업판의 정해진 점에 차례로 갖다 대고 Enter.
# 모인 (모터 각도, 실제 손끝 위치)로 ARM_SIGN, ARM_OFFSET_DEG 를 맞춰서 출력합니다.
# - 점마다 팔 자세를 다르게 하세요 (팔꿈치를 높게/낮게, 그리퍼를 세워서/눕혀서). 자세가 비슷하면 값이 안 정해집니다.
# - 작업판과 로봇이 config.py의 ROBOT_BASE 를 잴 때와 같은 자리에 있어야 합니다.
import _path  # noqa
import itertools
import numpy as np
import config as C
import calib
import so101_ik as K

# 손끝을 댈 작업판 점 (작업판 mm). 굵은 격자선의 교차점이라 찾기 쉬움
BOARD_POINTS = [(200, 50), (200, 100), (150, 50), (150, 100), (100, 50), (100, 100), (175, 75), (125, 75)]


def tip_of(motor_deg, sign, offset):
    q = np.radians(np.array(sign) * (np.asarray(motor_deg, float) - np.array(offset)))
    return K.fk(q, site=True)[:3, 3]


def fit(motors, points, prior=0.02):
    """motors: N x 5 (도), points: N x 3 (mm).  반환: (sign 5개, offset 5개, rms mm)
    손끝은 wrist_roll 축 위에 있어서 roll의 부호·영점은 여기서 못 구함 (1, 0 으로 둠)"""
    motors, points = np.asarray(motors, float), np.asarray(points, float)
    best = None
    for s4 in itertools.product([1, -1], repeat=4):
        sign = list(s4) + [1]
        off = np.zeros(4)

        def res(o):
            full = list(o) + [0.0]
            r = np.concatenate([tip_of(m, sign, full) - p for m, p in zip(motors, points)])
            return np.concatenate([r, prior * o])           # 영점은 0 근처일 거라는 약한 가정

        lam = 1.0
        f = res(off)
        for _ in range(80):
            J = np.zeros((len(f), 4))
            for i in range(4):
                d = np.zeros(4); d[i] = 1e-4
                J[:, i] = (res(off + d) - f) / 1e-4
            step = np.linalg.solve(J.T @ J + lam * np.eye(4), -J.T @ f)
            f2 = res(off + step)
            if f2 @ f2 < f @ f:
                off, f, lam = off + step, f2, max(lam / 3, 1e-6)
                if np.linalg.norm(step) < 1e-6:
                    break
            else:
                lam *= 5
        err = np.array([np.linalg.norm(tip_of(m, sign, list(off) + [0.0]) - p) for m, p in zip(motors, points)])
        rms = float(np.sqrt(np.mean(err ** 2)))
        if best is None or rms < best[2]:
            best = (sign, [round(float(v), 2) for v in off] + [0.0], rms, err)
    return best


def main():
    from arm import Arm
    pts = [(*calib.table_to_robot(bx, by), 0.0) for bx, by in BOARD_POINTS]
    input("토크를 끄면 팔이 내려앉습니다. 팔을 손으로 받친 뒤 Enter...")
    arm = Arm().connect(torque=False)
    motors, used = [], []
    try:
        for (bx, by), p in zip(BOARD_POINTS, pts):
            a = input(f"작업판 ({bx}, {by}) 점에 고정 집게 손끝을 대고 Enter  (건너뛰기: s, 그만: q) ").strip().lower()
            if a == "q":
                break
            if a == "s":
                continue
            m = np.median([arm.read_motor() for _ in range(5)], axis=0)
            motors.append(m[:5]); used.append(p)
            print("   모터:", np.round(m[:5], 1), " 로봇 좌표:", np.round(p, 1))
    finally:
        arm.close()
    if len(motors) < 5:
        print("점이 5개 이상 필요합니다.")
        return
    sign, off, rms, err = fit(motors, used)
    print("\n점별 오차(mm):", np.round(err, 1))
    print(f"평균 오차(RMS): {rms:.1f} mm")
    print("\nconfig.py에 붙여 넣기:")
    print(f"ARM_SIGN = {sign}")
    print(f"ARM_OFFSET_DEG = {off}")
    if rms > 6:
        print("\n오차가 큽니다. 붙여 넣지 말고: 손끝을 정확히 댔는지, 점마다 자세를 다르게 했는지, ROBOT_BASE 측정이 맞는지 확인 후 다시 하세요.")
    else:
        print("\n오차가 작으면 값을 넣고 arm_read.py로 손끝 좌표가 맞는지 다시 본 뒤 ARM_MAPPING_CHECKED = True 로 바꾸세요.")
        print("wrist_roll(5번째)의 부호는 여기서 못 구합니다. arm_hover.py에서 집게 방향을 보고 확인합니다.")


if __name__ == "__main__":
    main()
