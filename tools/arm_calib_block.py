# arm_calib_block.py  -  관절 영점·방향 매핑을 "블록 집기"로 구하기 (팔은 스스로 움직이지 않음)
# 격자 좌표를 눈으로 읽는 대신 카메라가 블록 위치를 재므로, 비전 좌표와 팔이 직접 맞춰집니다.
# 한 번에 한 점:
#   1) 블록 하나를 작업판 위 아무 데나 놓는다 (점마다 다른 곳: 가깝게/멀게/좌우)
#   2) 팔을 손으로 움직여 그리퍼로 블록을 "블록이 움직이지 않게" 위에서 집는다 -> Enter (모터 값 기록)
#   3) 그리퍼를 살며시 벌리고 팔을 카메라 화면 밖으로 치운다 -> Enter (카메라가 블록 위치를 잼)
# 6~8번 반복 후 q.  필요: data/H.npy, ROBOT_BASE 측정, 카메라 연결
import _path  # noqa
import numpy as np
import config as C
from arm_calib import fit

N_MAX = 10


def main():
    from arm import Arm
    from vision import get_blocks, close
    input("토크를 끄면 팔이 내려앉습니다. 팔을 손으로 받친 뒤 Enter...")
    arm = Arm().connect(torque=False)
    motors, points = [], []
    try:
        while len(motors) < N_MAX:
            print(f"\n[{len(motors) + 1}번째]  블록을 새 위치에 놓고, 팔로 블록을 위에서 집으세요.")
            a = input("  집었으면 Enter  (끝내기: q) ").strip().lower()
            if a == "q":
                break
            m = np.median([arm.read_motor() for _ in range(5)], axis=0)
            input("  그리퍼를 살며시 벌리고 팔을 카메라 화면 밖으로 치운 뒤 Enter")
            blocks = get_blocks(obj_h=C.BLOCK_MM)
            if len(blocks) != 1:
                print(f"  블록이 {len(blocks)}개 보입니다. 화면에 블록 하나만 두고 다시 하세요. (이번 점은 버림)")
                continue
            b = blocks[0]
            p = (b["x"], b["y"], C.BLOCK_MM / 2)      # 블록 중심 (바닥 블록)
            motors.append(m[:5]); points.append(p)
            print(f"  모터 {np.round(m[:5], 1)}  ->  블록 중심 로봇 좌표 ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f})")
    finally:
        close()
        arm.close()
    if len(motors) < 5:
        print("점이 5개 이상 필요합니다.")
        return
    sign, off, rms, err = fit(motors, points, site=False)
    print("\n점별 오차(mm):", np.round(err, 1))
    print(f"평균 오차(RMS): {rms:.1f} mm")
    print("\nconfig.py에 붙여 넣기:")
    print(f"ARM_SIGN = {sign}")
    print(f"ARM_OFFSET_DEG = {off}")
    if rms > 6:
        print("\n오차가 큽니다. 점별 오차가 유난히 큰 점이 있으면 그 위치에서 다시 해 보세요.")


if __name__ == "__main__":
    main()
