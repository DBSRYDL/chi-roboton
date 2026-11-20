# arm_place_calib.py  -  로봇이 블록을 직접 놓고, 카메라가 놓인 위치를 재서 (1) 실제 오차 확인 (2) 관절 매핑 다듬기
# 자동: 처음에 블록 하나만 바닥에 두면, 로봇이 카메라로 찾아 집고 -> 목표에 놓고 -> 사진 -> 다시 찾아 집고 ... 를 반복
#   python tools\arm_place_calib.py            왼쪽 20mm 제한, 자동
#   python tools\arm_place_calib.py 0          왼쪽(카메라 쪽)으로는 안 감
#   python tools\arm_place_calib.py 20 manual  예전 방식: 매번 사람이 블록을 집게에 끼움
# 준비: config.py의 관절 매핑(임시라도), data/H.npy, ROBOT_BASE, 카메라. 한 명은 전원 옆에.
import _path  # noqa
import sys
import time
import numpy as np
import config as C
from arm_calib import fit

TARGETS = [(150, 0), (200, 0), (130, 50), (180, 60), (130, -50), (180, -60), (230, 30), (230, -30), (160, -20), (210, -15)]
HOVER_Z = C.BLOCK_MM / 2 + 60.0      # 블록 중심 기준 60mm 위
RELEASE_Z = C.BLOCK_MM / 2 + 10.0    # 블록 아랫면이 바닥에서 10mm 뜬 상태에서 놓음
READY = (150.0, 0.0, 110.0)
MAX_PICK_TRY = 2


def dist(b, x, y):
    return float(np.hypot(b["x"] - x, b["y"] - y))


def main():
    args = [a for a in sys.argv[1:] if a != "manual"]
    manual = "manual" in sys.argv
    left = float(args[0]) if args else 20.0
    targets = [(x, y) for x, y in TARGETS if y <= left]
    print(f"목표 {len(targets)}곳: {targets}   {'수동(블록 끼우기)' if manual else '자동'}")
    print("임시 매핑:", C.ARM_SIGN, C.ARM_OFFSET_DEG, "  높이 보정:", C.USE_HEIGHT_FIX)
    print("준비: 팔은 카메라 화면 밖 쉬는 자세, " + ("" if manual else "블록 하나를 회전축에서 9~25cm 안 바닥에, ") + "한 명은 전원 옆에.")
    if input("시작하려면 yes: ").strip().lower() != "yes":
        return
    C.ARM_MAPPING_CHECKED = True                    # 이 실행에서만
    C.ARM_SPEED_DEG_S = min(C.ARM_SPEED_DEG_S, 15.0)

    from arm import Arm
    from vision import get_blocks, close
    from pick_place_test import pick
    arm = Arm().connect()
    motors, points, misses = [], [], []
    try:
        arm.grip_open()
        blocks = [] if manual else get_blocks()
        for i, (x, y) in enumerate(targets):
            print(f"\n[{i + 1}/{len(targets)}] 목표 ({x}, {y})")
            # 1) 블록 쥐기
            if manual:
                arm.grip_open()
                if input("  블록을 집게 사이에 끼우고 Enter (q: 끝내기) ").strip().lower() == "q":
                    break
                arm.grip_close()
            else:
                held = False
                for t in range(MAX_PICK_TRY):
                    if not blocks:
                        print("  블록을 찾지 못했습니다.")
                        break
                    b = blocks[0]
                    print(f"  집기 ({b['x']:.1f}, {b['y']:.1f}) 각도 {b['angle']:.0f}" + (f"  (다시 시도 {t})" if t else ""))
                    if not pick(arm, b):
                        print("  그 위치는 갈 수 없습니다.")
                        break
                    held = True                      # 집었다고 보고 진행 (놓은 뒤 카메라로 확인)
                    break
                if not held:
                    break
            # 2) 목표에 놓기
            if arm.move_to(*READY) is None or arm.move_to(x, y, HOVER_Z, yaw_deg=0) is None \
                    or arm.move_to(x, y, RELEASE_Z, yaw_deg=0, speed=8) is None:
                print("  이 목표는 갈 수 없습니다. 건너뜁니다.")
                arm.move_to(*READY)
                if not manual:                      # 들고 있는 블록은 원래 근처에 다시 내려놓음
                    arm.move_to(blocks[0]["x"], blocks[0]["y"], HOVER_Z, yaw_deg=0)
                    arm.move_to(blocks[0]["x"], blocks[0]["y"], RELEASE_Z, yaw_deg=0, speed=8)
                    arm.grip_open()
                    arm.move_to(*READY)
                arm.park()
                blocks = [] if manual else get_blocks()
                continue
            m = np.median([arm.read_motor() for _ in range(5)], axis=0)
            arm.grip_open()
            arm.move_to(x, y, HOVER_Z, yaw_deg=0, speed=8)
            arm.move_to(*READY)
            arm.park()
            # 3) 놓인 위치 재기
            found = get_blocks(obj_h=C.BLOCK_MM)
            if not found:
                print("  블록을 찾지 못했습니다. 끝냅니다.")
                break
            near = min(found, key=lambda c: dist(c, x, y))
            if dist(near, x, y) > 35:
                print(f"  목표 근처에 블록이 없습니다 (가장 가까운 것 {dist(near, x, y):.0f}mm). 집기 실패로 보고 이번 점은 버림")
                blocks = found
                continue
            dx, dy = near["x"] - x, near["y"] - y
            print(f"  놓인 위치 ({near['x']:.1f}, {near['y']:.1f})  ->  빗나감 x {dx:+.1f}, y {dy:+.1f}  (거리 {np.hypot(dx, dy):.1f}mm)")
            motors.append(m[:5])
            points.append((near["x"], near["y"], RELEASE_Z))
            misses.append((dx, dy))
            blocks = [near] + [c for c in found if c is not near]
    finally:
        close()
        try:
            arm.park()
            time.sleep(0.5)
        except Exception:
            pass
        arm.close()                                  # 쉬는 자세에서 토크를 끔

    if not misses:
        return
    mis = np.array(misses)
    d = np.hypot(mis[:, 0], mis[:, 1])
    print(f"\n[지금 매핑의 실제 정확도]  평균 빗나감 {d.mean():.1f}mm, 최대 {d.max():.1f}mm  ({len(d)}점)")
    print(f"  한쪽으로 쏠림: x {mis[:, 0].mean():+.1f}, y {mis[:, 1].mean():+.1f} mm")
    if len(motors) < 5:
        print("매핑을 다시 계산하려면 5점 이상 필요합니다.")
        return
    sign, off, rms, err = fit(motors, points, site=False)
    print("\n[이번 측정으로 다시 맞춘 매핑]")
    print("점별 오차(mm):", np.round(err, 1))
    print(f"평균 오차(RMS): {rms:.1f} mm")
    print(f"ARM_SIGN = {sign}")
    print(f"ARM_OFFSET_DEG = {off}")
    print("\n새 값을 넣고 한 번 더 돌려서 평균 빗나감이 줄어드는지 확인하세요.")


if __name__ == "__main__":
    main()
