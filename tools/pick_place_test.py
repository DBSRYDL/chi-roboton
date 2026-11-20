# pick_place_test.py  -  비전 확인용 반복 테스트: 블록을 찾아 집고 -> 다른 곳에 놓고 -> 다시 찾아서 집고 ...
# 놓을 곳은 로봇이 정하므로 "카메라가 찾은 위치"와 "실제로 놓은 위치"를 비교해 비전 정확도를 잽니다.
#   python tools\pick_place_test.py              왼쪽 20mm 제한, 5회, 자동으로 연속
#   python tools\pick_place_test.py 20 10        왼쪽 20mm 제한, 10회
#   python tools\pick_place_test.py 20 10 step   매 회 Enter를 눌러야 진행 (지켜보며 할 때)
#   python tools\pick_place_test.py 20 50 fit    50회 + 성공 10회마다 관절 매핑을 다시 맞춰서 더 좋아지면 바로 적용
# 회마다: 목표와 찾은 위치 차이, 누적 성공률, 최근 10회 평균, 쏠림 방향을 출력
# 결과: data\pick_test\log.csv (회마다 한 줄), data\pick_test\*.jpg (회마다 인식 화면)
# 시작 전: 팔을 카메라 화면 밖 쉬는 자세로, 블록 하나를 회전축에서 9~25cm 안에. 한 명은 전원 옆에.
import _path  # noqa
import os
import sys
import csv
import math
import time
import cv2
import numpy as np
import config as C
import camera
import vision
from arm import Arm

READY = (150.0, 0.0, 110.0)
HOVER_Z = C.BLOCK_MM / 2 + 60.0     # 블록 중심 기준 60mm 위
PICK_Z = C.BLOCK_MM / 2 + 1.0 + C.PICK_Z_EXTRA_MM   # 집을 때 집기 중심 높이 (블록 중심 + 1mm + 여유)
PLACE_Z = C.BLOCK_MM / 2 + 4.0 + C.PICK_Z_EXTRA_MM  # 놓을 때: 블록이 바닥에서 조금 뜬 상태에서 놓음
DROP_POINTS = [(150, -40), (200, -20), (220, 10), (140, 10), (170, -10), (190, -45), (230, -25)]   # 매핑을 맞춘 범위 안 (11/20: 오른쪽 끝 y<-50, 카메라 화면 가장자리는 뺌)
MIN_MOVE = 60.0                     # 지금 위치에서 이만큼 이상 떨어진 곳에 놓음
LOG_DIR = os.path.join(C.DATA_DIR, "pick_test")
REFIT_EVERY = 10                    # 성공 이만큼마다 관절 매핑을 다시 맞춰 봄 (좋아질 때만 바로 적용)
MAX_FAIL_STREAK = 3                 # 연속 실패가 이만큼이면 멈춤


def dist(a, x, y):
    return math.hypot(a["x"] - x, a["y"] - y)


def ang_err(a, b):
    d = (a - b) % 90.0
    return d - 90.0 if d > 45.0 else d


def snapshot(name):
    frames = camera.grab(vision._cap, 1)
    if not frames:
        return
    f = frames[0]
    for d in vision.frame_blocks(f, vision._H):
        cv2.polylines(f, [np.int32(d["corners"])], True, (0, 255, 0), 2)
        cv2.putText(f, f'{d["color"]} ({d["x"]:.0f},{d["y"]:.0f}) {d["angle"]:.0f}deg', (int(d["u"]) + 8, int(d["v"]) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    ok, buf = cv2.imencode(".jpg", f)
    if ok:
        buf.tofile(os.path.join(LOG_DIR, name))


def pick(arm, b):
    """블록 위 -> 내려가서 집기 -> 다시 위로. 실패하면 False (팔은 위로 빠져 있음)"""
    if arm.move_to(*READY) is None:
        return False
    jc = C.JAW_CLEAR_MM
    if arm.move_to(b["x"], b["y"], HOVER_Z, yaw_deg=b["angle"], jaw_clear=jc) is None:
        return False
    if arm.move_to(b["x"], b["y"], PICK_Z, yaw_deg=b["angle"], speed=10, jaw_clear=jc) is None:
        arm.move_to(*READY)
        return False
    print(f"  집는 높이에서 처짐(보정 전): {np.round(arm.droop, 1)} 도")
    arm.grip_close()
    arm.move_to(b["x"], b["y"], HOVER_Z, yaw_deg=b["angle"], speed=10)
    return True


def place(arm, x, y):
    """놓고 위로 빠짐. 반환: 놓는 순간의 모터 값 5개 (매핑 다듬기용). 갈 수 없으면 None"""
    if arm.move_to(x, y, HOVER_Z, yaw_deg=0) is None:
        return None
    arm.move_to(x, y, PLACE_Z, yaw_deg=0, speed=10)
    m = np.median([arm.read_motor() for _ in range(5)], axis=0)[:5]
    arm.grip_open()
    arm.move_to(x, y, HOVER_Z, yaw_deg=0, speed=10)
    arm.move_to(*READY)
    return m


def mapping_rms(motors, points, sign, off):
    from arm_calib import tip_of
    e = [np.linalg.norm(tip_of(m, sign, off, site=False) - np.array(p)) for m, p in zip(motors, points)]
    return float(np.sqrt(np.mean(np.square(e))))


def summary(rows):
    """rows: 성공한 회의 (목표 x, y, 찾은 x, y). 구역별 쏠림을 보여 줌"""
    a = np.array(rows, float)
    d = a[:, 2:4] - a[:, 0:2]
    e = np.hypot(d[:, 0], d[:, 1])
    print(f"  전체: 평균 {e.mean():.1f}mm, 최대 {e.max():.1f}mm, 쏠림 x {d[:, 0].mean():+.1f} y {d[:, 1].mean():+.1f}  ({len(e)}회)")
    r = np.hypot(a[:, 0], a[:, 1])
    for name, mask in [("가까운 곳(<170mm)", r < 170), ("먼 곳(>=170mm)", r >= 170),
                       ("왼쪽(y>-20)", a[:, 1] > -20), ("오른쪽(y<=-20)", a[:, 1] <= -20)]:
        if mask.sum():
            print(f"  {name:16s} 평균 {e[mask].mean():4.1f}mm  쏠림 x {d[mask, 0].mean():+5.1f} y {d[mask, 1].mean():+5.1f}  ({mask.sum()}회)")


def main():
    nums = [a for a in sys.argv[1:] if a not in ("auto", "step", "fit")]
    left = float(nums[0]) if len(nums) > 0 else 20.0
    cycles = int(nums[1]) if len(nums) > 1 else 5
    auto = "step" not in sys.argv
    refit = "fit" in sys.argv
    drops = [p for p in DROP_POINTS if p[1] <= left]
    os.makedirs(LOG_DIR, exist_ok=True)
    stamp = time.strftime("%m%d_%H%M%S")
    print(f"놓을 후보: {drops}   반복 {cycles}회   {'연속' if auto else '매 회 Enter'}")
    print("준비: 팔은 카메라 화면 밖 쉬는 자세, 블록 하나를 회전축에서 9~25cm 안에, 한 명은 전원 옆에.")
    if input("시작하려면 yes: ").strip().lower() != "yes":
        return
    C.ARM_SPEED_DEG_S = min(C.ARM_SPEED_DEG_S, 20.0)

    arm = Arm().connect()
    log_path = os.path.join(LOG_DIR, f"log_{stamp}.csv")
    logf = open(log_path, "w", newline="", encoding="utf-8-sig")
    w = csv.writer(logf)
    w.writerow(["cycle", "seen_x", "seen_y", "seen_angle", "conf", "target_x", "target_y",
                "found_x", "found_y", "found_angle", "place_err_mm", "angle_err_deg", "pick_ok",
                "m_pan", "m_lift", "m_elbow", "m_wflex", "m_roll", "offset_version"])
    errs, aerrs, oks, rows = [], [], [], []
    fit_m, fit_p = [], []
    version, streak = 0, 0
    try:
        arm.grip_open()
        blocks = vision.get_blocks()
        snapshot(f"{stamp}_00_start.jpg")
        for k in range(1, cycles + 1):
            if not blocks:
                print("블록을 찾지 못했습니다. 끝냅니다.")
                break
            b = blocks[0]
            cands = [p for p in drops if dist(b, *p) >= MIN_MOVE] or drops
            tx, ty = cands[(k - 1) % len(cands)]
            print(f"\n[{k}/{cycles}] 찾은 블록 ({b['x']:.1f}, {b['y']:.1f}) 각도 {b['angle']:.0f}  conf {b['conf']}  ->  놓을 곳 ({tx}, {ty})")
            if not auto and input("  Enter: 진행 / q: 끝내기 ").strip().lower() == "q":
                break
            if not pick(arm, b):
                print("  이 블록 위치는 갈 수 없습니다. 끝냅니다.")
                break
            m_rel = place(arm, tx, ty)
            arm.park()
            blocks = vision.get_blocks()
            snapshot(f"{stamp}_{k:02d}.jpg")

            left_behind = [c for c in blocks if dist(c, b["x"], b["y"]) < 20]
            near = min(blocks, key=lambda c: dist(c, tx, ty)) if blocks else None
            ok = near is not None and dist(near, tx, ty) < 30 and not left_behind
            if ok:
                e = dist(near, tx, ty)
                ae = ang_err(near["angle"], 0.0)
                errs.append(e); aerrs.append(abs(ae)); rows.append((tx, ty, near["x"], near["y"]))
                print(f"  놓은 뒤 찾은 위치 ({near['x']:.1f}, {near['y']:.1f})  ->  목표와 차이 {e:.1f}mm "
                      f"(x {near['x'] - tx:+.1f}, y {near['y'] - ty:+.1f}), 각도 {ae:+.1f}도")
                blocks = [near] + [c for c in blocks if c is not near]
                if m_rel is not None:
                    fit_m.append(m_rel); fit_p.append((near["x"], near["y"], PLACE_Z))
                streak = 0
            elif left_behind:
                print("  블록이 원래 자리에 남아 있습니다 -> 집기 실패 (찾은 위치가 틀렸거나 집게가 놓침)")
                streak += 1
            else:
                print("  놓을 곳 근처에서 블록을 못 찾았습니다 -> 떨어뜨렸거나 인식 실패. 사진을 확인하세요.")
                streak += 1
            oks.append(ok)
            recent = errs[-10:]
            print(f"  누적 성공 {sum(oks)}/{len(oks)}" + (f"  최근 {len(recent)}회 평균 {np.mean(recent):.1f}mm" if recent else ""))
            w.writerow([k, round(b["x"], 1), round(b["y"], 1), b["angle"], b["conf"], tx, ty,
                        *((near["x"], near["y"], near["angle"], round(dist(near, tx, ty), 1), round(ang_err(near["angle"], 0.0), 1))
                          if near else ("", "", "", "", "")), int(ok),
                        *(np.round(m_rel, 2) if m_rel is not None else [""] * 5), version])
            logf.flush()

            if refit and ok and len(fit_m) >= REFIT_EVERY and len(fit_m) % REFIT_EVERY == 0:
                from arm_calib import fit
                cur = mapping_rms(fit_m, fit_p, C.ARM_SIGN, C.ARM_OFFSET_DEG)
                sign, off, rms, _ = fit(fit_m, fit_p, site=False)
                print(f"  [매핑 다듬기] 지금 값 오차 {cur:.1f}mm -> 새로 맞춘 값 {rms:.1f}mm  {off}")
                jump = float(np.max(np.abs(np.array(off[:4]) - np.array(C.ARM_OFFSET_DEG[:4]))))
                if jump > 10:
                    print(f"  -> 관절 하나가 {jump:.0f}도나 바뀌는 값이라 믿을 수 없어서 버림 (점이 한쪽에 몰렸을 때 생김)")
                elif sign == list(C.ARM_SIGN) and rms < 0.85 * cur:
                    C.ARM_OFFSET_DEG = off
                    version += 1
                    print(f"  -> 좋아져서 바로 적용 (버전 {version}). 끝난 뒤 config.py에 넣을 값은 마지막에 출력")
                else:
                    print("  -> 크게 나아지지 않아 그대로 둠")
            temps = arm.read_temps()
            if temps is not None:
                hot = int(np.argmax(temps))
                names = ["어깨회전", "어깨", "팔꿈치", "손목", "손목회전", "그리퍼"]
                print(f"  모터 온도 최고 {temps[hot]:.0f}도 ({names[hot]})")
                if temps[hot] >= C.MOTOR_TEMP_STOP:
                    print(f"\n모터 온도가 {C.MOTOR_TEMP_STOP}도를 넘어 멈춥니다. 전원을 끄고 식힌 뒤 다시 하세요.")
                    break
                if temps[hot] >= C.MOTOR_TEMP_WARN:
                    print(f"  {C.MOTOR_TEMP_WARN}도 이상이라 쉬는 자세에서 식힙니다 (토크 끄고 최대 5분, {C.MOTOR_TEMP_WARN - 5}도까지)")
                    arm.relax()
                    t0 = time.time()
                    while time.time() - t0 < 300:
                        time.sleep(15)
                        tt = arm.read_temps()
                        if tt is None or tt.max() < C.MOTOR_TEMP_WARN - 5:
                            break
                    arm.hold()
            if streak >= MAX_FAIL_STREAK:
                print(f"\n연속 {streak}회 실패해서 멈춥니다. 사진({LOG_DIR})을 확인하세요.")
                break
    finally:
        logf.close()
        vision.close()
        try:
            arm.park()
        except Exception:
            pass
        if not auto:
            input("\n토크를 끕니다. 팔이 내려앉을 수 있으니 받치고 Enter")
        arm.close()

    if oks:
        print(f"\n[결과] 성공 {sum(oks)}/{len(oks)}")
        if rows:
            print("  놓은 위치 vs 카메라가 찾은 위치 (쏠림이 한쪽으로 크면 보정 문제, 들쭉날쭉하면 인식 문제)")
            summary(rows)
            print(f"  각도 차이: 평균 {np.mean(aerrs):.1f}도, 최대 {np.max(aerrs):.1f}도")
        if version:
            print("\n[실행 중에 다듬어진 관절 매핑]  config.py에 넣으세요")
            print(f"ARM_OFFSET_DEG = {C.ARM_OFFSET_DEG}")
        elif refit and len(fit_m) >= 5:
            from arm_calib import fit
            sign, off, rms, _ = fit(fit_m, fit_p, site=False)
            jump = float(np.max(np.abs(np.array(off[:4]) - np.array(C.ARM_OFFSET_DEG[:4]))))
            print(f"\n[참고] 이번 자료로 맞춘 매핑: 오차 {rms:.1f}mm  ARM_OFFSET_DEG = {off}  (지금 값 {mapping_rms(fit_m, fit_p, C.ARM_SIGN, C.ARM_OFFSET_DEG):.1f}mm)"
                  + ("  <- 관절이 10도 넘게 바뀌어 믿을 수 없음, 넣지 마세요" if jump > 10 else ""))
        print(f"  기록: {log_path}")
        print(f"  사진: {LOG_DIR}\\{stamp}_*.jpg")


if __name__ == "__main__":
    main()
