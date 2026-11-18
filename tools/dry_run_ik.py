# dry_run_ik.py  -  로봇을 움직이지 않고 "비전 좌표 -> IK 관절각"까지만 확인
# 블록마다 (1) 블록 위 대기점 (2) 집는 점의 관절각을 계산해서 출력
#   python tools\dry_run_ik.py          카메라로 실제 블록 사용
#   python tools\dry_run_ik.py fake     카메라 없이 fake_vision 사용
import _path  # noqa
import sys
import math
import numpy as np
import config as C
import so101_ik as K

HOVER_MM = 50.0                 # 집기 전에 블록 중심보다 이만큼 위에서 멈춤
PICK_Z = C.BLOCK_MM / 2         # 바닥에 놓인 블록의 중심 높이 (TCP 목표)

if len(sys.argv) > 1 and sys.argv[1] == "fake":
    from fake_vision import get_blocks, close
else:
    from vision import get_blocks, close


def solve(x, y, z, angle_deg, q_prev=None):
    q, tilt = K.ik_auto(x, y, z, yaw=math.radians(angle_deg), q_prev=q_prev)
    return q, tilt


blocks = get_blocks()
close()
print(f"블록 {len(blocks)}개")
np.set_printoptions(precision=1, suppress=True)
for i, b in enumerate(blocks):
    r = math.hypot(b["x"], b["y"])
    print(f'\n[{i}] {b["color"]}  x={b["x"]:.1f} y={b["y"]:.1f} (거리 {r:.0f}mm)  angle={b["angle"]:.1f}  conf={b["conf"]}')
    q_up, t_up = solve(b["x"], b["y"], PICK_Z + HOVER_MM, b["angle"])
    if q_up is None:
        print("    위 대기점: 도달 불가")
        continue
    q_dn, t_dn = solve(b["x"], b["y"], PICK_Z, b["angle"], q_prev=q_up)
    print(f"    위 대기점 z={PICK_Z + HOVER_MM:.1f}: {np.degrees(q_up)} deg" + (f"  (기울임 {t_up}도)" if t_up else ""))
    if q_dn is None:
        print("    집는 점: 도달 불가")
        continue
    print(f"    집는 점   z={PICK_Z:.1f}: {np.degrees(q_dn)} deg" + (f"  (기울임 {t_dn}도)" if t_dn else ""))
    err = K.tool_pos(q_dn) - np.array([b["x"], b["y"], PICK_Z])
    print(f"    FK로 되돌린 위치 오차: {np.linalg.norm(err):.3f} mm")
print("\n관절 순서:", K.JOINT_NAMES, "(모델 기준 각도. 실물 모터 값으로 바꾸는 건 모션 쪽 calib)")
