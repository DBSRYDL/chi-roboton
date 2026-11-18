# arm_hover.py  -  첫 통합 테스트: 블록을 찾아 그 "위"까지만 가서 멈춤 (집지 않음, 내려가지 않음)
# 단계마다 Enter를 눌러야 다음으로 넘어갑니다. 이상하면 Ctrl+C 또는 전원을 뽑으세요.
# 시작 전: 팔을 카메라 화면 밖의 쉬는 자세로 두세요. 끝나면 그 자세로 돌아옵니다.
#   python tools\arm_hover.py          카메라로 블록 찾기
#   python tools\arm_hover.py 150 0    카메라 없이 로봇 좌표 (150, 0) 위로
import _path  # noqa
import sys
import math
import numpy as np
import config as C
from arm import Arm

HOVER_Z = C.BLOCK_MM / 2 + 60.0     # 블록 중심보다 60mm 위 (집게 끝이 블록 윗면에서 약 45mm 위)
READY = (150.0, 0.0, 110.0)         # 먼저 올라가 있을 높은 자세 (책상을 쓸지 않게)

if len(sys.argv) >= 3:
    target = {"x": float(sys.argv[1]), "y": float(sys.argv[2]), "angle": 0.0, "color": "manual"}
else:
    from vision import get_blocks, close
    blocks = get_blocks()
    close()
    if not blocks:
        print("블록을 찾지 못했습니다.")
        raise SystemExit
    target = blocks[0]
print(f'목표: {target["color"]}  x={target["x"]:.1f} y={target["y"]:.1f} angle={target["angle"]:.1f}')
print(f'회전축에서 {math.hypot(target["x"], target["y"]):.0f}mm, 정면 기준 {"왼쪽" if target["y"] > 0 else "오른쪽"} {abs(target["y"]):.0f}mm')

input("\n[1] 로봇에 연결합니다. 토크가 켜져 지금 자세로 굳습니다. Enter...")
arm = Arm().connect()
np.set_printoptions(precision=1, suppress=True)
try:
    print("    현재 TCP:", arm.tcp())
    input(f"[2] 높은 준비 자세 {READY} 로 천천히 올라갑니다. 경로에 걸리는 것이 없는지 확인하고 Enter...")
    if arm.move_to(*READY) is None:
        raise RuntimeError("준비 자세를 풀 수 없습니다.")
    print("    현재 TCP:", arm.tcp())
    input(f"[3] 블록 위 z={HOVER_Z:.0f}mm 로 갑니다. Enter...")
    tilt = arm.move_to(target["x"], target["y"], HOVER_Z, yaw_deg=target["angle"])
    if tilt is None:
        print("    이 위치는 갈 수 없습니다 (도달 불가).")
    else:
        print("    도착. 계산상 TCP:", arm.tcp(), f"(기울임 {tilt}도)")
        print("    확인할 것: 집게 가운데가 블록 중심 바로 위인지, 집게가 벌어지는 방향이 블록 변과 나란한지")
        print("    빗나갔다면 어느 방향으로 몇 mm인지 적어 두세요.")
    input("[4] 준비 자세를 거쳐 처음 자세로 돌아갑니다. Enter...")
    arm.move_to(*READY)
    arm.park()
    input("[5] 토크를 끕니다. 팔이 내려앉을 수 있으니 손으로 받치고 Enter...")
finally:
    arm.close()
