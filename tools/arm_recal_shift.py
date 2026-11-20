# arm_recal_shift.py  -  LeRobot 캘리브레이션을 다시 한 뒤, 관절 매핑(ARM_OFFSET_DEG)을 새 캘리브레이션에 맞게 옮겨 줌
# 같은 팔 자세라도 캘리브레이션이 바뀌면 LeRobot이 읽는 각도가 관절마다 일정한 값만큼 달라집니다.
# 두 캘리브레이션 파일(이전 id, 새 id)을 비교해 그 차이를 계산하므로, 매핑을 처음부터 다시 잴 필요가 없습니다.
#   python tools\arm_recal_shift.py so101_team so101_full
import _path  # noqa
import sys
import json
import config as C
from lerobot.utils.constants import HF_LEROBOT_CALIBRATION

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
TICK = 360 / 4095


def load(i):
    p = HF_LEROBOT_CALIBRATION / "robots" / "so_follower" / f"{i}.json"
    with open(p, encoding="utf-8") as f:
        return json.load(f), p


old_id, new_id = sys.argv[1], sys.argv[2]
old, po = load(old_id)
new, pn = load(new_id)
print("이전:", po)
print("새로:", pn)
shift, out = [], []
for j, n in enumerate(JOINTS):
    o, w = old[n], new[n]
    # 같은 실제 자세에서: 새 읽기 - 이전 읽기 = (이전 homing - 새 homing) + (이전 범위 중앙 - 새 범위 중앙)
    d = ((o["homing_offset"] - w["homing_offset"]) + ((o["range_min"] + o["range_max"]) / 2 - (w["range_min"] + w["range_max"]) / 2)) * TICK
    d = (d + 180) % 360 - 180
    shift.append(d)
    out.append(round(C.ARM_OFFSET_DEG[j] + C.ARM_SIGN[j] * d, 2) if j < 4 else C.ARM_OFFSET_DEG[j])
    span_o = (o["range_max"] - o["range_min"]) * TICK
    span_n = (w["range_max"] - w["range_min"]) * TICK
    print(f"  {n:14s} 읽기 차이 {d:+7.2f}도   가동 범위 {span_o:5.0f} -> {span_n:5.0f}도")
print("\nconfig.py에 넣을 값:")
print(f'ARM_ID = "{new_id}"')
print(f"ARM_OFFSET_DEG = {out}")
print("그리퍼는 0~100 비율이라 범위가 바뀌면 값이 달라집니다. arm_read.py로 GRIP_OPEN / GRIP_CLOSE를 다시 재세요.")
