# arm_read.py  -  토크를 끄고 관절 값을 실시간으로 읽기만 함 (팔은 스스로 움직이지 않음)
# 손으로 팔을 움직이며 화면의 값이 실제와 맞는지 봅니다.
#   motor : LeRobot이 읽은 모터 각도(도), 마지막은 그리퍼(0~100)
#   tip   : 지금 매핑으로 계산한 고정 집게 손끝 위치 mm (로봇 좌표: x 정면, y 왼쪽, z 위)
# 그리퍼를 손으로 벌리고 닫아 보며 gripper 값을 읽어 config.py의 GRIP_OPEN / GRIP_CLOSE에 넣으세요.
# Ctrl+C: 종료
import _path  # noqa
import time
import numpy as np
import so101_ik as K
from arm import Arm, q_from_motor

input("토크를 끄면 팔이 내려앉습니다. 팔을 손으로 받친 뒤 Enter...")
arm = Arm().connect(torque=False)
np.set_printoptions(precision=1, suppress=True, floatmode="fixed")
try:
    while True:
        m = arm.read_motor()
        q = q_from_motor(m)
        tip = K.fk(q, site=True)[:3, 3]
        print(f"motor {m[:5]} grip {m[5]:5.1f} | tip x={tip[0]:6.1f} y={tip[1]:6.1f} z={tip[2]:6.1f}", end="\r")
        time.sleep(0.1)
except KeyboardInterrupt:
    print()
finally:
    arm.close()
