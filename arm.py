# arm.py  -  실물 SO-101 구동: IK 관절각 -> 모터 명령
#   arm = Arm(); arm.connect()
#   arm.move_to(x, y, z, yaw_deg)   # 로봇 좌표 mm, 그리퍼 아래 방향
#   arm.grip_open(); arm.grip_close()
#   arm.park(); arm.close()
# LeRobot의 SO-101 follower 클래스와 그 캘리브레이션 파일을 그대로 씁니다.
# 주의: 이 파일은 실물 로봇 없이 작성됐습니다. tools/arm_probe.py -> arm_read.py -> arm_calib.py 순서로 확인한 뒤 쓰세요.
import math
import time
import numpy as np
import config as C
import so101_ik as K

JOINTS = K.JOINT_NAMES                      # 5개
MOTORS = JOINTS + ["gripper"]
DT = 0.05                                   # 명령 주기 (초)


def q_from_motor(deg):
    """모터 각도(도) 5개 -> IK 모델 각도(rad)"""
    return np.radians(np.array(C.ARM_SIGN) * (np.asarray(deg[:5], float) - np.array(C.ARM_OFFSET_DEG)))


def motor_from_q(q):
    """IK 모델 각도(rad) 5개 -> 모터 각도(도)"""
    return np.array(C.ARM_SIGN) * np.degrees(np.asarray(q, float)) + np.array(C.ARM_OFFSET_DEG)


def make_robot():
    """설치된 LeRobot 버전에 맞는 SO-101 follower 객체를 만듦"""
    if not C.ARM_PORT or not C.ARM_ID:
        raise RuntimeError("config.py의 ARM_PORT, ARM_ID를 먼저 채우세요 (python tools\\arm_probe.py)")
    kw = dict(port=C.ARM_PORT, id=C.ARM_ID, use_degrees=True)
    try:
        from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
        return SO101Follower(SO101FollowerConfig(**kw))
    except ImportError:
        pass
    try:
        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
        return SO101Follower(SO101FollowerConfig(**kw))
    except ImportError:
        pass
    try:                                    # 예전 경로
        from lerobot.common.robots.so101_follower import SO101Follower, SO101FollowerConfig
        return SO101Follower(SO101FollowerConfig(**kw))
    except ImportError as e:
        raise RuntimeError("LeRobot에서 SO-101 follower 클래스를 찾지 못했습니다. "
                           "python tools\\arm_probe.py 결과를 확인하세요.") from e


class Arm:
    def __init__(self):
        self.robot = None
        self.start_motor = None             # 연결했을 때의 자세 (park에서 여기로 돌아옴)
        self.grip = None

    # ---------- 연결 ----------
    def connect(self, torque=True):
        self.robot = make_robot()
        if not getattr(self.robot, "calibration", None):
            raise RuntimeError(f"id '{C.ARM_ID}'의 캘리브레이션 파일이 없습니다. lerobot-calibrate를 먼저 하거나 ARM_ID를 확인하세요.")
        self.robot.connect(calibrate=False)  # 캘리브레이션을 새로 하지 않음
        if not torque:
            self.robot.bus.disable_torque()
        m = self.read_motor()
        self.start_motor = m.copy()
        self.grip = float(m[5])
        return self

    def close(self):
        if self.robot is not None:
            self.robot.disconnect()         # LeRobot 기본 설정이면 여기서 토크가 꺼짐 -> 팔이 내려앉음
            self.robot = None

    def relax(self):
        self.robot.bus.disable_torque()

    def hold(self):
        self.robot.bus.enable_torque()

    # ---------- 읽기 ----------
    def read_motor(self):
        """모터 6개 현재 값: 관절 5개(도) + 그리퍼(0~100)"""
        pos = self.robot.bus.sync_read("Present_Position")
        return np.array([float(pos[n]) for n in MOTORS])

    def read_q(self):
        return q_from_motor(self.read_motor())

    def tcp(self):
        """현재 집기 중심 위치 mm (로봇 좌표)"""
        return K.tool_pos(self.read_q())

    # ---------- 움직이기 ----------
    def _send(self, joint_deg, grip):
        action = {f"{n}.pos": float(v) for n, v in zip(JOINTS, joint_deg)}
        action["gripper.pos"] = float(grip)
        self.robot.send_action(action)

    def _check(self):
        if not C.ARM_MAPPING_CHECKED:
            raise RuntimeError("관절 매핑을 아직 확인하지 않았습니다. python tools\\arm_calib.py 를 하고 "
                               "config.py의 ARM_MAPPING_CHECKED를 True로 바꾸세요.")

    def move_motor(self, target_deg, speed=None):
        """모터 각도(도) 5개로 천천히 이동. 가장 많이 도는 관절이 speed(도/초)를 넘지 않게 잘게 나눠 보냄"""
        self._check()
        speed = speed or C.ARM_SPEED_DEG_S
        start = self.read_motor()[:5]
        target = np.asarray(target_deg, float)
        steps = max(1, int(math.ceil(np.max(np.abs(target - start)) / (speed * DT))))
        for i in range(1, steps + 1):
            s = i / steps
            s = s * s * (3 - 2 * s)         # 부드럽게 출발하고 멈춤
            self._send(start + (target - start) * s, self.grip)
            time.sleep(DT)
        time.sleep(0.3)

    def move_q(self, q, speed=None):
        if not K.within_limits(q):
            raise ValueError("관절 한계를 넘는 자세입니다.")
        self.move_motor(motor_from_q(q), speed)

    def move_to(self, x, y, z, yaw_deg=None, speed=None, max_tilt_deg=30):
        """집기 중심(TCP)을 (x, y, z) mm로. yaw_deg = 집게가 벌어지는 방향(블록 angle을 그대로 넣으면 됨)
        반환: 기울인 각도(도). 갈 수 없으면 None (팔은 움직이지 않음)"""
        self._check()
        yaw = None if yaw_deg is None else math.radians(yaw_deg)
        q, tilt = K.ik_auto(x, y, z, yaw=yaw, q_prev=self.read_q(), max_tilt_deg=max_tilt_deg)
        if q is None:
            q, tilt = K.ik_auto(x, y, z, yaw=yaw, max_tilt_deg=max_tilt_deg)   # 현재 자세와 무관한 해도 시도
        if q is None:
            return None
        self.move_q(q, speed)
        return tilt

    def set_grip(self, value, wait=0.6):
        self._check()
        start, n = self.grip, 10
        m = self.read_motor()[:5]
        for i in range(1, n + 1):
            self.grip = start + (value - start) * i / n
            self._send(m, self.grip)
            time.sleep(DT)
        time.sleep(wait)

    def grip_open(self):
        if C.GRIP_OPEN is None:
            raise RuntimeError("config.py의 GRIP_OPEN을 먼저 재서 넣으세요 (tools\\arm_read.py)")
        self.set_grip(C.GRIP_OPEN)

    def grip_close(self):
        if C.GRIP_CLOSE is None:
            raise RuntimeError("config.py의 GRIP_CLOSE를 먼저 재서 넣으세요 (tools\\arm_read.py)")
        self.set_grip(C.GRIP_CLOSE)

    def park(self):
        """연결했을 때의 자세로 천천히 돌아감 (끝내기 전에 호출)"""
        if self.start_motor is not None and C.ARM_MAPPING_CHECKED:
            self.move_motor(self.start_motor[:5])
