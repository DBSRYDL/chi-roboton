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
    try:                                    # LeRobot 0.6 (확인한 버전: 0.6.1)
        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
        return SO101Follower(SO101FollowerConfig(**kw))
    except ImportError:
        pass
    try:
        from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
        return SO101Follower(SO101FollowerConfig(**kw))
    except ImportError:
        pass
    try:                                    # 예전 경로
        from lerobot.common.robots.so101_follower import SO101Follower, SO101FollowerConfig
        return SO101Follower(SO101FollowerConfig(**kw))
    except ImportError as e:
        import sys
        try:
            import lerobot  # noqa
        except ImportError:
            raise RuntimeError(f"지금 파이썬({sys.executable})에는 LeRobot이 없습니다. lerobot 환경의 파이썬으로 실행하세요:\n"
                               r"  C:\Users\gyo30\miniforge3\envs\lerobot\python.exe tools\이름.py") from None
        raise RuntimeError("LeRobot은 있지만 SO-101 follower 클래스를 찾지 못했습니다. "
                           "python tools\\arm_probe.py 결과를 확인하세요.") from e


class Arm:
    def __init__(self):
        self.robot = None
        self.start_motor = None             # 연결했을 때의 자세 (park에서 여기로 돌아옴)
        self.motor_limit = None
        self.droop = np.zeros(5)            # 마지막 이동 직후 목표와 실제의 차이 (보정 전, 도)
        self.grip = None

    # ---------- 연결 ----------
    def connect(self, torque=True):
        for attempt in range(1, 4):         # 통신이 한 번 끊기는 일이 있어 3번까지 다시 시도
            self.robot = make_robot()
            if not getattr(self.robot, "calibration", None):
                raise RuntimeError(f"id '{C.ARM_ID}'의 캘리브레이션 파일이 없습니다. lerobot-calibrate를 먼저 하거나 ARM_ID를 확인하세요.")
            try:
                self.robot.connect(calibrate=False)  # 캘리브레이션을 새로 하지 않음. 연결이 끝나면 LeRobot이 토크를 켬
                break
            except ConnectionError as e:
                print(f"[연결 {attempt}/3 실패] {str(e).splitlines()[-1]}")
                try:
                    self.robot.bus.port_handler.closePort()
                except Exception:
                    pass
                self.robot = None
                if attempt == 3:
                    raise RuntimeError("모터와 통신이 계속 실패합니다. 오류에 나온 id의 모터 케이블과 전원 어댑터를 확인하고 "
                                       "python tools\\arm_scan.py 로 모터 6개가 다 보이는지 보세요.") from e
                time.sleep(1.0)
        if not torque:
            self.robot.bus.disable_torque()
        if not self.robot.is_calibrated:
            self.robot.bus.disable_torque()
            self.robot.disconnect()
            self.robot = None
            raise RuntimeError(f"모터에 저장된 캘리브레이션이 파일(id '{C.ARM_ID}')과 다릅니다. 이 상태로는 각도가 틀리게 읽힙니다. "
                               "이 팔로 캘리브레이션한 id가 맞는지 확인하거나 LeRobot 캘리브레이션을 다시 하세요.")
        # 캘리브레이션 때 기록된 각 관절의 가동 범위 (도). LeRobot의 0도 = 이 범위의 한가운데
        cal = self.robot.calibration
        half = np.array([(cal[n].range_max - cal[n].range_min) / 2 * 360 / 4095 for n in JOINTS])
        self.motor_half = half              # 캘리브레이션 범위의 절반 (모터가 실제로 멈추는 한계)
        self.motor_limit = half - 3.0       # 새 목표는 양 끝에서 3도 여유
        m = self.read_motor()
        self.start_motor = m.copy()
        self.grip = float(m[5])
        return self

    def close(self):
        if self.robot is not None:
            try:
                self.robot.disconnect()     # LeRobot 기본 설정이면 여기서 토크가 꺼짐 -> 팔이 내려앉음
            except ConnectionError:
                print("모터가 응답하지 않아 토크를 끄지 못했습니다. 팔을 받친 채 전원 어댑터를 뽑으세요.")
                try:
                    self.robot.bus.port_handler.closePort()
                except Exception:
                    pass
            self.robot = None

    def relax(self):
        self.robot.bus.disable_torque()

    def hold(self):
        self.robot.bus.enable_torque()

    # ---------- 읽기 ----------
    def read_motor(self):
        """모터 6개 현재 값: 관절 5개(도) + 그리퍼(0~100)"""
        pos = self.robot.bus.sync_read("Present_Position", num_retry=3)   # 깨진 응답이 가끔 와서 다시 읽음
        return np.array([float(pos[n]) for n in MOTORS])

    def read_temps(self):
        """모터 6개 온도 (도C). 읽기 실패하면 None"""
        try:
            t = self.robot.bus.sync_read("Present_Temperature", normalize=False, num_retry=2)
            return np.array([float(t[n]) for n in MOTORS])
        except Exception:
            return None

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

    def move_motor(self, target_deg, speed=None, check_limits=True):
        """모터 각도(도) 5개로 천천히 이동. 가장 많이 도는 관절이 speed(도/초)를 넘지 않게 잘게 나눠 보냄"""
        self._check()
        speed = speed or C.ARM_SPEED_DEG_S
        start = self.read_motor()[:5]
        target = np.asarray(target_deg, float)
        if not check_limits:                # 원래 있던 자세로 돌아갈 때: 모터 한계까지만 맞춰서 보냄
            target = np.clip(target, -self.motor_half, self.motor_half)
        over = np.abs(target) > (self.motor_limit if check_limits else self.motor_half + 1e-6)
        if over.any():
            raise ValueError("캘리브레이션 가동 범위를 넘는 목표입니다: " +
                             ", ".join(f"{JOINTS[i]} {target[i]:.0f}도 (한계 ±{self.motor_limit[i]:.0f})" for i in np.where(over)[0]))
        steps = max(1, int(math.ceil(np.max(np.abs(target - start)) / (speed * DT))))
        for i in range(1, steps + 1):
            s = i / steps
            s = s * s * (3 - 2 * s)         # 부드럽게 출발하고 멈춤
            self._send(start + (target - start) * s, self.grip)
            time.sleep(DT)
        time.sleep(0.3)
        # 처짐 보정: 팔 무게 때문에 모터가 목표보다 덜 가서 멈춤(특히 팔을 뻗었을 때 아래로 처짐).
        # 실제 각도를 읽어 모자란 만큼 목표를 더 밀어 줌 (최대 6도, 2번)
        cmd = target.copy()
        for i in range(2 if C.ARM_DROOP_FIX else 0):
            err = target - self.read_motor()[:5]
            if i == 0:
                self.droop = err
            if np.max(np.abs(err)) < 0.7:
                break
            cmd = np.clip(cmd + np.clip(err, -6, 6), -self.motor_half, self.motor_half)
            self._send(cmd, self.grip)
            time.sleep(0.35)

    def move_q(self, q, speed=None):
        if not K.within_limits(q):
            raise ValueError("관절 한계를 넘는 자세입니다.")
        self.move_motor(motor_from_q(q), speed)

    def motor_ok(self, q):
        """IK 해 q를 모터 각도로 바꿨을 때 캘리브레이션 가동 범위 안인지"""
        return bool(np.all(np.abs(motor_from_q(q)) <= self.motor_limit))

    def solve(self, x, y, z, yaw_deg=None, max_tilt_deg=30, jaw_clear=0.0):
        """(q, 기울임 도) 또는 (None, None). 수직부터 시도하고, 모터 가동 범위를 넘으면 조금씩 기울여 다시 풂
        jaw_clear: 고정 집게를 블록 옆면에서 이만큼(mm) 더 띄움. 집기 중심을 고정 집게 쪽으로 옮겨서,
                   내려갈 때 고정 집게가 블록 윗면에 걸리지 않고, 닫을 때 움직이는 집게가 블록을 밀어 붙임"""
        yaw = None if yaw_deg is None else math.radians(yaw_deg)
        q_now = self.read_q() if self.robot is not None else None
        for t in K.TILT_STEPS_DEG:
            if t > max_tilt_deg:
                break
            for prev in (q_now, None):        # 지금 자세와 가까운 해 먼저, 안 되면 아무 해나
                q = K.ik_topdown(x, y, z, yaw=yaw, q_prev=prev, tilt=math.radians(t))
                if q is None or not self.motor_ok(q):
                    continue
                if jaw_clear:
                    a = K.fk(q)[:3, K.JAW_AXIS_COL]           # 고정 집게 -> 움직이는 집게 방향
                    a = a[:2] / max(np.linalg.norm(a[:2]), 1e-9)
                    q2 = K.ik_topdown(x - jaw_clear * a[0], y - jaw_clear * a[1], z, yaw=yaw, q_prev=q, tilt=math.radians(t))
                    if q2 is None or not self.motor_ok(q2):
                        continue
                    a2 = K.fk(q2)[:3, K.JAW_AXIS_COL][:2]
                    if a2 @ a < 0.9 * np.linalg.norm(a2):        # 집게 방향이 뒤집힌 해면 버림
                        continue
                    q = q2
                return q, t
        return None, None

    def move_to(self, x, y, z, yaw_deg=None, speed=None, max_tilt_deg=30, jaw_clear=0.0):
        """집기 중심(TCP)을 (x, y, z) mm로. yaw_deg = 집게가 벌어지는 방향(블록 angle을 그대로 넣으면 됨)
        jaw_clear는 solve() 설명 참고. 반환: 기울인 각도(도). 갈 수 없으면 None (팔은 움직이지 않음)"""
        self._check()
        q, tilt = self.solve(x, y, z, yaw_deg, max_tilt_deg, jaw_clear)
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
            self.move_motor(self.start_motor[:5], check_limits=False)   # 쉬는 자세는 범위 끝에 걸쳐 있을 수 있음
