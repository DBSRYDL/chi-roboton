"""SO-101 forward/inverse kinematics (MuJoCo 없이 numpy만 사용).

좌표계 (로봇 베이스 기준, 단위 mm)
  원점: shoulder_pan 회전축이 바닥과 만나는 점
  x: 로봇 정면, y: 왼쪽, z: 위
관절각: 라디안, 순서 = JOINT_NAMES (gripper 제외 5개)

상수는 SO-ARM100/Simulation/SO101/so101_new_calib.xml 의 영점 자세(q=0)에서
추출했어요. 실물 로봇의 관절각 영점·방향과 이 모델의 영점·방향을 맞추는 것은
calib 단계에서 따로 해야 해요 (motor_to_model / model_to_motor).
"""
from __future__ import annotations

import math
import numpy as np

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

# 모델 원점(base body)에서 pan 축까지의 x 거리. 시뮬 장면을 만들 때 씀.
PAN_OFFSET_X_MM = 38.8353

# q=0 자세에서 각 관절 축 위의 한 점(mm)과 축 방향 (로봇 좌표계)
_ANCHORS = np.array([
    [0.0, 0.0, 62.4],
    [30.3992, -18.2778, 116.6],
    [58.3992, -18.2778, 229.17],
    [193.2992, -18.2778, 234.37],
    [254.3992, -0.1778, 234.37],
])
_AXES = np.array([
    [0.0, 0.0, -1.0],
    [0.0, 1.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0],
])
# q=0 자세의 gripperframe site 자세. 이 점은 "고정 집게 손끝"이에요 (두 집게 가운데가 아님).
#   열벡터 0 = 접근 방향(손가락이 향하는 쪽)
#   열벡터 2 = 집게 축 (고정 집게 안쪽면이 z=0, 움직이는 집게는 +z 쪽에서 닫힘)
_M_SITE = np.eye(4)
_M_SITE[:3, :3] = np.array([
    [1.0, 0.0, 0.0],
    [0.0, 0.998815, -0.04866],
    [0.0, 0.04866, 0.998815],
])
_M_SITE[:3, 3] = [352.5266, -0.0112, 226.4687]

# TCP(집기 중심) = 25mm 블록을 잡았을 때 블록 중심 위치, site 좌표계 기준 (mm).
#   x=-2: 손끝이 블록 중심보다 2mm 아래까지 내려감 (바닥과 약 4mm 여유)
#   z=+12.5: 고정 집게 안쪽면에서 블록 반 폭만큼
# 시뮬에서 잡은 뒤 블록이 TCP에서 1~2mm 안에 오는 값으로 맞췄어요.
# 실물 그리퍼(V홈 등)로 바꾸면 이 값을 다시 재야 해요.
TCP_OFFSET_MM = np.array([-2.0, 0.0, 12.5])
_M = _M_SITE.copy()
_M[:3, 3] = _M_SITE[:3, 3] + _M_SITE[:3, :3] @ TCP_OFFSET_MM
JAW_AXIS_COL = 2

# 관절 한계 (rad). 실물에서는 여기서 여유를 더 줘서 쓰세요.
JOINT_LIMITS = np.array([
    [-1.91986, 1.91986],
    [-1.74533, 1.74533],
    [-1.69, 1.69],
    [-1.65806, 1.65806],
    [-2.74385, 2.84121],
])
LIMIT_MARGIN = math.radians(3.0)


def _skew(w):
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])


def _exp_twist(w, p, theta):
    """회전축 w(단위벡터)가 점 p를 지날 때 theta만큼 돌리는 4x4 변환."""
    K = _skew(w)
    R = np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * (K @ K)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = (np.eye(3) - R) @ p
    return T


def fk(q, site=False) -> np.ndarray:
    """관절각 q(5개, rad) → TCP(집기 중심)의 4x4 자세 (mm). site=True면 gripperframe."""
    T = np.eye(4)
    for i in range(5):
        T = T @ _exp_twist(_AXES[i], _ANCHORS[i], q[i])
    return T @ (_M_SITE if site else _M)


def tool_pos(q) -> np.ndarray:
    return fk(q)[:3, 3]


def within_limits(q, margin: float = LIMIT_MARGIN) -> bool:
    q = np.asarray(q)
    return bool(np.all(q >= JOINT_LIMITS[:, 0] + margin) and np.all(q <= JOINT_LIMITS[:, 1] - margin))


# ---------------------------------------------------------------- IK
# 평면(팔이 놓인 수직면) 2링크 근사로 초기값을 구한 뒤, 전체 모델로 뉴턴 보정.
_S = _ANCHORS[1][[0, 2]]           # shoulder_lift (r, z)
_E = _ANCHORS[2][[0, 2]]
_W = _ANCHORS[3][[0, 2]]
_V1, _V2 = _E - _S, _W - _E
_L1, _L2 = np.linalg.norm(_V1), np.linalg.norm(_V2)
_PHI1, _PHI2 = math.atan2(_V1[1], _V1[0]), math.atan2(_V2[1], _V2[0])
_TOOL_FROM_WRIST_DOWN = np.array([-(_M[2, 3] - _W[1]), -(_M[0, 3] - _W[0])])  # 수직 하향일 때 (dr, dz)


def _seeds(x, y, z):
    pan = -math.atan2(y, x)
    r = math.hypot(x, y)
    wrist = np.array([r, z]) - _TOOL_FROM_WRIST_DOWN
    D = wrist - _S
    d = np.linalg.norm(D)
    c = (d * d - _L1 * _L1 - _L2 * _L2) / (2 * _L1 * _L2)
    out = []
    if abs(c) <= 1.0:
        for gamma in (math.acos(c), -math.acos(c)):
            q2 = _PHI1 - (math.atan2(D[1], D[0]) - math.atan2(_L2 * math.sin(gamma), _L1 + _L2 * math.cos(gamma)))
            q3 = _PHI2 - _PHI1 - gamma
            q4 = math.pi / 2 - q2 - q3
            out.append(np.array([pan, q2, q3, q4, 0.0]))
    out.append(np.array([pan, 0.3, 0.3, 1.0, 0.0]))
    return out


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _residual(q, target, yaw, tilt=0.0):
    T = fk(q)
    approach = T[:3, 0]
    jaw = T[:3, JAW_AXIS_COL]
    # 기울기 방향은 팔이 놓인 수직면(= pan 각) 기준. pan 축이 -z라서 방향은 -q[0].
    ux, uy = math.cos(-q[0]), math.sin(-q[0])
    want = np.array([math.sin(tilt) * ux, math.sin(tilt) * uy, -math.cos(tilt)])
    res = [*(T[:3, 3] - target), *(100.0 * (approach - want))]
    if yaw is not None:
        d = (math.cos(yaw), math.sin(yaw))
        res.append(100.0 * (jaw[0] * d[1] - jaw[1] * d[0]))   # 집게 축이 yaw 방향과 평행 (0 또는 180°)
    return np.array(res)


def _newton(q, target, yaw, tilt=0.0, iters=60, tol=1e-3):
    q = q.copy()
    for _ in range(iters):
        f = _residual(q, target, yaw, tilt)
        if np.linalg.norm(f) < tol:
            return q, True
        J = np.zeros((len(f), 5))
        h = 1e-6
        for i in range(5):
            dq = np.zeros(5); dq[i] = h
            J[:, i] = (_residual(q + dq, target, yaw, tilt) - f) / h
        lam = 1e-3
        step = np.linalg.solve(J.T @ J + lam * np.eye(5), -J.T @ f)
        q += step
        q[4] = _wrap(q[4])
    return q, np.linalg.norm(_residual(q, target, yaw, tilt)) < 0.05


def ik_topdown(x, y, z, yaw=None, q_prev=None, tilt=0.0):
    """그리퍼가 아래를 향한 채 TCP(집기 중심)를 (x, y, z) mm에 놓는 관절각.

    yaw: 집게가 벌어지는 축의 방향 (rad, 로봇 x축 기준). 블록은 90° 대칭이라
         yaw, yaw+90° 중 wrist_roll이 덜 도는 쪽을 고름. None이면 롤은 자유.
    q_prev: 주면 그와 가장 가까운 해를 고름 (연속 동작용).
    tilt: 수직에서 바깥쪽(로봇 반대쪽)으로 기울이는 각도 (rad). 0이면 정확히 수직.
    반환: 관절각 5개 (rad) 또는 None (도달 불가/한계 초과).
    """
    target = np.array([x, y, z], dtype=float)
    yaws = [None] if yaw is None else [yaw, yaw + math.pi / 2]
    best, best_cost = None, float("inf")
    # TCP가 롤 축에서 떨어져 있어서 롤을 180° 돌리면 다른 해가 됨 → 롤 초기값을 여러 개로 시도
    roll_seeds = [q_prev[4]] if q_prev is not None else [0.0, math.pi / 2, -math.pi / 2, math.pi, -math.pi]
    for yw in yaws:
        for s0 in _seeds(x, y, z):
            for roll in roll_seeds:
                s = s0.copy()
                s[3] -= tilt
                s[4] = roll
                q, ok = _newton(s, target, yw, tilt)
                if not ok or not within_limits(q):
                    continue
                ref = q_prev if q_prev is not None else np.zeros(5)
                cost = float(np.sum((q - ref) ** 2)) + (abs(q[4]) if q_prev is None else 0.0)
                # 팔꿈치가 위로 가는 자세 선호 (elbow 관절 z가 높을수록 좋음)
                cost -= 1e-3 * _elbow_height(q)
                if cost < best_cost:
                    best, best_cost = q, cost
    return best


TILT_STEPS_DEG = (0, 5, 10, 15, 20, 25, 30)


def ik_auto(x, y, z, yaw=None, q_prev=None, max_tilt_deg=30):
    """수직(tilt 0)부터 시도하고, 안 되면 바깥쪽으로 조금씩 기울여서 풀리는 첫 해를 반환.

    반환: (q, tilt_deg) 또는 (None, None)
    높은 층에 놓을 때 wrist_flex 한계 때문에 수직이 안 되는 경우를 위한 함수예요.
    """
    for t in TILT_STEPS_DEG:
        if t > max_tilt_deg:
            break
        q = ik_topdown(x, y, z, yaw=yaw, q_prev=q_prev, tilt=math.radians(t))
        if q is not None:
            return q, t
    return None, None


def _elbow_height(q):
    T = _exp_twist(_AXES[0], _ANCHORS[0], q[0]) @ _exp_twist(_AXES[1], _ANCHORS[1], q[1])
    return (T @ np.append(_ANCHORS[2], 1.0))[2]


if __name__ == "__main__":
    np.set_printoptions(precision=2, suppress=True)
    for p in [(150, 0, 12.5), (200, 80, 12.5), (90, -40, 40), (250, 0, 12.5)]:
        q = ik_topdown(*p, yaw=0.3)
        if q is None:
            print(p, "-> 도달 불가")
            continue
        T = fk(q)
        print(p, "-> q(deg)", np.degrees(q), " FK", T[:3, 3], " approach", T[:3, 0])
