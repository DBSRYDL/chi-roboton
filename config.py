# config.py  -  비전 쪽 설정값은 전부 여기서만 바꿉니다
import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(BASE_DIR, "data")
PHOTO_DIR = os.path.join(DATA_DIR, "photos")
H_PATH = os.path.join(DATA_DIR, "H.npy")
WEIGHTS = os.path.join(DATA_DIR, "best.pt")

# ----- 카메라 -----
CAM_INDEX = 1            # tools/find_camera.py 로 확인
W, H_IMG = 1280, 720
TEAM = "team_4"            # 사진 파일 이름 앞에 붙음

# ----- 검출 -----
BACKEND = "hsv"          # "hsv" 또는 "yolo" (yolo는 data/best.pt 필요)
CONF = 0.5               # yolo 신뢰도 기준
MIN_AREA = 6000          # 이 픽셀 면적보다 작은 덩어리는 무시
MAX_AREA = 15000         # 블록 하나가 이보다 크면 무시 (손, 로봇팔 등). 붙은 블록은 나눈 뒤에 잼
TOP_V_RATIO = 0.85       # 윗면 밝기의 이 비율보다 어두운 곳(옆면, 그림자)은 지움. 0이면 끔
SEED_RATIO = 0.6         # 붙은 블록 나누기 민감도. 안 나뉘면 올리고(0.7), 하나가 쪼개지면 내리기(0.5)
# 색 이름: [(하한 HSV, 상한 HSV), ...]  tools/hsv_tuner.py 로 찾아서 고치세요 (지금 값은 임시)
HSV_RANGES = {
    "wood":  [((8, 25, 50), (30, 166, 204))],
    "red":   [((0, 120, 70), (8, 255, 255)), ((170, 120, 70), (179, 255, 255))],
    "green": [((40, 80, 60), (85, 255, 255))],
    "blue":  [((95, 100, 60), (130, 255, 255))],
}
COLORS = ["wood", "red", "green", "blue"]   # 실제로 찾을 색. 나무색만 쓰려면 ["wood"]

# ----- 좌표 보정 -----
# ArUco 마커(DICT_4X4_50) 중심의 작업판 좌표 mm. 지금은 튜터링 A4 판 값
MARKER_MM = {0: (0.0, 0.0), 1: (227.0, 0.0), 2: (227.0, 140.0), 3: (0.0, 140.0)}

BLOCK_MM = 25.0          # 블록 한 변
USE_HEIGHT_FIX = False   # 11/20 22:03 처음 잘 되던 설정으로 되돌림. 켜면 관절 매핑을 다시 해야 함 (값은 아래 그대로 둠)
CAM_H_MM = 400.0         # 바닥에서 카메라 렌즈까지 자로 잰 값 (11/20)
NADIR_MM = (100.0, 76.3)     # 카메라 바로 아래 점 (작업판 mm). calib_height(400) + 로봇 놓기 테스트로 구함 (11/20)

# 로봇 좌표: 원점 = 로봇 회전축, x = 로봇 정면, y = 로봇 왼쪽
ROBOT_BASE = (357.0, 70.0)   # 작업판 좌표에서 로봇 회전축 위치 mm (11/18 측정: ID1-ID2 선에서 130mm, 세로 가운데)
ROBOT_THETA_DEG = 180.0      # 로봇 정면이 작업판 +x 에서 몇 도 돌아가 있는지 (판을 -x 방향으로 바라봄)

# ----- get_blocks() -----
R_MIN, R_MAX = 90.0, 250.0   # 회전축에서 이 거리(mm) 밖의 검출은 버림
N_FRAMES = 10                # 한 번 부를 때 찍는 프레임 수
MERGE_MM = 15.0              # 이 거리 안의 검출은 같은 블록으로 봄

# ----- 로봇팔 (arm.py) -----
ARM_PORT = "COM5"             # 예: "COM5". tools/arm_probe.py 가 찾아 줌
ARM_ID = "so101_full"          # 11/20 손목까지 전체 범위로 다시 캘리브레이션
# IK 모델 각도 <-> LeRobot 모터 각도(도):  모델각 = SIGN * (모터각 - OFFSET)
# 순서: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll.  tools/arm_calib.py 로 구함
ARM_SIGN = [1, 1, 1, 1, 1]
ARM_OFFSET_DEG = [6.58, 5.09, 13.34, -16.43, -5.25]   # so101_full. 관절 1~4: 반복 테스트 15회 자료로 다시 맞춤 (11/20 23:30), 손목 회전: arm_roll_calib
ARM_MAPPING_CHECKED = True   # 11/20 arm_place_calib로 확인 (RMS 4.5mm). 캘리브레이션을 다시 하면 False로
ARM_DROOP_FIX = False        # 팔 무게로 처진 만큼 모터 목표를 더 밀어 줌. 처음 잘 되던 설정으로 되돌림
PICK_Z_EXTRA_MM = 5.0        # 집기·놓기 높이에 더할 여유. 바닥을 긁어서 5로 (11/20). 그래도 긁으면 8, 블록을 놓치면 3
JAW_CLEAR_MM = 8.0           # 고정 집게를 블록 옆면에서 띄우는 거리. 벌린 폭(약 47mm) - 블록 25mm 중 8은 고정 집게 쪽, 나머지는 움직이는 집게 쪽 여유
MOTOR_TEMP_WARN = 50         # 모터 온도(도C)가 이 이상이면 경고, 반복 테스트는 식을 때까지 쉼
MOTOR_TEMP_STOP = 60         # 이 이상이면 반복 테스트를 멈춤 (모터 보호. STS3215 자체 보호는 더 높은 온도에서 걸림)
ARM_SPEED_DEG_S = 25.0       # 관절 최대 속도 (도/초). 처음에는 낮게
GRIP_OPEN = 32.0             # so101_full 기준. 96이면 120mm나 벌어져서 옆 블록·바닥을 쓸음 -> 약 45~50mm 벌어지게 32 (11/20). 끝까지는 96
GRIP_CLOSE = 9.0             # so101_full 기준. 블록에 닿을 때 11.7 -> 살짝 더 조이게 9
