# fake_vision.py  -  카메라 없이 모션을 개발할 때 쓰는 가짜 비전
#   from fake_vision import get_blocks     (나중에 from vision import get_blocks 로 한 줄만 바꾸면 됨)
# 형식과 좌표계는 vision.py와 같음: 로봇 좌표 mm (원점 = 회전축, x = 정면, y = 왼쪽), angle 0~90도

BLOCKS = [
    {"color": "wood", "x": 120.0, "y": 0.0,   "angle": 0.0,  "conf": 0.95},
    {"color": "wood", "x": 150.0, "y": 60.0,  "angle": 30.0, "conf": 0.90},
    {"color": "wood", "x": 180.0, "y": -70.0, "angle": 45.0, "conf": 0.88},
    {"color": "wood", "x": 220.0, "y": 20.0,  "angle": 75.0, "conf": 0.80},
]


def get_blocks(obj_h=None):
    return [dict(b) for b in BLOCKS]


def close():
    pass


if __name__ == "__main__":
    for b in get_blocks():
        print(b)
