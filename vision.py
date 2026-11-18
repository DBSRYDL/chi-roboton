# vision.py  -  모션 쪽이 쓰는 유일한 창구
#   from vision import get_blocks
#   blocks = get_blocks()   # 팔이 화면을 가리지 않는 대기 자세에서만 부르기
# 돌려주는 값: [{"color": "wood", "x": 152.3, "y": -41.0, "angle": 27.5, "conf": 0.91}, ...]
#   x, y  : 로봇 좌표 mm (원점 = 회전축, x = 정면, y = 왼쪽), 블록 윗면 중심
#   angle : 블록 변의 방향, 로봇 x축 기준 0~90도
#   가까운 블록부터 정렬. 없으면 빈 리스트
import numpy as np
import config as C
import calib

_cap = None
_H = None


def _mean_angle(angles):
    """0도와 90도가 같은 방향이므로 4배 해서 원 위에서 평균"""
    a = np.radians(np.array(angles) * 4.0)
    return float(np.degrees(np.arctan2(np.sin(a).mean(), np.cos(a).mean())) / 4.0 % 90.0)


def merge(per_frame):
    """프레임별 검출 [[{color,x,y,angle,conf}, ...], ...]을 위치로 묶어 블록 하나당 하나로"""
    groups = []
    for dets in per_frame:
        for d in dets:
            for g in groups:
                gx, gy = np.median([[p["x"], p["y"]] for p in g], axis=0)
                if np.hypot(d["x"] - gx, d["y"] - gy) < C.MERGE_MM:
                    g.append(d)
                    break
            else:
                groups.append([d])
    out = []
    need = max(1, len(per_frame) // 3)                  # 몇 프레임만 잡힌 헛것은 버림
    for g in groups:
        if len(g) < need:
            continue
        x, y = np.median([[p["x"], p["y"]] for p in g], axis=0)
        if not (C.R_MIN <= np.hypot(x, y) <= C.R_MAX):
            continue
        names = [p["color"] for p in g]
        out.append({"color": max(set(names), key=names.count),
                    "x": round(float(x), 1), "y": round(float(y), 1),
                    "angle": round(_mean_angle([p["angle"] for p in g]), 1),
                    "conf": round(float(np.mean([p["conf"] for p in g])), 2)})
    return sorted(out, key=lambda b: np.hypot(b["x"], b["y"]))


def frame_blocks(frame, H, obj_h=None):
    """프레임 한 장 -> 로봇 좌표 검출 목록 (거르기 전)"""
    from detector import detect
    out = []
    for d in detect(frame):
        x, y = calib.to_robot(H, d["u"], d["v"], obj_h)
        out.append({"color": d["color"], "x": x, "y": y,
                    "angle": calib.block_angle(H, d["corners"], obj_h), "conf": d["conf"],
                    "u": d["u"], "v": d["v"], "corners": d["corners"]})
    return out


def get_blocks(obj_h=None):
    """obj_h: 블록 윗면 높이 mm. 바닥의 블록은 기본값(25), 2층 위 블록은 50 ..."""
    global _cap, _H
    import camera
    if _cap is None:
        _cap = camera.open_camera()
    if _H is None:
        _H = calib.load_H()
    frames = camera.grab(_cap, C.N_FRAMES)
    return merge([frame_blocks(f, _H, obj_h) for f in frames])


def close():
    global _cap
    if _cap is not None:
        _cap.release()
        _cap = None


if __name__ == "__main__":
    for b in get_blocks():
        print(b)
    close()
