# detector.py  -  프레임 한 장에서 블록 찾기
# detect(frame) -> [{"color", "u", "v", "corners"(4x2 픽셀), "conf"}, ...]
import os
import cv2
import numpy as np
import config as C

_kernel = np.ones((5, 5), np.uint8)
_model = None
rejected = []      # 디버깅용: 마지막 프레임에서 버린 후보 [(꼭짓점, 이유), ...]
masks = {}         # 디버깅용: 마지막 프레임의 색별 마스크


def color_mask(hsv, color):
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in C.HSV_RANGES[color]:
        mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _kernel)


def top_faces(mask, hsv):
    """윗면만 남기기: 비스듬한 카메라에는 블록 옆면도 보이는데, 옆면은 윗면보다 어두움
    -> 블록 색 픽셀 전체의 밝기 중앙값(= 윗면 밝기)보다 일정 비율 어두운 곳을 지움
    그다음 나뭇결·옹이 때문에 생긴 구멍을 메움 (구멍이 있으면 블록 나누기가 흔들림)"""
    if C.TOP_V_RATIO and mask.any():
        v = hsv[:, :, 2]
        mask = mask.copy()
        mask[v < np.median(v[mask > 0]) * C.TOP_V_RATIO] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filled = np.zeros_like(mask)
    cv2.drawContours(filled, contours, -1, 255, cv2.FILLED)
    return filled


def split_blobs(mask, frame):
    """붙어 있는 블록을 나눠 블록 하나당 윤곽선 하나씩 돌려줌
    1) 거리 변환: 블록 한가운데는 '깊고' 맞닿은 목 부분은 얕음 -> 깊은 곳만 남기면 블록마다 씨앗 하나
    2) 워터셰드: 씨앗에서 물을 채워 블록 사이 경계를 정함"""
    dist = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
    n, labels = cv2.connectedComponents(mask)
    seeds = np.zeros(mask.shape, np.uint8)
    for i in range(1, n):
        comp = labels == i
        seeds[comp & (dist > C.SEED_RATIO * dist[comp].max())] = 255
    n_seed, markers = cv2.connectedComponents(seeds)
    markers = markers + 1                      # 배경 = 1, 씨앗 = 2, 3, ...
    markers[(mask > 0) & (seeds == 0)] = 0     # 아직 주인이 없는 영역
    markers = cv2.watershed(frame, markers.astype(np.int32))
    out = []
    for m in range(2, n_seed + 1):
        region = ((markers == m) & (mask > 0)).astype(np.uint8) * 255
        cs, _ = cv2.findContours(region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if cs:
            out.append(max(cs, key=cv2.contourArea))
    return out


def split_rect(rect):
    """변끼리 딱 붙어 긴 직사각형 하나로 잡힌 덩어리를 정사각형 2~4개로 자름"""
    (u, v), (w, h), ang = rect
    long_, short = max(w, h), min(w, h)
    k = int(round(long_ / max(short, 1)))
    if k < 2 or k > 4:
        return [rect]
    t = np.radians(ang if w >= h else ang + 90)          # 긴 변 방향
    step = long_ / k
    rects = []
    for i in range(k):
        off = (i - (k - 1) / 2) * step
        center = (u + off * np.cos(t), v + off * np.sin(t))
        rects.append((center, (step, short) if w >= h else (short, step), ang))
    return rects


def detect_hsv(frame):
    blur = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    found = []
    rejected.clear()
    masks.clear()
    for color in C.COLORS:
        masks[color] = top_faces(color_mask(hsv, color), hsv)
        for cnt in split_blobs(masks[color], blur):
            area = cv2.contourArea(cnt)
            rect = cv2.minAreaRect(cnt)                 # ((u, v), (w, h), 각도)
            if area < C.MIN_AREA:
                if area > 100:
                    rejected.append((cv2.boxPoints(rect), f"small {area:.0f}"))
                continue
            fill = area / max(rect[1][0] * rect[1][1], 1)   # 네모에 가까울수록 1
            for r in split_rect(rect):
                (u, v), (w, h), _ = r
                if w * h > C.MAX_AREA:                  # 나눈 뒤에도 너무 크면 블록이 아님 (손, 로봇팔)
                    rejected.append((cv2.boxPoints(r), f"big {w * h:.0f}"))
                    continue
                found.append({"color": color, "u": float(u), "v": float(v),
                              "corners": cv2.boxPoints(r), "conf": float(min(fill, 1.0))})
    return found


def detect_yolo(frame):
    global _model
    if _model is None:
        from ultralytics import YOLO
        if not os.path.exists(C.WEIGHTS):
            raise FileNotFoundError("data/best.pt가 없습니다. 블록으로 학습한 가중치를 넣으세요.")
        _model = YOLO(C.WEIGHTS)
        print("모델 클래스:", _model.names)
    r = _model.predict(frame, conf=C.CONF, verbose=False)[0]
    found = []
    if r.obb is not None:                               # 회전 박스(OBB) 모델: 꼭짓점이 바로 나옴
        for pts, cls, cf in zip(r.obb.xyxyxyxy, r.obb.cls, r.obb.conf):
            corners = pts.cpu().numpy().reshape(4, 2)
            u, v = corners.mean(axis=0)
            found.append({"color": _model.names[int(cls)], "u": float(u), "v": float(v),
                          "corners": corners, "conf": float(cf)})
    else:                                               # 일반 박스 모델: 각도는 믿을 수 없음 (항상 0도 근처)
        for box, cls, cf in zip(r.boxes.xyxy, r.boxes.cls, r.boxes.conf):
            x1, y1, x2, y2 = box.tolist()
            corners = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], np.float32)
            found.append({"color": _model.names[int(cls)], "u": (x1 + x2) / 2, "v": (y1 + y2) / 2,
                          "corners": corners, "conf": float(cf)})
    return found


def detect(frame):
    return detect_yolo(frame) if C.BACKEND == "yolo" else detect_hsv(frame)
