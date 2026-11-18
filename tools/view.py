# view.py  -  검출 결과를 실시간으로 보기 (디버깅용)
# H.npy가 있으면 로봇 좌표 mm와 각도, 없으면 픽셀 좌표만 표시
# 스페이스: get_blocks()와 같은 방식(여러 프레임 묶기)으로 한 번 출력 / q: 종료
import _path  # noqa
import cv2
import numpy as np
import config as C
import camera
import calib
import vision
import detector
from detector import detect

try:
    H = calib.load_H()
except FileNotFoundError as e:
    print(e)
    H = None

cap = camera.open_camera()
while True:
    ok, frame = cap.read()
    if not ok:
        break
    raw = frame.copy()
    dets = vision.frame_blocks(raw, H) if H is not None else detect(raw)
    for d in dets:
        cv2.polylines(frame, [np.int32(d["corners"])], True, (0, 255, 0), 2)
        cv2.circle(frame, (int(d["u"]), int(d["v"])), 4, (0, 0, 255), -1)
        if H is not None:
            r = np.hypot(d["x"], d["y"])
            inside = C.R_MIN <= r <= C.R_MAX
            label = f'{d["color"]} ({d["x"]:.0f},{d["y"]:.0f}) {d["angle"]:.0f}deg'
            color = (0, 255, 0) if inside else (0, 165, 255)      # 주황 = 가동 범위 밖
        else:
            label, color = f'{d["color"]} ({d["u"]:.0f},{d["v"]:.0f})px', (0, 255, 0)
        cv2.putText(frame, label, (int(d["u"]) + 8, int(d["v"]) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    for corners, why in detector.rejected:               # 빨강 = 버린 후보와 이유 (small/big + 면적)
        cv2.polylines(frame, [np.int32(corners)], True, (0, 0, 255), 1)
        cv2.putText(frame, why, tuple(np.int32(corners[0])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    for d in dets:                                       # 잡힌 블록의 면적(px)
        cv2.putText(frame, f'{cv2.contourArea(np.float32(d["corners"])):.0f}', (int(d["u"]) + 8, int(d["v"]) + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
    if detector.masks:
        m = np.maximum.reduce(list(detector.masks.values()))
        cv2.imshow("mask", cv2.resize(m, None, fx=0.5, fy=0.5))
    cv2.putText(frame, f"{C.BACKEND}  n={len(dets)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.imshow("view", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord(" ") and H is not None:
        frames = camera.grab(cap, C.N_FRAMES)
        print("--- get_blocks ---")
        for b in vision.merge([vision.frame_blocks(f, H) for f in frames]):
            print(b)
    elif key == ord("q"):
        break
cap.release()
cv2.destroyAllWindows()
