# calib_homography.py  -  ArUco 마커 4개로 H를 구해 data/H.npy에 저장
# 왼쪽 클릭: 그 점의 작업판 mm와 로봇 mm 출력 / s: 저장 / q: 종료
# 카메라나 판이 조금이라도 움직였으면 다시 해야 합니다
import _path  # noqa
import os
import cv2
import numpy as np
import config as C
import camera
import calib

detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
                                   cv2.aruco.DetectorParameters())
state = {"H": None}


def on_mouse(event, u, v, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and state["H"] is not None:
        x, y = calib.pixel_to_mm(state["H"], u, v)
        rx, ry = calib.table_to_robot(x, y)
        print(f"픽셀 ({u}, {v}) -> 작업판 ({x:.1f}, {y:.1f}) mm -> 로봇 ({rx:.1f}, {ry:.1f}) mm (바닥 기준)")


WIN = "calib homography"
cv2.namedWindow(WIN)
cv2.setMouseCallback(WIN, on_mouse)
cap = camera.open_camera()
ids_need = sorted(C.MARKER_MM)
while True:
    ok, frame = cap.read()
    if not ok:
        break
    corners, ids, _ = detector.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    centers = {}
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        centers = {int(i): c[0].mean(axis=0) for c, i in zip(corners, ids.flatten())}
    found = [i for i in ids_need if i in centers]
    if len(found) == len(ids_need):
        src = np.array([centers[i] for i in ids_need], np.float32)
        dst = np.array([C.MARKER_MM[i] for i in ids_need], np.float32)
        state["H"], _ = cv2.findHomography(src, dst)
        text, color = "markers OK  (s: save)", (0, 255, 0)
    else:
        text, color = f"markers {len(found)}/{len(ids_need)}", (0, 0, 255)
    cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    cv2.imshow(WIN, frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("s") and state["H"] is not None:
        os.makedirs(C.DATA_DIR, exist_ok=True)
        np.save(C.H_PATH, state["H"])
        print("저장:", C.H_PATH)
    elif key == ord("q"):
        break
cap.release()
cv2.destroyAllWindows()
