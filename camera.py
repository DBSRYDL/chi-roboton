# camera.py  -  카메라 열기 / 프레임 가져오기
import cv2
import config as C


def open_camera(index=None):
    cap = cv2.VideoCapture(C.CAM_INDEX if index is None else index, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))   # 이 카메라는 MJPG여야 1280x720
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, C.W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, C.H_IMG)
    if not cap.isOpened():
        raise RuntimeError(f"카메라 {C.CAM_INDEX}번을 열 수 없습니다. config.py의 CAM_INDEX를 확인하세요.")
    return cap


def grab(cap, n=1, flush=3):
    """버퍼에 쌓인 옛 프레임 flush장을 버린 뒤 n장을 돌려줌"""
    for _ in range(flush):
        cap.read()
    frames = []
    for _ in range(n):
        ok, frame = cap.read()
        if ok:
            frames.append(frame)
    return frames
