# capture.py  -  학습용 사진 찍기. 스페이스: 저장 / q: 종료
import _path  # noqa
import os
import time
import cv2
import config as C
import camera

os.makedirs(C.PHOTO_DIR, exist_ok=True)
cap = camera.open_camera()
count = len([f for f in os.listdir(C.PHOTO_DIR) if f.endswith(".jpg")])
while True:
    ok, frame = cap.read()
    if not ok:
        break
    view = frame.copy()
    cv2.putText(view, f"saved: {count}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)
    cv2.imshow("capture", view)
    key = cv2.waitKey(1) & 0xFF
    if key == ord(" "):
        name = f"{C.TEAM}_{time.strftime('%m%d_%H%M%S')}_{count:03d}.jpg"
        ok, buf = cv2.imencode(".jpg", frame)
        if ok:
            buf.tofile(os.path.join(C.PHOTO_DIR, name))   # 한글 경로에서도 저장되게
            count += 1
            print("저장:", name)
    elif key == ord("q"):
        break
cap.release()
cv2.destroyAllWindows()
