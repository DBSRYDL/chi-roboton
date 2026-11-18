# hsv_tuner.py  -  트랙바로 색 범위 찾기. 블록만 흰색으로 남게 맞춘 뒤 p를 누르면 config.py에 넣을 값을 출력
# p: 출력 / q: 종료
import _path  # noqa
import cv2
import numpy as np
import camera

WIN = "hsv tuner"
cv2.namedWindow(WIN)
for name, val, top in [("H min", 10, 179), ("H max", 30, 179), ("S min", 40, 255),
                       ("S max", 170, 255), ("V min", 90, 255), ("V max", 255, 255)]:
    cv2.createTrackbar(name, WIN, val, top, lambda v: None)

cap = camera.open_camera()
while True:
    ok, frame = cap.read()
    if not ok:
        break
    g = lambda n: cv2.getTrackbarPos(n, WIN)
    lo = (g("H min"), g("S min"), g("V min"))
    hi = (g("H max"), g("S max"), g("V max"))
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lo), np.array(hi))
    small = cv2.resize(np.hstack([frame, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)]), None, fx=0.5, fy=0.5)
    cv2.imshow(WIN, small)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("p"):
        print(f"[({lo}, {hi})],")
    elif key == ord("q"):
        break
cap.release()
cv2.destroyAllWindows()
