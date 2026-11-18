# find_camera.py  -  0~4번 카메라를 차례로 열어 보여 줌. 외부 카메라 번호를 config.py의 CAM_INDEX에 적기
# 아무 키: 다음 카메라 / q: 종료
import cv2

for i in range(5):
    cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
    ok, frame = cap.read()
    if not ok:
        print(f"{i}번: 없음")
        cap.release()
        continue
    print(f"{i}번: 열림 {frame.shape[1]}x{frame.shape[0]}")
    cv2.putText(frame, f"CAM_INDEX = {i}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)
    cv2.imshow("find camera", frame)
    key = cv2.waitKey(0) & 0xFF
    cap.release()
    if key == ord("q"):
        break
cv2.destroyAllWindows()
