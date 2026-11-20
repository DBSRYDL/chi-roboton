# arm_scan.py  -  모터가 응답하는지 진단 (읽기만 함. 아무 값도 쓰지 않음)
# 1) 시리얼 포트 목록  2) 각 USB 포트에서 모든 통신 속도로 모터를 찾아봄
#   python tools\arm_scan.py          USB 포트 전부 검사
#   python tools\arm_scan.py COM5     지정한 포트만
import _path  # noqa
import sys
from serial.tools import list_ports

ports = [p for p in list_ports.comports() if "Bluetooth" not in p.description]
print("[USB 시리얼 포트]")
for p in ports:
    print(f"  {p.device}  {p.description}")
if not ports:
    print("  없음 -> USB 케이블이 빠졌거나 드라이버 문제")
    raise SystemExit

targets = sys.argv[1:] or [p.device for p in ports]
from lerobot.motors.feetech import FeetechMotorsBus

for port in targets:
    print(f"\n[{port}] 모든 통신 속도로 모터 찾는 중 (10~20초)...")
    try:
        found = FeetechMotorsBus.scan_port(port)
    except Exception as e:
        print(f"  포트를 열 수 없음: {type(e).__name__}: {e}")
        print("  -> 다른 프로그램(다른 터미널의 스크립트, LeRobot 텔레옵 등)이 이 포트를 쓰고 있는지 확인")
        continue
    if not found:
        print("  어떤 속도에서도 모터가 응답하지 않음")
        print("  -> 모터 전원(어댑터) 문제일 가능성이 가장 큼. USB만으로는 모터가 켜지지 않음")
    for baud, ids in found.items():
        print(f"  속도 {baud}: 모터 ID {sorted(ids)}")
        if baud != 1_000_000:
            print("  -> 정상 속도(1000000)가 아님. 모터 설정이 바뀐 것")
        missing = sorted(set(range(1, 7)) - set(ids))
        if missing:
            print(f"  -> 없는 ID: {missing}. 그 모터 쪽 케이블을 확인")
