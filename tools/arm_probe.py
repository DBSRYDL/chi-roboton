# arm_probe.py  -  로봇을 건드리지 않고 환경만 확인 (LeRobot 버전, 포트, 캘리브레이션 파일)
# lerobot이 설치된 환경에서 실행:  conda activate lerobot  ->  python tools\arm_probe.py
import os
import sys
import glob

print("python:", sys.version.split()[0], sys.executable)
try:
    import lerobot
    print("lerobot:", getattr(lerobot, "__version__", "?"), os.path.dirname(lerobot.__file__))
except ImportError:
    print("lerobot: 이 환경에 없음  ->  conda activate lerobot 후 다시 실행")
    raise SystemExit

for mod in ("lerobot.robots.so101_follower", "lerobot.robots.so_follower", "lerobot.common.robots.so101_follower"):
    try:
        m = __import__(mod, fromlist=["x"])
        names = [n for n in dir(m) if "Follower" in n]
        print(f"  import {mod}: OK  {names}")
    except Exception as e:
        print(f"  import {mod}: 실패 ({type(e).__name__})")

for mod in ("cv2", "numpy", "ultralytics", "serial"):
    try:
        m = __import__(mod)
        print(f"{mod}: {getattr(m, '__version__', 'OK')}")
    except ImportError:
        print(f"{mod}: 없음")

print("\n[시리얼 포트]  로봇 USB를 뽑았다 꽂으며 두 번 실행하면 어느 것이 로봇인지 알 수 있음")
try:
    from serial.tools import list_ports
    for p in list_ports.comports():
        print(f"  {p.device}  {p.description}")
except ImportError:
    print("  pyserial 없음")

print("\n[캘리브레이션 파일]  파일 이름(확장자 제외)이 ARM_ID")
home = os.environ.get("HF_LEROBOT_CALIBRATION") or os.path.join(os.path.expanduser("~"), ".cache", "huggingface", "lerobot", "calibration")
files = glob.glob(os.path.join(home, "**", "*.json"), recursive=True)
for f in files:
    print("  ", os.path.relpath(f, home))
if not files:
    print("   없음 ->", home, " (lerobot-calibrate로 follower 팔 캘리브레이션이 필요)")
