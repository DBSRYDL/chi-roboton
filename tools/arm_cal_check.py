# arm_cal_check.py  -  모터 안에 저장된 캘리브레이션과 파일을 나란히 비교 (읽기만 함. 모터 설정·토크를 건드리지 않음)
#   python tools\arm_cal_check.py                 비교만
#   python tools\arm_cal_check.py save 새id       모터에 저장된 값을 새 캘리브레이션 파일로 저장 (로봇은 그대로)
import _path  # noqa
import sys
import config as C
from arm import make_robot

robot = make_robot()
file_cal = robot.calibration
robot.bus.connect()                       # 포트만 엶. configure()를 부르지 않으므로 아무것도 쓰지 않음
try:
    motor_cal = robot.bus.read_calibration()
finally:
    robot.bus.disconnect(disable_torque=False)

print(f"파일: {robot.calibration_fpath}")
print(f"{'관절':14s} {'':6s} {'homing_offset':>14s} {'range_min':>10s} {'range_max':>10s} {'범위(도)':>9s}")
same = True
for name, m in motor_cal.items():
    f = file_cal.get(name)
    span = (m.range_max - m.range_min) * 360 / 4095
    print(f"{name:14s} {'모터':6s} {m.homing_offset:14d} {m.range_min:10d} {m.range_max:10d} {span:9.0f}")
    if f is None:
        print(f"{'':14s} {'파일':6s} (없음)")
        same = False
        continue
    fspan = (f.range_max - f.range_min) * 360 / 4095
    diff = (f.homing_offset, f.range_min, f.range_max) != (m.homing_offset, m.range_min, m.range_max)
    same &= not diff
    print(f"{'':14s} {'파일':6s} {f.homing_offset:14d} {f.range_min:10d} {f.range_max:10d} {fspan:9.0f}" + ("   <- 다름" if diff else ""))

print("\n결과:", "같음" if same else "다름")
untouched = all(m.range_min == 0 and m.range_max >= 4095 and m.homing_offset == 0 for m in motor_cal.values())
if untouched:
    print("모터 값이 전부 초기값(offset 0, 범위 0~4095)입니다. 이 팔은 캘리브레이션된 적이 없어 보입니다.")

if len(sys.argv) >= 3 and sys.argv[1] == "save":
    new_id = sys.argv[2]
    path = robot.calibration_dir / f"{new_id}.json"
    if path.exists():
        print(f"\n{path} 가 이미 있습니다. 다른 id를 쓰세요.")
    else:
        robot.calibration = motor_cal
        robot._save_calibration(path)
        print(f"\n모터 값을 {path} 로 저장했습니다. config.py의 ARM_ID를 \"{new_id}\" 로 바꾸세요.")
