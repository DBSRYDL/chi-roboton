# chi-roboton

2026-2 CHIRO × 현대모비스 로보톤 (자율 Pick & Place) 준비 코드.
카메라로 블록을 찾아 로봇 기준 좌표(mm)와 각도를 내고, SO-101 로봇팔을 그 좌표로 보냅니다.

## 현재 상태 (11/19)

| 부분 | 상태 |
|---|---|
| 블록 인식 (HSV 마스크, 붙은 블록 분리) | 실물 카메라로 확인. 나무색만 색 범위를 맞춤 |
| 픽셀 → mm → 로봇 좌표 | 실물로 확인 (격자와 대조) |
| 좌표 → 관절각 (IK, `so101_ik.py`) | 시뮬에서 검증된 코드. 비전 출력과 좌표계 일치 |
| 관절각 → 실물 모터 (`arm.py`) | **작성만 함. 실물에서 미검증** |
| YOLO | 스위치만 있음. 블록 사진으로 학습 필요 |

## 비전 → 모션 인터페이스

```python
from vision import get_blocks        # 카메라 없이 개발할 때: from fake_vision import get_blocks
get_blocks()
# [{"color": "wood", "x": 213.0, "y": 49.0, "angle": 86.0, "conf": 0.95}, ...]
```

- `x`, `y`: mm. 원점 = 로봇 회전축, x = 로봇 정면, y = 로봇 왼쪽. 블록 윗면 중심
- `angle`: 블록 변의 방향, 로봇 x축 기준 0~90도
- `color`: wood / red / green / blue
- 가까운 블록부터 정렬. 회전축에서 90~250mm 밖은 제외. 없으면 빈 리스트
- 겹치거나 기울어진 블록은 일부러 내보내지 않음
- **팔이 카메라 화면 밖에 있을 때만 호출**

## 파일

| 파일 | 내용 |
|---|---|
| `config.py` | 모든 설정값. 값은 여기서만 바꿈 |
| `vision.py` | `get_blocks()` |
| `fake_vision.py` | 같은 형식의 가짜 값 |
| `detector.py` | 프레임에서 블록 찾기 (HSV / YOLO 전환) |
| `calib.py` | 픽셀 → 작업판 mm → 로봇 좌표, 블록 각도 |
| `camera.py` | 카메라 열기 |
| `so101_ik.py` | FK / IK (모션 담당의 `kinematics.py` 복사본) |
| `arm.py` | 실물 로봇팔: `move_to`, `grip_open`, `grip_close` |
| `marker_board_A4.pdf` | 보정용 마커 판. A4 가로, 100% 크기로 인쇄 |

`tools/` (직접 실행)

| 파일 | 내용 |
|---|---|
| `find_camera.py` | 카메라 번호 찾기 |
| `hsv_tuner.py` | 색 범위 찾기 |
| `calib_homography.py` | 마커 판으로 `data/H.npy` 저장 |
| `view.py` | 인식 결과 실시간 확인 |
| `capture.py` | 학습용 사진 촬영 |
| `dry_run_ik.py` | 로봇 없이 블록 좌표 → 관절각 확인 |
| `arm_probe.py` | LeRobot 버전, 포트, 캘리브레이션 파일 확인 |
| `arm_read.py` | 토크를 끄고 관절 값 읽기 |
| `arm_calib.py` | 관절 영점·방향 매핑 구하기 |
| `arm_hover.py` | 블록 위까지만 이동 (첫 통합 테스트) |

## 세팅 순서 (자리를 옮기면 처음부터)

1. 카메라, 마커 판, 로봇을 테이프로 고정
2. `python tools\find_camera.py` → `CAM_INDEX`
3. `python tools\hsv_tuner.py` → `HSV_RANGES`
4. `python tools\calib_homography.py` → `s`로 저장
5. 로봇 회전축 위치를 자로 재서 `ROBOT_BASE`, `ROBOT_THETA_DEG`
6. `python tools\view.py` → 격자와 좌표 대조 (목표 오차 3mm)

로봇팔은 `arm_probe` → `arm_read` → `arm_calib` → `arm_hover` 순서. `ARM_MAPPING_CHECKED`가 `False`이면 팔이 움직이지 않습니다.

## 올리지 않는 것

`data/H.npy`(자리마다 다름), 사진, 모델 가중치는 `.gitignore`로 제외했습니다.
