# hand-gesture-robot-control
손동작 인식 기반 로봇 매니퓰레이터 제어 시스템 (KETI 멘토링 AI P-실무프로젝트)
Robot manipulator control system driven by real-time hand gesture recognition (KETI-mentored AI P-Project)

웹캠 하나로 손 위치와 손가락 동작을 인식해 소형 로봇팔(ROBOTIS OMX-AI, 5자유도 + 그리퍼)의 끝단 이동과 집게를 제어한다.
목표는 저가 웹캠으로도 안정적으로 조작할 수 있게 만드는 것이다.

## 시스템 구조
```
웹캠 → 손 추적(MediaPipe 랜드마크 21개) → 떨림 필터 → /hand/state ─┬→ 명령 생성(IK) → 로봇 제어 인터페이스 → OMX-AI
        └──────────── 인식부 (ROS 2 노드) ────────────┘          └─ 안전 감시          (시뮬레이션: Gazebo)
```
- 인식부와 제어부는 **`/hand/state` 메시지 하나로만 연결한다.** 이 형식만 맞추면 두 파트를 따로 개발하고 테스트할 수 있다.
- 조작은 클러치 방식이다. 활성화 제스처를 취한 순간의 손 위치를 기준으로, 손을 움직인 만큼 로봇이 움직인다(상대 매핑).
- 단일 웹캠은 깊이를 알 수 없으므로 화면 좌우·상하는 로봇 평면 이동, 높이는 높이 모드에서만 조절한다.
- 판단이 애매하면 정지한다. 손을 놓치거나 신뢰도가 떨어지면 일시정지하고, 다시 움직이려면 활성화 제스처가 필요하다.

## 저장소 구성
```
hand_teleop_msgs/        /hand/state 메시지 정의 (ROS 2 패키지)
  msg/HandState.msg        필드 정의
  README.md                인식부·제어부 역할 분담, 좌표 규칙, 모드 전이
hand_teleop/             인식부 Python 코드
  tracker.py               손 좌표 모듈 (ROS 없는 순수 Python)
  filters.py               떨림 필터: 이동평균, 데드존, One Euro (같은 입력에 바꿔 끼우는 구조)
  camera.py                웹캠 열기 (노드와 도구가 같은 설정 사용)
  clutch.py                재개 보호: 손 소실·튐 뒤에는 활성화 전까지 조작 모드를 내지 않음
  ros_node.py              인식 노드: 웹캠 → tracker → 필터 → /hand/state 발행
tools/
  check_env.py             웹캠 + MediaPipe 동작과 FPS 점검
  capture.py               필터 비교 화면, 원본 좌표 CSV 녹화 (ROS 없이)
  compare_filters.py       녹화 CSV로 필터·파라미터 오프라인 비교 (떨림 감소율, 지연, 5mm 반영률)
  mp_smoothing_check.py    MediaPipe VIDEO 모드와 IMAGE 모드의 정지 떨림 비교 (내부 보정 여부 확인)
  setup_ros_jazzy.sh       ROS 2 Jazzy, rosbag2, .venv-ros, MediaPipe 모델 설치
models/                  MediaPipe 모델 (설치 스크립트가 받음, git 제외)
```

| 구분 | 사용 |
|---|---|
| 언어 | Python 3 (인식부), ROS 2 IDL (`.msg`), Bash (설치) |
| 라이브러리 | MediaPipe 1.2 (Tasks API `HandLandmarker`), OpenCV 4.11, NumPy 1.26, rclpy |
| 환경 | Ubuntu 24.04, ROS 2 Jazzy |

## 핵심 설계
**`/hand/state` (`hand_teleop_msgs/msg/HandState`, v0.1)**
| 필드 | 뜻 |
|---|---|
| `header.stamp` | 카메라 프레임을 얻은 시각 (종단 지연 측정 기준) |
| `hand_detected`, `handedness`, `confidence` | 손 유무, 왼손/오른손, MediaPipe 신뢰도 |
| `x`, `y` | 떨림 필터를 거친 손 기준점. **두 축 모두 영상 너비로 정규화** |
| `mode` | IDLE / MOVE / HEIGHT / HOLD (클러치 상태, 인식부가 결정) |
| `gripper` | OPEN / CLOSE (목표 상태) |

손이 없어도 프레임마다 발행한다. 그래서 메시지가 0.2초 끊기면 "손 없음"이 아니라 "인식부 이상"으로 판단할 수 있다.
자세한 내용은 [`hand_teleop_msgs/README.md`](hand_teleop_msgs/README.md).

**손 좌표 모듈 (`hand_teleop/tracker.py`)**
- 위치 기준점은 손목, 검지 MCP, 새끼 MCP 세 점의 평균이다. 손가락을 움직여도 위치가 덜 흔들린다.
- 손이 여러 개 보이면 추적하던 손과 좌우가 같고 가장 가까운 손을 고른다. 처음에는 신뢰도가 가장 높은 손을 고른다.
- MediaPipe는 거울 영상을 가정하고 좌우 라벨을 붙인다. 반전하지 않은 영상을 넣으므로 라벨을 뒤집어 실제 손과 맞춘다.
- 손 좌표와 필터 코어는 ROS 없이 동작하고, `rclpy`는 노드 파일에서만 import한다. ROS 없이도 테스트할 수 있게 하려는 것이다.

**떨림 필터 (`hand_teleop/filters.py`)**
- 모든 필터가 `update(t, x, y) -> (x, y)`, `reset()` 형태라서 같은 녹화 입력에 바꿔 끼워 비교할 수 있다.
- `ma`(이동평균): 최근 N프레임 평균. `deadzone`: 반경 안의 움직임은 무시하고, 벗어나면 경계까지만 따라간다(출력이 튀지 않음). `oneeuro`: 느릴 때 강하게, 빠를 때 약하게 거른다(x, y에 같은 차단 주파수를 써서 궤적 모양 유지).
- 손을 놓치면 필터를 초기화한다. 한 프레임에 `max_jump`보다 멀리 튀면 다른 손으로 넘어간 것으로 보고 그 프레임을 `IDLE`로 낸다(제어부가 기준점을 새로 잡아 로봇이 튀지 않음).
- 파라미터 기본값은 임시값이다. 녹화 데이터로 비교한 뒤 정한다.

**재개 보호 (`hand_teleop/clutch.py`)**
- 노드는 시작할 때, 그리고 손을 놓치거나 위치가 튀면 "해제" 상태가 된다. 해제 상태에서는 손이 다시 보여도 `IDLE`이다(안전 규칙 "재개는 활성화 제스처로만").
- 손이 보일 때 활성화해야 다시 조작 모드가 나간다. 활성화 제스처가 붙기 전까지는 `/hand/activate` 서비스(`std_srvs/srv/Trigger`)를 쓴다.

## 설치와 실행 (Ubuntu 24.04)
```bash
# 1. ROS 2 Jazzy, rosbag2, .venv-ros(rclpy + MediaPipe), 모델 설치
bash tools/setup_ros_jazzy.sh

# 2. 메시지 패키지 빌드
source /opt/ros/jazzy/setup.bash
colcon build --packages-select hand_teleop_msgs
source install/setup.bash

# 3. 인식 노드 실행 (Ctrl+C로 종료하면 평균 FPS 출력)
.venv-ros/bin/python -m hand_teleop.ros_node
# 제어부 연동 시험용: 활성화 후 손이 보이면 mode=MOVE로 발행 (제스처 판별이 붙기 전 임시 기능)
.venv-ros/bin/python -m hand_teleop.ros_node --ros-args -p mode_when_tracking:=move
ros2 service call /hand/activate std_srvs/srv/Trigger   # 손을 보인 상태에서 활성화 (손을 놓치면 다시 호출)
# 시뮬레이션에서 매번 활성화하기 번거로우면 (실물에서는 쓰지 않음)
.venv-ros/bin/python -m hand_teleop.ros_node --ros-args -p mode_when_tracking:=move -p auto_resume:=true

# 4. 다른 터미널에서 확인
ros2 topic echo /hand/state
```

노드 파라미터
| 파라미터 | 기본값 | 뜻 |
|---|---|---|
| `cam`, `width`, `height`, `fps` | 0, 640, 480, 30 | 웹캠 번호와 요청 해상도·FPS (실제 적용값은 시작 로그에 출력) |
| `min_confidence` | 0.5 | MediaPipe 손 검출·존재·추적 임계값. 이보다 낮으면 손 없음(`IDLE`) |
| `mode_when_tracking` | idle | 활성화 상태에서 손이 보일 때 낼 mode (idle / move / height / hold). 제스처 판별 전 임시 |
| `auto_resume` | false | true면 손 소실·튐 뒤 손이 다시 보이면 바로 재개 (시뮬레이션 시험용, 실물 금지) |
| `filter` | none | none / ma / deadzone / oneeuro. 녹화는 none(원본 좌표)으로 한다 |
| `ma_window` | 5 | 이동평균 프레임 수 |
| `deadzone_radius` | 0.003 | 데드존 반경 (정규화 단위) |
| `oneeuro_min_cutoff`, `oneeuro_beta`, `oneeuro_d_cutoff` | 1.0, 10.0, 1.0 | One Euro 파라미터 |
| `max_jump` | 0.1 | 한 프레임 최대 이동량 (정규화 단위). 넘으면 그 프레임 `IDLE` + 필터 초기화 |

```bash
.venv-ros/bin/python -m hand_teleop.ros_node --ros-args -p filter:=oneeuro -p oneeuro_beta:=10.0
```

ROS 없이 확인할 때:
```bash
.venv-ros/bin/python tools/check_env.py          # 웹캠 + MediaPipe FPS
.venv-ros/bin/python tools/capture.py            # 필터 비교 화면: 원본(빨강)과 필터 출력(초록), 1~4로 필터 전환 (q 종료)
.venv-ros/bin/python -m hand_teleop.tracker      # tracker 자체 점검 (카메라 불필요)
.venv-ros/bin/python -m hand_teleop.filters      # 필터 자체 점검: 같은 가상 입력에 필터 4개 적용 (카메라 불필요)
.venv-ros/bin/python -m hand_teleop.clutch       # 재개 보호 자체 점검 (카메라 불필요)
```

## 필터 비교 (녹화 → 오프라인 비교)
```bash
# 1. 구간별 녹화 (원본 좌표만 저장, 영상 저장 안 함). --guide: 화면 가운데에 동작 안내 (처음 3초는 READY)
.venv-ros/bin/python tools/capture.py --record data/still.csv --secs 23 --guide still   # 손을 가만히
.venv-ros/bin/python tools/capture.py --record data/move.csv  --secs 23 --guide move    # 좌우·상하로 자연스럽게
.venv-ros/bin/python tools/capture.py --record data/fine5.csv --secs 33 --guide fine    # 안내 박자에 맞춰 5mm 왕복

# 2. 비교: 필터·파라미터 조합마다 지표 계산 → data/results/summary.md, all.csv, filters.png
.venv-ros/bin/python tools/compare_filters.py --still data/still.csv --move data/move.csv --fine data/fine5.csv
```
| 지표 | 계산 | 목표 |
|---|---|---|
| 정지 떨림 감소율 | 1초 창마다 2D 표준편차 평균, 원본 대비 (처음 3초 제외) | 70% 이상 |
| 빠른 떨림 감소율 (보조) | 0.5초 이동평균을 뺀 나머지(≈2Hz 이상)의 RMS, 원본 대비. 느린 손 흔들림을 뺀 잡음 성분 | 참고 |
| 필터 추가 지연 | 1ms 격자로 보간한 원본·필터 출력의 상호상관이 최대인 시간차 | 50ms 이내 |
| 5mm 반영률 | 멈춘 구간 사이 이동량을 원본 이동 방향으로 투영한 비율 | 90% 이상 |

세 지표 모두 비율·시간이라 영상 좌표 → mm 배율 없이 계산된다. 지연 50ms 이내와 반영률 90% 이상을 만족하는 조합 중 떨림 감소가 가장 큰 것을 고른다.

## 데이터 규칙
- 원본 영상은 저장하지 않고 랜드마크 좌표만 남긴다. rosbag2 녹화에도 영상 토픽은 넣지 않는다.

## 진행 상황
| 상태 | 항목 |
|---|---|
| 완료 | `/hand/state` 메시지 v0.1, 손 좌표 모듈, 인식 노드 (640×480에서 약 29.5 FPS, 처리 12~15 ms/프레임), 떨림 필터 3종과 교체 구조 |
| 진행 중 | rosbag2 녹화와 필터 오프라인 비교 (정지 떨림 감소율, 교차상관 지연, 5mm 미세 이동 반영률), 파라미터 선정 |
| 예정 | 제스처 판별(클러치·높이·그리퍼), 명령 생성 노드, Gazebo 시뮬레이션, OMX-AI 실물 연동 |

인식 노드의 `mode`와 `gripper`는 제스처 판별이 붙기 전까지 임시값이다.
