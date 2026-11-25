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
  ros_node.py              인식 노드: 웹캠 → tracker → /hand/state 발행
tools/
  check_env.py             웹캠 + MediaPipe 동작과 FPS 점검
  capture.py               손 기준점 화면 확인, 좌표 CSV 녹화 (ROS 없이)
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
# 제어부 연동 시험용: 손이 보이면 mode=MOVE로 발행 (제스처 판별이 붙기 전 임시 기능)
.venv-ros/bin/python -m hand_teleop.ros_node --ros-args -p mode_when_tracking:=move

# 4. 다른 터미널에서 확인
ros2 topic echo /hand/state
```

노드 파라미터: `cam`(기본 0), `width`·`height`(640×480), `fps`(30), `min_confidence`(0.5), `mode_when_tracking`(idle / move / height / hold).

ROS 없이 확인할 때:
```bash
.venv-ros/bin/python tools/check_env.py          # 웹캠 + MediaPipe FPS
.venv-ros/bin/python tools/capture.py            # 손 기준점 화면 확인 (q 종료)
.venv-ros/bin/python -m hand_teleop.tracker      # tracker 자체 점검 (카메라 불필요)
```

## 데이터 규칙
- 원본 영상은 저장하지 않고 랜드마크 좌표만 남긴다. rosbag2 녹화에도 영상 토픽은 넣지 않는다.

## 진행 상황
| 상태 | 항목 |
|---|---|
| 완료 | `/hand/state` 메시지 v0.1, 손 좌표 모듈, 인식 노드 최소 버전 (640×480에서 29.7 FPS, 처리 12.3 ms/프레임, 손 미검출 상태 측정) |
| 진행 중 | 떨림 필터 3종(이동평균, 데드존, One Euro)과 필터 교체 구조 |
| 예정 | rosbag2 녹화와 필터 오프라인 비교, 제스처 판별(클러치·높이·그리퍼), 명령 생성 노드, Gazebo 시뮬레이션, OMX-AI 실물 연동 |

인식 노드의 `mode`와 `gripper`는 제스처 판별이 붙기 전까지 임시값이고, `x`, `y`에는 아직 필터가 적용되지 않는다.
