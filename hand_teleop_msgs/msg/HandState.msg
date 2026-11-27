# /hand/state : 인식부 → 제어부 유일한 연결. 카메라 프레임마다 1번 발행(~30Hz).
# 손이 안 보여도 hand_detected=false로 계속 발행한다.
# 그래서 0.2초 동안 메시지가 없으면 "손 없음"이 아니라 "인식부 고장"이다.

# stamp = 카메라 프레임 획득 시각(발행 시각 아님). 종단 지연 측정 기준.
# frame_id = "camera"
std_msgs/Header header

# --- 손 ---
bool hand_detected
uint8 HANDEDNESS_UNKNOWN = 0
uint8 HANDEDNESS_LEFT = 1
uint8 HANDEDNESS_RIGHT = 2
uint8 handedness
# 0.5~1. MediaPipe 좌우 판별(handedness) 점수. 손이 없으면 0.
# "왼손/오른손이 얼마나 확실한가"이지 "손이 있다는 확신"이 아니다. 둘 중 하나를 고른 점수라 항상 0.5 이상이다.
# 정지 판단에 쓰지 말 것: 신뢰도 낮은 손은 인식부가 이미 hand_detected=false + mode=IDLE로 낸다.
# (인식 노드 min_confidence 파라미터 = MediaPipe 검출·존재·추적 임계값)
float32 confidence

# --- 위치 (떨림 필터 적용) ---
# 손목과 손바닥 기저부(검지·새끼 MCP) 평균점. 좌우 반전하지 않은 카메라 영상 기준.
# x: 오른쪽 +, y: 아래쪽 +. 두 축 모두 영상 "너비"로 나눈 값이다(x 0~1, y 0~H/W).
# 축마다 배율이 같아서, 손으로 원을 그리면 원으로 전달된다.
# hand_detected=false면 마지막 값을 유지한다(사용하지 말 것).
# 손을 놓쳤다가 다시 잡으면 필터를 새로 시작하므로, 다시 잡힌 첫 프레임은 원본 위치다.
# 필터 전 원본 위치는 디버그 토픽 /hand/raw (geometry_msgs/PointStamped, 같은 stamp·단위)로 따로 나간다.
float32 x
float32 y

# --- 제어 모드 (클러치 상태 기계는 인식부가 소유) ---
# IDLE   : 조작 안 함. 대기, 그리고 손 소실이나 신뢰도 저하로 인한 일시정지도 여기. 재개는 활성화 제스처로만.
#          위치가 한 프레임에 비정상적으로 튀면(다른 손으로 넘어감 등) 그 프레임도 IDLE이다.
#          제어부가 다음 MOVE에서 기준점을 새로 잡으므로 로봇이 튀지 않는다.
# MOVE   : x,y 변화량 → 로봇 평면(좌우·전후) 이동
# HEIGHT : y 변화량 → 로봇 높이. x는 무시
# HOLD   : 현재 위치 고정. 정지 제스처 후보(0.5초 확인 중), 그리퍼 전환 중, 판단 애매
# 제어부는 IDLE/HOLD → MOVE/HEIGHT로 들어가는 순간, 그리고 MOVE ↔ HEIGHT로 바뀌는 순간마다
# (손 x,y, 로봇 위치)를 새 기준점으로 잡는다(클러치 기준 상대 매핑).
uint8 MODE_IDLE = 0
uint8 MODE_MOVE = 1
uint8 MODE_HEIGHT = 2
uint8 MODE_HOLD = 3
uint8 mode

# --- 그리퍼 (목표 상태) ---
# 제스처가 5프레임 연속 같을 때만 바뀐다. 모드와 상관없이 마지막 목표 유지.
uint8 GRIPPER_OPEN = 0
uint8 GRIPPER_CLOSE = 1
uint8 gripper
