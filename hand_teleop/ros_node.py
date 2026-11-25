"""인식 노드: 웹캠 → HandTracker → /hand/state (hand_teleop_msgs/HandState).

rclpy는 이 파일에서만 import한다. 손 좌표 로직은 tracker.py(순수 Python)에 있다.

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    .venv-ros/bin/python -m hand_teleop.ros_node
    .venv-ros/bin/python -m hand_teleop.ros_node --ros-args -p mode_when_tracking:=move

최소 버전(v0): 제스처 판별이 아직 없어서 mode·gripper는 임시값이다.
- mode: 손이 보이면 `mode_when_tracking` 값, 안 보이면 IDLE.
  기본값 idle. 제어부 연동 시험에는 move로 켜면 "손 따라 움직임"을 확인할 수 있다.
- min_confidence: MediaPipe의 손 검출·존재·추적 임계값. 이보다 낮은 손은 MediaPipe가 버려서
  hand_detected=false(→ IDLE)가 된다. 메시지의 confidence(좌우 판별 점수)는 항상 0.5 이상이라
  그 값과 비교하는 방식으로는 일시정지가 걸리지 않는다.
- gripper: 항상 OPEN.

필터: `filter` 파라미터로 none / ma / deadzone / oneeuro 중 선택 (기본 none = 원본 좌표).
    .venv-ros/bin/python -m hand_teleop.ros_node --ros-args -p filter:=oneeuro -p oneeuro_beta:=10.0
- 손을 놓치면 필터를 초기화한다. 다시 잡힌 첫 프레임은 원본 그대로 나간다.
- 한 프레임에 max_jump(정규화 단위)보다 멀리 튀면 다른 손으로 넘어간 것으로 보고
  그 프레임을 IDLE로 내고 필터를 초기화한다. 제어부는 IDLE → MOVE에서 기준점을 새로 잡으므로 로봇이 튀지 않는다.
- 필터 비교용 녹화는 filter:=none으로 원본 좌표를 남긴다.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from hand_teleop_msgs.msg import HandState
from hand_teleop.camera import actual_settings, open_camera
from hand_teleop.filters import make_filter
from hand_teleop.tracker import HandTracker

MODES = {"idle": HandState.MODE_IDLE, "move": HandState.MODE_MOVE,
         "height": HandState.MODE_HEIGHT, "hold": HandState.MODE_HOLD}


class HandStateNode(Node):
    def __init__(self):
        super().__init__("hand_state_node")
        p = self.declare_parameter
        self.cam = p("cam", 0).value
        self.width = p("width", 640).value
        self.height = p("height", 480).value
        self.fps_req = p("fps", 30).value
        self.min_conf = p("min_confidence", 0.5).value
        mode_name = p("mode_when_tracking", "idle").value
        if mode_name not in MODES:
            raise ValueError(f"mode_when_tracking must be one of {list(MODES)}, got {mode_name!r}")
        self.mode_tracking = MODES[mode_name]

        filter_name = p("filter", "none").value
        filter_params = {
            "ma": {"window": p("ma_window", 5).value},
            "deadzone": {"radius": p("deadzone_radius", 0.003).value},
            "oneeuro": {"min_cutoff": p("oneeuro_min_cutoff", 1.0).value,
                        "beta": p("oneeuro_beta", 10.0).value,
                        "d_cutoff": p("oneeuro_d_cutoff", 1.0).value},
        }.get(filter_name, {})
        self.filter = make_filter(filter_name, **filter_params)
        # 30fps에서 0.1/프레임 = 초당 영상 너비 3배. 정상적인 손 이동보다 훨씬 빠르다
        self.max_jump = p("max_jump", 0.1).value
        self.raw_prev: tuple[float, float] | None = None  # 직전 프레임 원본 기준점 (튐 판정용)

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(HandState, "/hand/state", qos)

        self.cap = open_camera(self.cam, self.width, self.height, self.fps_req)
        self.tracker = HandTracker(min_confidence=self.min_conf)

        self.last_x = self.last_y = 0.0  # 손이 없을 때 유지할 마지막 값
        self.t0 = time.perf_counter()
        self.n = self.seen = self.read_fail = self.jumps = 0
        self.proc_ms_sum = 0.0
        self.win_t, self.win_n, self.win_proc = self.t0, 0, 0.0
        self.get_logger().info(
            f"cam={self.cam} requested {self.width}x{self.height}@{self.fps_req}, "
            f"actual {actual_settings(self.cap)}, min_conf={self.min_conf}, "
            f"mode_when_tracking={mode_name}, filter={filter_name} {filter_params}, max_jump={self.max_jump}")

    def step(self) -> bool:
        ok, frame = self.cap.read()
        stamp = self.get_clock().now()  # 프레임 획득 직후. 드라이버 버퍼 지연은 240fps 촬영으로 따로 잰다
        if not ok:
            # 발행하지 않는다: 제어부가 0.2초 끊김으로 "인식부 이상"을 감지해야 한다.
            # 카메라가 빠졌을 때 CPU를 100% 쓰며 헛도는 것을 막으려고 잠깐 쉰다.
            self.read_fail += 1
            self.get_logger().warn("camera read failed", throttle_duration_sec=2.0)
            time.sleep(0.01)
            return False
        t_proc = time.perf_counter()
        hand = self.tracker.process(frame, int((t_proc - self.t0) * 1000))

        msg = HandState()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = "camera"
        msg.gripper = HandState.GRIPPER_OPEN
        if hand is not None:
            jumped = (self.raw_prev is not None
                      and math.hypot(hand.x - self.raw_prev[0], hand.y - self.raw_prev[1]) > self.max_jump)
            self.raw_prev = (hand.x, hand.y)
            if jumped:
                self.jumps += 1
                self.filter.reset()
            t = stamp.nanoseconds * 1e-9
            self.last_x, self.last_y = self.filter.update(t, hand.x, hand.y)
            msg.hand_detected = True
            msg.handedness = hand.handedness
            msg.confidence = hand.confidence
            msg.mode = HandState.MODE_IDLE if jumped else self.mode_tracking
        else:
            if self.raw_prev is not None:  # 손을 놓친 첫 프레임: 다시 잡히면 필터를 새로 시작
                self.filter.reset()
                self.raw_prev = None
            msg.hand_detected = False
            msg.handedness = HandState.HANDEDNESS_UNKNOWN
            msg.confidence = 0.0
            msg.mode = HandState.MODE_IDLE
        msg.x, msg.y = self.last_x, self.last_y
        self.pub.publish(msg)

        proc_ms = (time.perf_counter() - t_proc) * 1000
        self.n += 1
        self.seen += hand is not None
        self.proc_ms_sum += proc_ms
        self.win_n += 1
        self.win_proc += proc_ms
        now = time.perf_counter()
        if now - self.win_t >= 5.0:
            self.get_logger().info(
                f"{self.win_n / (now - self.win_t):.1f} FPS, process {self.win_proc / self.win_n:.1f} ms/frame, "
                f"hand {'yes' if hand else 'no'}")
            self.win_t, self.win_n, self.win_proc = now, 0, 0.0
        return True

    def summary(self) -> str:
        dt = time.perf_counter() - self.t0
        n = max(self.n, 1)
        return (f"{self.n} frames in {dt:.1f}s = {self.n / dt:.1f} FPS avg, "
                f"process {self.proc_ms_sum / n:.1f} ms/frame, hand seen {self.seen}/{self.n}, "
                f"read failures {self.read_fail}, jumps {self.jumps}")

    def close(self):
        self.cap.release()
        self.tracker.close()


def main(args=None):
    rclpy.init(args=args)
    node = HandStateNode()
    try:
        # 카메라 read()가 프레임 주기를 정하므로 타이머 대신 직접 루프를 돈다. 파라미터 등 콜백은 spin_once로 처리.
        while rclpy.ok():
            node.step()
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        print(node.summary(), file=sys.stderr)
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
