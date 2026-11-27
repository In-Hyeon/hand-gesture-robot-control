"""손 좌표 모듈: 웹캠 프레임 → MediaPipe 랜드마크 21개 → 추적 중인 손 하나의 기준점.

ROS 없이 돌아가는 순수 Python. ROS 노드는 이 모듈을 감싸기만 한다.
좌표 규칙은 hand_teleop_msgs/README.md 참고 (좌우 반전 안 한 영상, 두 축 모두 영상 너비로 정규화).
"""
from dataclasses import dataclass
from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks.python import BaseOptions, vision

MODEL = Path(__file__).resolve().parent.parent / "models" / "hand_landmarker.task"
BASE_IDX = (0, 5, 17)  # 손목, 검지 MCP, 새끼 MCP: 손가락을 움직여도 덜 흔들리는 점들
LEFT, RIGHT = 1, 2      # HandState.HANDEDNESS_*


@dataclass
class Hand:
    x: float                # 기준점, 영상 너비로 정규화
    y: float
    handedness: int         # LEFT / RIGHT
    confidence: float       # 0~1
    landmarks: np.ndarray   # (21, 3) MediaPipe 원본 정규화 좌표. 제스처 판별용(필터 안 거침)


def base_point(lm: np.ndarray, w: int, h: int) -> tuple[float, float]:
    """랜드마크(21,3, MediaPipe 정규화) → 기준점 (x, y), 두 축 모두 너비 기준."""
    p = lm[list(BASE_IDX), :2].mean(axis=0)
    return float(p[0]), float(p[1] * h / w)


def select_hand(hands: list[Hand], prev: Hand | None) -> Hand | None:
    """이전에 추적하던 손이 있으면 좌우가 같고 가장 가까운 손, 없으면 신뢰도가 가장 높은 손."""
    if not hands:
        return None
    if prev is not None:
        same = [c for c in hands if c.handedness == prev.handedness]
        if same:
            return min(same, key=lambda c: (c.x - prev.x) ** 2 + (c.y - prev.y) ** 2)
        # ponytail: 추적하던 손이 사라지고 반대 손만 보이면 그 손으로 넘어간다.
        # 활성화한 손만 따라가야 하는 규칙은 클러치(활성화 제스처)가 생기면 그쪽에서 reset으로 처리.
    return max(hands, key=lambda c: c.confidence)


class HandTracker:
    def __init__(self, num_hands: int = 2, min_confidence: float = 0.5, video_mode: bool = True):
        """video_mode=False면 IMAGE 모드: 프레임마다 독립적으로 검출한다(프레임 간 추적·보정 없음).
        MediaPipe가 VIDEO 모드에서 좌표를 얼마나 다듬는지 비교할 때만 쓴다(tools/mp_smoothing_check.py)."""
        self.video_mode = video_mode
        opts = vision.HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=str(MODEL)),
            running_mode=vision.RunningMode.VIDEO if video_mode else vision.RunningMode.IMAGE,
            num_hands=num_hands,
            min_hand_detection_confidence=min_confidence,
            min_hand_presence_confidence=min_confidence,
            min_tracking_confidence=min_confidence,
        )
        self._lm = vision.HandLandmarker.create_from_options(opts)
        self.prev: Hand | None = None
        self._last_ms = -1

    def reset(self):
        """추적 대상을 잊는다(예: 새로 활성화할 때)."""
        self.prev = None

    def process(self, bgr: np.ndarray, t_ms: int) -> Hand | None:
        """BGR 프레임 1장 → 추적 중인 손. 없으면 None (prev는 유지해서 다시 나타나면 같은 손을 고른다)."""
        t_ms = max(t_ms, self._last_ms + 1)  # VIDEO 모드는 타임스탬프가 단조 증가해야 한다
        self._last_ms = t_ms
        h, w = bgr.shape[:2]
        img = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        res = self._lm.detect_for_video(img, t_ms) if self.video_mode else self._lm.detect(img)

        hands = []
        for lms, cats in zip(res.hand_landmarks, res.handedness):
            lm = np.array([(p.x, p.y, p.z) for p in lms], dtype=np.float32)
            x, y = base_point(lm, w, h)
            # MediaPipe는 입력이 거울 영상(셀카)이라고 가정하고 좌우를 붙인다.
            # 우리는 반전하지 않은 영상을 넣으므로 라벨을 뒤집어야 실제 손과 맞는다.
            side = RIGHT if cats[0].category_name == "Left" else LEFT
            hands.append(Hand(x, y, side, float(cats[0].score), lm))

        hand = select_hand(hands, self.prev)
        if hand is not None:
            self.prev = hand
        return hand

    def close(self):
        self._lm.close()


if __name__ == "__main__":
    # 카메라 없이 도는 자체 점검
    lm = np.zeros((21, 3), np.float32)
    lm[[0, 5, 17], :2] = [[0.4, 0.8], [0.5, 0.6], [0.3, 0.6]]
    x, y = base_point(lm, 640, 480)
    assert abs(x - 0.4) < 1e-6 and abs(y - 2 / 3 * 0.75) < 1e-6, (x, y)

    a = Hand(0.2, 0.3, RIGHT, 0.9, lm)
    b = Hand(0.6, 0.3, RIGHT, 0.95, lm)
    c = Hand(0.25, 0.3, LEFT, 0.99, lm)
    assert select_hand([], None) is None
    assert select_hand([a, b, c], None) is c                      # 처음엔 신뢰도 최고
    assert select_hand([a, b, c], Hand(0.21, 0.3, RIGHT, 1, lm)) is a  # 같은 손 중 가장 가까운 것
    assert select_hand([c], Hand(0.21, 0.3, RIGHT, 1, lm)) is c   # 같은 손이 없으면 남은 손
    print("tracker self-check OK")
