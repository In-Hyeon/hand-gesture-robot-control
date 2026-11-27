"""웹캠 열기. ros_node, capture, check_env가 같은 설정을 쓰도록 한곳에 둔다."""
import sys

import cv2


def open_camera(cam: int = 0, width: int = 640, height: int = 480, fps: int = 30) -> cv2.VideoCapture:
    """웹캠을 열고 해상도·FPS를 요청한다. 실제 적용값은 actual_settings()로 확인.

    BUFFERSIZE=2: 드라이버 버퍼(OpenCV V4L2 기본 4)에 오래된 프레임이 쌓이는 양을 줄여 획득 지연을 줄인다.
    처리가 한 번 늦어지면 기본값에서는 최대 4프레임(30fps 기준 ~130ms) 묵은 영상을 처리하게 된다.
    1로 하면 드라이버가 다음 프레임을 받을 자리가 없어 30 → 20 FPS로 떨어진다(ASUS 내장 웹캠 실측, 2026-11-26).
    """
    cap = cv2.VideoCapture(cam, cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"camera {cam} not opened")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
    return cap


def actual_settings(cap: cv2.VideoCapture) -> str:
    """드라이버가 실제로 받아들인 해상도·FPS (요청값과 다를 수 있다)."""
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    return f"{w}x{h}@{cap.get(cv2.CAP_PROP_FPS):.0f}"
