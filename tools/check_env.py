"""환경 점검: 웹캠 + MediaPipe HandLandmarker가 돌아가는지, FPS가 얼마인지 잰다.

    python tools/check_env.py [카메라번호] [초]
영상은 저장하지 않는다(9장 데이터 규칙).
"""
import sys
import time
from pathlib import Path

import cv2
import mediapipe as mp
from mediapipe.tasks.python import BaseOptions, vision

cam = int(sys.argv[1]) if len(sys.argv) > 1 else 0
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 5
model = Path(__file__).resolve().parent.parent / "models" / "hand_landmarker.task"

cap = cv2.VideoCapture(cam, cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)
assert cap.isOpened(), f"camera {cam} not opened"

opts = vision.HandLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=str(model)),
    running_mode=vision.RunningMode.VIDEO,
    num_hands=2,
)
frames = hands = 0
mp_ms = 0.0
with vision.HandLandmarker.create_from_options(opts) as lm:
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < secs:
        ok, bgr = cap.read()
        if not ok:
            continue
        img = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        t1 = time.perf_counter()
        res = lm.detect_for_video(img, int((t1 - t0) * 1000))
        mp_ms += (time.perf_counter() - t1) * 1000
        frames += 1
        hands += bool(res.hand_landmarks)
    dt = time.perf_counter() - t0
cap.release()

print(f"camera {cam}: {bgr.shape[1]}x{bgr.shape[0]}, {frames / dt:.1f} FPS, "
      f"MediaPipe {mp_ms / max(frames, 1):.1f} ms/frame, hand seen in {hands}/{frames} frames")
