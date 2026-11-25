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

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from hand_teleop.camera import actual_settings, open_camera  # noqa: E402

cam = int(sys.argv[1]) if len(sys.argv) > 1 else 0
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 5
model = Path(__file__).resolve().parent.parent / "models" / "hand_landmarker.task"

cap = open_camera(cam)

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
            time.sleep(0.01)
            continue
        img = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        t1 = time.perf_counter()
        res = lm.detect_for_video(img, int((t1 - t0) * 1000))
        mp_ms += (time.perf_counter() - t1) * 1000
        frames += 1
        hands += bool(res.hand_landmarks)
    dt = time.perf_counter() - t0
settings = actual_settings(cap)
cap.release()

if frames == 0:
    sys.exit(f"camera {cam}: opened but no frame read in {secs:.0f}s")
print(f"camera {cam}: {settings}, {frames / dt:.1f} FPS, "
      f"MediaPipe {mp_ms / max(frames, 1):.1f} ms/frame, hand seen in {hands}/{frames} frames")
