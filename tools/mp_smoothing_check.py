"""MediaPipe가 VIDEO 모드에서 손 좌표를 내부적으로 다듬는지 확인한다.

같은 웹캠 프레임을 VIDEO 모드(프레임 간 추적, 현재 노드가 쓰는 방식)와 IMAGE 모드(프레임마다 독립 검출)로
동시에 처리하고, 손을 가만히 들고 있는 동안의 정지 떨림을 비교한다. 영상은 저장하지 않는다.

    python tools/mp_smoothing_check.py            # 20초. 처음 3초는 자리 잡기(계산에서 뺌)
    python tools/mp_smoothing_check.py --save data/mp_check.csv   # 두 모드 좌표를 CSV로 남김

화면: 빨강 = IMAGE 모드 기준점, 초록 = VIDEO 모드 기준점 (거울 반전 표시)
IMAGE 모드 떨림이 VIDEO 모드보다 훨씬 크면 MediaPipe가 이미 필터 역할을 하고 있다는 뜻이다.
"""
import argparse
import csv
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from hand_teleop.camera import actual_settings, open_camera  # noqa: E402
from hand_teleop.tracker import HandTracker  # noqa: E402
from compare_filters import jitter, jitter_hf  # noqa: E402  (같은 tools/ 폴더)

ap = argparse.ArgumentParser()
ap.add_argument("--cam", type=int, default=0)
ap.add_argument("--secs", type=float, default=20)
ap.add_argument("--save", type=Path)
args = ap.parse_args()

cap = open_camera(args.cam)
print(f"camera {args.cam}: {actual_settings(cap)}")
video, image = HandTracker(), HandTracker(video_mode=False)
rows = []  # t, vx, vy, ix, iy (손이 없으면 nan)
ms = {"video": [], "image": []}
t0 = time.perf_counter()
try:
    while True:
        ok, frame = cap.read()
        t = time.perf_counter() - t0
        if t > args.secs:
            break
        if not ok:
            time.sleep(0.01)
            continue
        a = time.perf_counter()
        hv = video.process(frame, int(t * 1000))
        b = time.perf_counter()
        hi = image.process(frame, int(t * 1000))
        ms["video"].append((b - a) * 1000)
        ms["image"].append((time.perf_counter() - b) * 1000)
        nan = float("nan")
        rows.append((t, hv.x if hv else nan, hv.y if hv else nan, hi.x if hi else nan, hi.y if hi else nan))

        h, w = frame.shape[:2]
        for hand, color in ((hi, (0, 0, 255)), (hv, (0, 200, 0))):
            if hand:
                cv2.circle(frame, (int(hand.x * w), int(hand.y * w)), 7, color, -1)
        frame = cv2.flip(frame, 1)
        msg = "READY: put your hand up" if t < 3 else f"HOLD STILL  {args.secs - t:4.1f}s"
        for text, y, color in ((msg, 30, (255, 255, 255)), ("red = IMAGE mode, green = VIDEO mode", 58, (200, 200, 200))):
            cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 1, cv2.LINE_AA)
        cv2.imshow("mediapipe smoothing check", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
finally:
    cap.release()
    video.close()
    image.close()
    cv2.destroyAllWindows()

a = np.array(rows)
t = a[:, 0]
if args.save:
    args.save.parent.mkdir(parents=True, exist_ok=True)
    with open(args.save, "w", newline="") as f:
        csv.writer(f).writerows([("t", "video_x", "video_y", "image_x", "image_y"), *rows])
    print(f"saved {args.save}")

W = 640
print(f"\n{len(t)} frames, {len(t) / t[-1]:.1f} FPS (두 모드를 같이 돌려서 평소보다 느림)")
print(f"처리 시간: VIDEO {np.mean(ms['video']):.1f} ms, IMAGE {np.mean(ms['image']):.1f} ms")
res = {}
for name, xi, yi in (("VIDEO", 1, 2), ("IMAGE", 3, 4)):
    det = ~np.isnan(a[:, xi])
    j, jh = jitter(t, det, a[:, xi], a[:, yi]), jitter_hf(t, det, a[:, xi], a[:, yi])
    res[name] = (j, jh)
    print(f"{name:5s}: 손 검출 {det.mean():.0%}, 정지 떨림 {j * W:.3f}px, 빠른 떨림(≈2Hz 이상) {jh * W:.3f}px")
(jv, hv_), (ji, hi_) = res["VIDEO"], res["IMAGE"]
if ji > 0 and hi_ > 0:
    print(f"\nVIDEO 모드가 IMAGE 모드 대비 줄인 양: 정지 떨림 {1 - jv / ji:.0%}, 빠른 떨림 {1 - hv_ / hi_:.0%}")
    print("→ 크면(예: 50% 이상) MediaPipe가 이미 내부에서 떨림을 걸러 주고 있다는 뜻")
