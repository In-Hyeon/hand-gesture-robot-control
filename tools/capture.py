"""웹캠으로 손 기준점을 보고, 필요하면 좌표를 CSV로 녹화한다. 영상은 저장하지 않는다.

    python tools/capture.py                     # 화면으로 확인 (q 종료)
    python tools/capture.py --record still.csv  # 녹화 (정지, 이동, 5mm 미세 이동 구간별로 따로)
    python tools/capture.py --cam 0 --no-view   # 창 없이 FPS만

CSV 한 줄 = 카메라 프레임 1장: t(초, 프레임 획득 직후 시각), detected, handedness, confidence, x, y, lm0_x..lm20_z
rosbag2 녹화는 Ubuntu에서 ROS 노드로 한다. 그 전까지 필터 비교용 입력은 이 CSV다.
"""
import argparse
import csv
import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from hand_teleop.camera import actual_settings, open_camera  # noqa: E402
from hand_teleop.tracker import BASE_IDX, LEFT, HandTracker  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--cam", type=int, default=0)
ap.add_argument("--record", type=Path)
ap.add_argument("--no-view", action="store_true")
ap.add_argument("--secs", type=float, default=0, help="이 시간(초)이 지나면 자동 종료. 0이면 q나 Ctrl+C로 종료")
args = ap.parse_args()

cap = open_camera(args.cam)
print(f"camera {args.cam}: {actual_settings(cap)}")

out = writer = None
if args.record:
    args.record.parent.mkdir(parents=True, exist_ok=True)
    out = open(args.record, "w", newline="")
    writer = csv.writer(out)
    writer.writerow(["t", "detected", "handedness", "confidence", "x", "y"]
                    + [f"lm{i}_{a}" for i in range(21) for a in "xyz"])

tracker = HandTracker()
t0 = time.perf_counter()
n = seen = 0
fps_t, fps_n, fps = t0, 0, 0.0
try:
    while True:
        ok, frame = cap.read()
        t = time.perf_counter() - t0  # ponytail: read() 직후 시각. 드라이버 버퍼 지연은 13주차 240fps 촬영으로 따로 잰다
        if args.secs and t > args.secs:
            break
        if not ok:
            time.sleep(0.01)  # 카메라가 빠졌을 때 CPU를 쓰며 헛돌지 않게
            continue
        hand = tracker.process(frame, int(t * 1000))
        n += 1
        seen += hand is not None

        if writer:
            if hand:
                writer.writerow([f"{t:.4f}", 1, hand.handedness, f"{hand.confidence:.3f}", f"{hand.x:.5f}", f"{hand.y:.5f}"]
                                + [f"{v:.5f}" for v in hand.landmarks.ravel()])
            else:
                writer.writerow([f"{t:.4f}", 0, 0, 0, "", ""] + [""] * 63)

        fps_n += 1
        if time.perf_counter() - fps_t >= 1:
            fps, fps_t, fps_n = fps_n / (time.perf_counter() - fps_t), time.perf_counter(), 0
            if args.no_view:
                print(f"{fps:.1f} FPS, hand {'yes' if hand else 'no'}")

        if not args.no_view:
            h, w = frame.shape[:2]
            if hand:
                for i, (px, py, _) in enumerate(hand.landmarks):
                    cv2.circle(frame, (int(px * w), int(py * h)), 5 if i in BASE_IDX else 2, (0, 255, 255) if i in BASE_IDX else (0, 200, 0), -1)
                cx, cy = int(hand.x * w), int(hand.y * w)  # y도 너비 기준이라 너비를 곱해야 픽셀
                cv2.drawMarker(frame, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
                label = f"{'L' if hand.handedness == LEFT else 'R'} {hand.confidence:.2f}  x={hand.x:.3f} y={hand.y:.3f}"
            else:
                label = "no hand"
            rec = "  REC" if writer else ""
            cv2.putText(frame, f"{fps:.1f} FPS  {label}{rec}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.imshow("hand", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
except KeyboardInterrupt:
    pass
finally:
    dt = time.perf_counter() - t0
    cap.release()
    tracker.close()
    cv2.destroyAllWindows()
    if out:
        out.close()
    print(f"{n} frames, {n / dt:.1f} FPS avg, hand seen {seen}/{n}" + (f", saved {args.record}" if args.record else ""))
