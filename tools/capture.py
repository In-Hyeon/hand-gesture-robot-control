"""웹캠으로 손 기준점과 떨림 필터 효과를 보고, 필요하면 원본 좌표를 CSV로 녹화한다. 영상은 저장하지 않는다.

    python tools/capture.py                     # 화면으로 확인 (필터 비교 화면)
    python tools/capture.py --filter oneeuro    # 시작 필터 지정
    python tools/capture.py --record still.csv  # 녹화 (정지, 이동, 5mm 미세 이동 구간별로 따로)
    python tools/capture.py --record data/fine5.csv --secs 33 --guide fine   # 화면 안내에 맞춰 5mm 왕복 녹화
    python tools/capture.py --cam 0 --no-view   # 창 없이 FPS만

화면 (거울처럼 좌우 반전해서 보여 준다. 좌표·녹화는 반전하지 않은 원본 기준)
    빨강 = 원본 기준점과 최근 1초 궤적, 초록 = 필터 출력과 궤적
    오른쪽 아래 = 6배 확대 창 (1~2픽셀 떨림을 눈으로 보기 위함)
    jitter = 최근 1초 위치 표준편차(픽셀). 손을 가만히 들고 있을 때만 의미가 있다

키
    1 none  2 ma  3 deadzone  4 oneeuro      필터 바꾸기
    [ ]   주 파라미터 줄이기/늘리기 (ma window, deadzone radius, oneeuro min_cutoff)
    - =   oneeuro beta 줄이기/늘리기
    c     궤적 지우기        q  종료

CSV 한 줄 = 카메라 프레임 1장: t(초, 프레임 획득 직후 시각), detected, handedness, confidence, x, y, lm0_x..lm20_z
x, y는 필터를 거치지 않은 원본이다(필터 비교는 녹화 후 오프라인으로 한다).
"""
import argparse
import csv
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from hand_teleop.camera import actual_settings, open_camera  # noqa: E402
from hand_teleop.filters import make_filter  # noqa: E402
from hand_teleop.tracker import BASE_IDX, LEFT, HandTracker  # noqa: E402

RED, GREEN, YELLOW, WHITE = (0, 0, 255), (0, 200, 0), (0, 255, 255), (255, 255, 255)
FILTER_KEYS = {ord("1"): "none", ord("2"): "ma", ord("3"): "deadzone", ord("4"): "oneeuro"}
TRAIL = 30        # 궤적·jitter 계산에 쓰는 프레임 수 (30fps에서 1초)
ZOOM, INSET = 6, 200  # 확대 배율, 확대 창 크기(px)

ap = argparse.ArgumentParser()
ap.add_argument("--cam", type=int, default=0)
ap.add_argument("--record", type=Path)
ap.add_argument("--no-view", action="store_true")
ap.add_argument("--filter", default="oneeuro", choices=list(FILTER_KEYS.values()))
ap.add_argument("--secs", type=float, default=0, help="이 시간(초)이 지나면 자동 종료. 0이면 q나 Ctrl+C로 종료")
ap.add_argument("--guide", choices=["still", "move", "fine"],
                help="녹화용 동작 안내를 화면에 띄운다. fine: 5mm 왕복 박자 (1초 이동, 1.5초 멈춤)")
args = ap.parse_args()


def guide_text(mode, t):
    """녹화 동작 안내 문구와 색. 처음 3초는 자리 잡기 (compare_filters가 정지 녹화의 처음 3초를 뺀다)."""
    if t < 3:
        return f"READY  {3 - t:.0f}", (0, 200, 255)
    if mode == "still":
        return "HOLD STILL", (0, 255, 0)
    if mode == "move":
        return "MOVE naturally: left/right, up/down", (0, 255, 0)
    # fine: 5초 주기. 화면이 거울 반전이라 화면의 오른쪽 = 내 오른쪽
    p = (t - 3) % 5.0
    if p < 1.0:
        return "MOVE RIGHT 5mm  -->", (0, 200, 255)
    if p < 2.5:
        return "HOLD", (0, 255, 0)
    if p < 3.5:
        return "<--  MOVE LEFT 5mm", (0, 200, 255)
    return "HOLD", (0, 255, 0)

# 필터별 파라미터 (키로 조절). 기본값은 filters.py / ros_node와 같다
params = {"none": {}, "ma": {"window": 5}, "deadzone": {"radius": 0.003},
          "oneeuro": {"min_cutoff": 1.0, "beta": 10.0}}
fname = args.filter
filt = make_filter(fname, **params[fname])


def set_filter(name):
    global fname, filt
    fname, filt = name, make_filter(name, **params[name])


def adjust(key):
    """키 입력으로 현재 필터 파라미터 조절. 바뀌었으면 True."""
    p = params[fname]
    if fname == "ma" and key in (ord("["), ord("]")):
        p["window"] = max(1, p["window"] + (1 if key == ord("]") else -1))
    elif fname == "deadzone" and key in (ord("["), ord("]")):
        p["radius"] = round(p["radius"] * (1.25 if key == ord("]") else 0.8), 5)
    elif fname == "oneeuro" and key in (ord("["), ord("]")):
        p["min_cutoff"] = round(p["min_cutoff"] * (1.25 if key == ord("]") else 0.8), 3)
    elif fname == "oneeuro" and key in (ord("-"), ord("=")):
        p["beta"] = round(p["beta"] * (1.5 if key == ord("=") else 1 / 1.5), 3)
    else:
        return False
    set_filter(fname)
    return True


def jitter_px(pts) -> float:
    """궤적 점들(px)의 2D 표준편차."""
    if len(pts) < 5:
        return 0.0
    a = np.array(pts)
    return float(np.sqrt(((a - a.mean(axis=0)) ** 2).sum(axis=1).mean()))


def draw_inset(frame, raw_pts, flt_pts):
    """오른쪽 아래에 필터 출력 중심으로 ZOOM배 확대한 궤적을 그린다."""
    h, w = frame.shape[:2]
    box = np.full((INSET, INSET, 3), 30, np.uint8)
    if flt_pts:
        cx, cy = flt_pts[-1]
        half = INSET // 2
        for pts, color in ((raw_pts, RED), (flt_pts, GREEN)):
            z = [(int(half + (x - cx) * ZOOM), int(half + (y - cy) * ZOOM)) for x, y in pts]
            if len(z) > 1:
                cv2.polylines(box, [np.array(z, np.int32)], False, color, 1, cv2.LINE_AA)
            if z:
                cv2.circle(box, z[-1], 4, color, -1)
        # 눈금: 1픽셀 = ZOOM칸
        cv2.line(box, (10, INSET - 12), (10 + 5 * ZOOM, INSET - 12), WHITE, 1)
        cv2.putText(box, "5px", (14 + 5 * ZOOM, INSET - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, WHITE, 1)
    cv2.putText(box, f"x{ZOOM}", (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, WHITE, 1)
    cv2.rectangle(box, (0, 0), (INSET - 1, INSET - 1), WHITE, 1)
    frame[h - INSET - 10:h - 10, w - INSET - 10:w - 10] = box


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
raw_trail: deque[tuple[float, float]] = deque(maxlen=TRAIL)  # px, 반전 전 좌표
flt_trail: deque[tuple[float, float]] = deque(maxlen=TRAIL)
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

        h, w = frame.shape[:2]
        if hand:
            fx, fy = filt.update(t, hand.x, hand.y)
            # 두 축 모두 너비로 정규화된 값이라 픽셀로 되돌릴 때 둘 다 너비를 곱한다
            raw_trail.append((hand.x * w, hand.y * w))
            flt_trail.append((fx * w, fy * w))
        elif raw_trail:  # 손을 놓치면 필터와 궤적을 새로 시작 (노드와 같은 동작)
            filt.reset()
            raw_trail.clear()
            flt_trail.clear()

        fps_n += 1
        if time.perf_counter() - fps_t >= 1:
            fps, fps_t, fps_n = fps_n / (time.perf_counter() - fps_t), time.perf_counter(), 0
            if args.no_view:
                print(f"{fps:.1f} FPS, hand {'yes' if hand else 'no'}")
        if args.no_view:
            continue

        # --- 그리기: 원본 좌표로 그린 뒤 화면만 좌우 반전, 글자는 반전 후에 쓴다 ---
        if hand:
            for i, (px, py, _) in enumerate(hand.landmarks):
                cv2.circle(frame, (int(px * w), int(py * h)), 4 if i in BASE_IDX else 2,
                           YELLOW if i in BASE_IDX else (0, 160, 0), -1)
        for pts, color in ((raw_trail, RED), (flt_trail, GREEN)):
            if len(pts) > 1:
                cv2.polylines(frame, [np.array(pts, np.int32)], False, color, 2, cv2.LINE_AA)
            if pts:
                cv2.circle(frame, tuple(int(v) for v in pts[-1]), 6, color, -1)
        frame = cv2.flip(frame, 1)
        # 확대 창은 반전된 화면 기준 좌표로 그려야 손 움직임 방향과 맞는다
        draw_inset(frame, [(w - x, y) for x, y in raw_trail], [(w - x, y) for x, y in flt_trail])

        jr, jf = jitter_px(raw_trail), jitter_px(flt_trail)
        side = ("L" if hand.handedness == LEFT else "R") + f" {hand.confidence:.2f}" if hand else "no hand"
        lines = [
            (f"{fps:.1f} FPS  {side}" + ("  REC" if writer else ""), WHITE),
            (f"filter: {fname} {params[fname] or ''}", GREEN),
            (f"jitter(1s): raw {jr:.2f}px  filt {jf:.2f}px"
             + (f"  reduction {1 - jf / jr:.0%}" if jr > 0.05 else ""), WHITE),
            ("1-4 filter  [ ] param  - = beta  c clear  q quit", (180, 180, 180)),
        ]
        for i, (text, color) in enumerate(lines):
            y = 24 + i * 24
            cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1, cv2.LINE_AA)
        if args.guide:
            text, color = guide_text(args.guide, t)
            if args.secs:
                text += f"   ({args.secs - t:.0f}s)"
            (tw, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)
            org = (max(10, (w - tw) // 2), h // 2)
            cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 6, cv2.LINE_AA)
            cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2, cv2.LINE_AA)
        cv2.imshow("hand", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key in FILTER_KEYS:
            set_filter(FILTER_KEYS[key])
            flt_trail.clear()
        elif key == ord("c"):
            raw_trail.clear()
            flt_trail.clear()
        elif adjust(key):
            flt_trail.clear()
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
