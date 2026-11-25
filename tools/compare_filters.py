"""떨림 필터 오프라인 비교: 같은 녹화(capture.py CSV)에 필터·파라미터를 바꿔 끼워 지표를 계산하고 최적값을 고른다.

    python tools/compare_filters.py --still data/still.csv --move data/move.csv --fine data/fine5.csv
    python tools/compare_filters.py ... --mm-per-unit 600   # 절대 떨림을 mm로도 보고 (배율 정해지면)

지표 (CLAUDE.md 5절)
    정지 떨림 감소율  still: 1초 창마다 2D 표준편차 → 평균. 1 - 필터/원본. 목표 70% 이상
    필터 추가 지연    move: 원본과 필터 출력을 1ms 격자로 보간, 상호상관이 최대인 시간차. 목표 50ms 이내
    5mm 반영률       fine: 손을 멈춘 구간(hold)을 찾아 연속 hold 사이 이동량을 원본·필터로 비교. 목표 90% 이상
세 지표 모두 비율·시간이라 mm 배율 없이 계산된다. 배율은 절대 떨림 보고에만 쓴다.

선정 규칙: 지연 ≤ 50ms 이고 반영률 ≥ 90% 인 조합 중 떨림 감소율이 가장 큰 것.

결과: <out>/all.csv (전체 조합), <out>/summary.md (필터별 최적값 표), <out>/*.png (원본 vs 선정 필터)
"""
import argparse
import csv
import itertools
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from hand_teleop.filters import make_filter  # noqa: E402

LAT_MAX_MS, REFLECT_MIN, STILL_MIN = 50.0, 0.90, 0.70

GRID = {
    "none": [{}],
    "ma": [{"window": w} for w in range(2, 16)],
    "deadzone": [{"radius": r} for r in (0.0005, 0.001, 0.0015, 0.002, 0.003, 0.004, 0.006, 0.008)],
    "oneeuro": [{"min_cutoff": mc, "beta": b} for mc, b in itertools.product(
        (0.005, 0.01, 0.02, 0.03, 0.05, 0.1, 0.2, 0.3, 0.5, 0.7, 1.0, 1.5, 2.0, 3.0, 5.0), (0, 1, 2, 5, 10, 20, 50, 100))],
}


# ---------------------------------------------------------------- 입출력
def load(path: Path):
    """capture.py CSV → t(초), det(bool), x, y (손이 없으면 nan)."""
    t, det, x, y = [], [], [], []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            t.append(float(row["t"]))
            d = row["detected"] == "1"
            det.append(d)
            x.append(float(row["x"]) if d else np.nan)
            y.append(float(row["y"]) if d else np.nan)
    return np.array(t), np.array(det), np.array(x), np.array(y)


def spans(det, min_len=1):
    """손이 연속으로 잡힌 구간 [(시작, 끝+1), ...]."""
    out, start = [], None
    for i, d in enumerate(np.append(det, False)):
        if d and start is None:
            start = i
        elif not d and start is not None:
            if i - start >= min_len:
                out.append((start, i))
            start = None
    return out


def apply(name, params, t, det, x, y):
    """녹화 전체에 필터 적용. 손을 놓치면 reset (노드·capture.py와 같은 동작)."""
    f = make_filter(name, **params)
    fx, fy = np.full_like(x, np.nan), np.full_like(y, np.nan)
    prev = False
    for i in range(len(t)):
        if det[i]:
            fx[i], fy[i] = f.update(t[i], x[i], y[i])
        elif prev:
            f.reset()
        prev = det[i]
    return fx, fy


# ---------------------------------------------------------------- 지표
def jitter(t, det, x, y, warmup=3.0, win=1.0):
    """정지 떨림: 각 연속 구간에서 처음 warmup초를 빼고, win초 창마다 2D 표준편차의 평균.
    창을 나누는 이유: 20초 동안 손이 천천히 흘러가는(drift) 것은 떨림이 아니므로 빼려는 것.
    warmup 3초: 녹화를 시작하고 손을 자리 잡는 움직임을 뺀다 (1초로는 부족했다. 실측 0.58 → 0.34px, 2026-11-26)."""
    stds = []
    for s, e in spans(det):
        i = s
        while i < e and t[i] - t[s] < warmup:
            i += 1
        while i < e:
            j = i
            while j < e and t[j] - t[i] < win:
                j += 1
            if j - i >= 10:
                px, py = x[i:j], y[i:j]
                stds.append(np.sqrt(((px - px.mean()) ** 2 + (py - py.mean()) ** 2).mean()))
            i = j
    return float(np.mean(stds)) if stds else np.nan


def jitter_hf(t, det, x, y, warmup=3.0, half=0.25):
    """빠른 떨림: 앞뒤 half초(합 0.5초) 이동평균을 뺀 나머지의 2D RMS. 대략 2Hz 이상 성분이다.
    손이 천천히 흔들리는 실제 움직임(1Hz 미만)은 빠지고, 센서·검출 잡음에 가까운 떨림만 남는다.
    정지 녹화의 94%가 1Hz 미만이었던 실측(2026-11-26) 때문에 추가한 보조 지표."""
    res = []
    for s, e in spans(det):
        tt, xx, yy = t[s:e], x[s:e], y[s:e]
        keep = tt - tt[0] >= warmup
        if keep.sum() < 30:
            continue
        lo = np.searchsorted(tt, tt - half)
        hi = np.searchsorted(tt, tt + half, side="right")
        cx, cy = np.concatenate([[0], np.cumsum(xx)]), np.concatenate([[0], np.cumsum(yy)])
        n = hi - lo
        mx, my = (cx[hi] - cx[lo]) / n, (cy[hi] - cy[lo]) / n
        res.append(((xx - mx) ** 2 + (yy - my) ** 2)[keep])
    return float(np.sqrt(np.concatenate(res).mean())) if res else np.nan


def latency_ms(t, det, x, y, fx, fy, max_ms=300, warmup=0.5):
    """원본 대비 필터 출력의 지연(ms): 1ms 격자로 보간한 뒤 정규화 상호상관이 최대인 시간차."""
    lags = np.arange(0, max_ms + 1)
    num = np.zeros(len(lags))
    weight = 0.0
    for s, e in spans(det, min_len=30):
        t0, t1 = t[s] + warmup, t[e - 1]
        if t1 - t0 < 1.0:
            continue
        g = np.arange(t0, t1, 0.001)
        seg = slice(s, e)
        sigs = [(np.interp(g, t[seg], a[seg]), np.interp(g, t[seg], b[seg])) for a, b in ((x, fx), (y, fy))]
        for k, L in enumerate(lags):
            c = 0.0
            for r, f in sigs:
                r0, f0 = r[:len(r) - L] if L else r, f[L:]
                r0, f0 = r0 - r0.mean(), f0 - f0.mean()
                c += (r0 * f0).sum() / (np.sqrt((r0 ** 2).sum() * (f0 ** 2).sum()) + 1e-12)
            num[k] += c * len(g)
        weight += len(g)
    return float(lags[np.argmax(num)]) if weight else np.nan


def holds(t, det, x, y, speed_th=0.0125, min_hold=0.5, half=0.17):
    """미세 이동 녹화에서 손을 멈춘 구간 찾기. 원본을 ±half초 평균으로 다듬고,
    ±half초 앞뒤 위치 차이로 속도를 구해 speed_th(정규화 단위/초, 640px에서 8px/s) 미만이 min_hold초 이상이면 hold."""
    out = []
    for s, e in spans(det, min_len=30):
        tt, xx, yy = t[s:e], x[s:e], y[s:e]
        n = len(tt)
        sx, sy = np.empty(n), np.empty(n)
        for i in range(n):
            m = np.abs(tt - tt[i]) <= half
            sx[i], sy[i] = xx[m].mean(), yy[m].mean()
        still = np.zeros(n, bool)
        for i in range(n):
            a = np.searchsorted(tt, tt[i] - half)
            b = min(np.searchsorted(tt, tt[i] + half), n - 1)
            dt = tt[b] - tt[a]
            if dt > 0:
                still[i] = np.hypot(sx[b] - sx[a], sy[b] - sy[a]) / dt < speed_th
        for a, b in spans(still):
            if tt[b - 1] - tt[a] >= min_hold:
                out.append((s + a, s + b))
    return out


def reflection(t, det, x, y, fx, fy, hold_list, min_step):
    """연속 hold 사이 이동: 원본 이동 벡터 방향으로 필터 이동량을 투영해 비율을 구한 평균.
    각 hold의 뒤쪽 절반만 쓴다(필터가 따라올 시간을 준다). min_step보다 작은 원본 이동은 떨림으로 보고 뺀다."""
    def pos(a, b, px, py):
        m = (a + b) // 2
        return np.array([px[m:b].mean(), py[m:b].mean()])

    ratios, steps = [], []
    for (a0, b0), (a1, b1) in zip(hold_list, hold_list[1:]):
        if not det[b0 - 1:a1 + 1].all():  # 사이에 손을 놓쳤으면 필터가 reset되어 비교 불가
            continue
        dr = pos(a1, b1, x, y) - pos(a0, b0, x, y)
        d = np.linalg.norm(dr)
        if d < min_step:
            continue
        df = pos(a1, b1, fx, fy) - pos(a0, b0, fx, fy)
        ratios.append(float(df @ dr) / d ** 2)
        steps.append(d)
    return (float(np.mean(ratios)) if ratios else np.nan), steps


# ---------------------------------------------------------------- 실행
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--still", type=Path, required=True)
    ap.add_argument("--move", type=Path, required=True)
    ap.add_argument("--fine", type=Path, required=True)
    ap.add_argument("--out", type=Path, default=Path("data/results"))
    ap.add_argument("--width", type=int, default=640, help="녹화 영상 너비(px). 정규화 좌표 → 픽셀 표시용")
    ap.add_argument("--mm-per-unit", type=float, help="정규화 1.0 = 로봇 공간 몇 mm (정해지면)")
    ap.add_argument("--speed-th", type=float, default=0.0125, help="hold 판정 속도 임계값 (정규화/초)")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    still, move, fine = load(args.still), load(args.move), load(args.fine)
    for name, (t, det, _, _) in (("still", still), ("move", move), ("fine", fine)):
        print(f"{name}: {len(t)} frames, {t[-1] - t[0]:.1f}s, hand {det.mean():.0%}, "
              f"{(len(t) - 1) / (t[-1] - t[0]):.1f} FPS")

    raw_jit = jitter(*still)
    raw_hf = jitter_hf(*still)
    hold_list = holds(*fine, speed_th=args.speed_th)
    # 의도한 이동으로 볼 최소 이동량. hold 위치는 7프레임 이상 평균이라 오차가 프레임 떨림의 약 1/√7(≈0.4배)이므로,
    # 프레임 떨림 1배면 위치 오차의 2.5배 정도다. (처음에 3배로 했다가 5mm 이동이 전부 걸러져서 고침)
    min_step = raw_jit
    _, raw_steps = reflection(*fine, fine[2], fine[3], hold_list, min_step)
    px = args.width
    print(f"\n원본 정지 떨림: {raw_jit * px:.2f}px" + (f" = {raw_jit * args.mm_per_unit:.2f}mm" if args.mm_per_unit else "")
          + f", 빠른 떨림(≈2Hz 이상): {raw_hf * px:.3f}px")
    print(f"미세 이동: hold {len(hold_list)}개, 비교한 이동 {len(raw_steps)}번, "
          f"원본 이동량 평균 {np.mean(raw_steps) * px if raw_steps else float('nan'):.1f}px "
          f"(5mm 이동이 이 정도 픽셀이어야 함. 너무 다르면 --speed-th 조정)")
    if len(raw_steps) < 5:
        print("경고: 비교할 미세 이동이 5번 미만이라 반영률을 믿기 어렵다. 녹화를 다시 하거나 --speed-th를 조정할 것")

    rows = []
    for name, grid in GRID.items():
        for params in grid:
            sfx, sfy = apply(name, params, *still)
            jit = jitter(still[0], still[1], sfx, sfy)
            hf = jitter_hf(still[0], still[1], sfx, sfy)
            mfx, mfy = apply(name, params, *move)
            lat = latency_ms(*move, mfx, mfy)
            ffx, ffy = apply(name, params, *fine)
            refl, _ = reflection(*fine, ffx, ffy, hold_list, min_step)
            red = 1 - jit / raw_jit
            ok = lat <= LAT_MAX_MS and refl >= REFLECT_MIN
            rows.append({"filter": name, "params": params, "reduction": red, "jitter_px": jit * px,
                         "hf_reduction": 1 - hf / raw_hf,
                         "latency_ms": lat, "reflection": refl, "pass": ok and red >= STILL_MIN, "eligible": ok})
        print(f"  {name}: {len(grid)}개 조합 완료")

    with open(args.out / "all.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["filter", "params", "reduction", "jitter_px", "hf_reduction", "latency_ms", "reflection",
                    "eligible", "meets_all"])
        for r in rows:
            w.writerow([r["filter"], r["params"], f"{r['reduction']:.3f}", f"{r['jitter_px']:.3f}", f"{r['hf_reduction']:.3f}",
                        f"{r['latency_ms']:.0f}", f"{r['reflection']:.3f}", r["eligible"], r["pass"]])

    def fmt(r):
        p = ", ".join(f"{k}={v}" for k, v in r["params"].items()) or "-"
        mark = "✅" if r["pass"] else ("⚠️ 떨림 감소 부족" if r["eligible"] else "❌")
        return (f"| {r['filter']} | {p} | {r['reduction']:.0%} | {r['jitter_px']:.2f} | {r['hf_reduction']:.0%} | "
                f"{r['latency_ms']:.0f} | {r['reflection']:.0%} | {mark} |")

    lines = [f"# 필터 비교 결과", "",
             f"입력: still `{args.still}`, move `{args.move}`, fine `{args.fine}`", "",
             f"- 원본 정지 떨림 {raw_jit * px:.2f}px"
             + (f" ({raw_jit * args.mm_per_unit:.2f}mm)" if args.mm_per_unit else " (mm 배율 미정)")
             + f", 빠른 떨림(≈2Hz 이상) {raw_hf * px:.3f}px",
             f"- 미세 이동 비교 {len(raw_steps)}번, 원본 이동량 평균 "
             f"{np.mean(raw_steps) * px if raw_steps else float('nan'):.1f}px",
             f"- 조건: 지연 ≤ {LAT_MAX_MS:.0f}ms, 반영률 ≥ {REFLECT_MIN:.0%} 중 떨림 감소 최대 (목표 {STILL_MIN:.0%} 이상)", "",
             "## 필터별 최적값", "",
             "| 필터 | 파라미터 | 떨림 감소 | 떨림(px) | 빠른 떨림 감소 | 지연(ms) | 5mm 반영 | 판정 |",
             "|---|---|---|---|---|---|---|---|"]
    best = {}
    for name in GRID:
        cand = [r for r in rows if r["filter"] == name and r["eligible"]]
        if cand:
            best[name] = max(cand, key=lambda r: r["reduction"])
            lines.append(fmt(best[name]))
        else:
            lines.append(f"| {name} | (조건을 만족하는 조합 없음) | | | | | | ❌ |")
    chosen = max(best.values(), key=lambda r: r["reduction"]) if best else None
    lines += ["", f"**선정:** {chosen['filter']} {chosen['params']}" if chosen else "**선정:** 조건을 만족하는 조합 없음"]
    # 최적값이 탐색 범위 끝에 걸리면 범위 밖에 더 좋은 값이 있을 수 있다
    for name, r in best.items():
        for k, v in r["params"].items():
            vals = sorted({p[k] for p in GRID[name]})
            if v in (vals[0], vals[-1]) and len(vals) > 1:
                lines.append(f"- 주의: {name} {k}={v}가 탐색 범위({vals[0]}~{vals[-1]}) 끝값이다. GRID 범위를 넓혀 다시 확인할 것")
    (args.out / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n" + "\n".join(lines[8:]))
    print(f"\n저장: {args.out / 'all.csv'}, {args.out / 'summary.md'}")

    if chosen and not args.no_plot:
        plot(args, still, move, fine, chosen, best, px)


def plot(args, still, move, fine, chosen, best, px):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = {"ma": "tab:blue", "deadzone": "tab:orange", "oneeuro": "tab:green"}
    fig, axes = plt.subplots(3, 1, figsize=(11, 10))
    # 한글 글꼴이 없는 환경에서도 깨지지 않게 그래프 글자는 영어로
    for ax, (title, data) in zip(axes, (("still: x", still), ("move: x", move), ("fine (5mm steps): x", fine))):
        t, det, x, y = data
        ax.plot(t, x * px, color="0.6", lw=1, label="raw")
        for name, r in best.items():
            if name == "none":
                continue
            fx, _ = apply(name, r["params"], *data)
            lw = 2 if r is chosen else 1
            ax.plot(t, fx * px, color=colors[name], lw=lw, label=f"{name} {r['params']}")
        ax.set_title(title)
        ax.set_ylabel("x (px)")
        ax.legend(fontsize=8, loc="upper right")
    axes[-1].set_xlabel("t (s)")
    fig.tight_layout()
    fig.savefig(args.out / "filters.png", dpi=110)
    print(f"그래프: {args.out / 'filters.png'}")


if __name__ == "__main__":
    main()
