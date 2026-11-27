"""떨림 필터: 손 기준점 (x, y)를 매 프레임 받아 안정화한다. ROS 없는 순수 Python.

모든 필터는 같은 형태라서 같은 입력(녹화 데이터)에 바꿔 끼울 수 있다.
    f = make_filter("oneeuro", min_cutoff=1.0, beta=10.0)
    x, y = f.update(t, x, y)   # t: 초 단위 프레임 획득 시각 (단조 증가)
    f.reset()                  # 손을 놓쳤을 때. 다음 update는 입력을 그대로 내보낸다

좌표 단위는 /hand/state와 같다(두 축 모두 영상 너비로 정규화). 파라미터 중 거리값(데드존 반경)도 이 단위다.
mm 배율이 정해지면 비교 스크립트에서 변환한다.
"""
import math
from collections import deque


class Filter:
    name = "none"

    def update(self, t: float, x: float, y: float) -> tuple[float, float]:
        return x, y

    def reset(self):
        pass


class MovingAverage(Filter):
    """최근 window개 프레임의 평균. 지연 ≈ (window-1)/2 프레임 (30fps, window 5 → 약 67ms)."""
    name = "ma"

    def __init__(self, window: int = 5):
        if window < 1:
            raise ValueError("window must be >= 1")
        self.buf: deque[tuple[float, float]] = deque(maxlen=window)
        self.sx = self.sy = 0.0

    def update(self, t, x, y):
        if len(self.buf) == self.buf.maxlen:
            ox, oy = self.buf[0]
            self.sx -= ox
            self.sy -= oy
        self.buf.append((x, y))
        self.sx += x
        self.sy += y
        n = len(self.buf)
        return self.sx / n, self.sy / n

    def reset(self):
        self.buf.clear()
        self.sx = self.sy = 0.0


class Deadzone(Filter):
    """출력 주변 반경 안의 움직임은 무시한다. 입력이 반경을 벗어나면 출력을 경계까지만 끌고 간다.

    출력이 순간 이동하지 않고, 움직이는 동안의 지연은 거리로 radius만큼이다(시간 지연 없음).
    반경보다 작은 움직임은 반영되지 않으므로 5mm 미세 이동 반영률로 반경 상한을 정한다.
    """
    name = "deadzone"

    def __init__(self, radius: float = 0.003):
        if radius < 0:
            raise ValueError("radius must be >= 0")
        self.r = radius
        self.out: tuple[float, float] | None = None

    def update(self, t, x, y):
        if self.out is None:
            self.out = (x, y)
            return self.out
        ox, oy = self.out
        dx, dy = x - ox, y - oy
        d = math.hypot(dx, dy)
        if d > self.r:
            k = (d - self.r) / d
            self.out = (ox + dx * k, oy + dy * k)
        return self.out

    def reset(self):
        self.out = None


def _alpha(cutoff: float, dt: float) -> float:
    """1차 저역통과 필터 계수. cutoff(Hz)가 낮을수록 부드럽고 느리다."""
    tau = 1.0 / (2 * math.pi * cutoff)
    return 1.0 / (1.0 + tau / dt)


class OneEuro(Filter):
    """One Euro 필터 (Casiez et al., CHI 2012). 느리게 움직이면 강하게, 빠르게 움직이면 약하게 거른다.

    cutoff = min_cutoff + beta × 속도. 정지 떨림은 min_cutoff로, 이동 지연은 beta로 조절한다.
    x, y에 같은 cutoff를 쓴다(속도 = 2D 속력). 축마다 다르면 원을 그렸을 때 찌그러진다.
    속도 단위는 정규화 좌표/초라서 beta도 이 단위 기준이다.
    """
    name = "oneeuro"

    def __init__(self, min_cutoff: float = 1.0, beta: float = 10.0, d_cutoff: float = 1.0):
        if min_cutoff <= 0 or d_cutoff <= 0 or beta < 0:
            raise ValueError("min_cutoff, d_cutoff must be > 0 and beta >= 0")
        self.min_cutoff, self.beta, self.d_cutoff = min_cutoff, beta, d_cutoff
        self.reset()

    def update(self, t, x, y):
        if self.t is None:
            self.t, self.x, self.y = t, x, y
            return x, y
        dt = t - self.t
        if dt <= 0:  # 같은 시각이 두 번 들어오면 이전 출력 유지
            return self.x, self.y
        self.t = t
        a_d = _alpha(self.d_cutoff, dt)
        self.dx += a_d * ((x - self.x) / dt - self.dx)
        self.dy += a_d * ((y - self.y) / dt - self.dy)
        a = _alpha(self.min_cutoff + self.beta * math.hypot(self.dx, self.dy), dt)
        self.x += a * (x - self.x)
        self.y += a * (y - self.y)
        return self.x, self.y

    def reset(self):
        self.t = None
        self.x = self.y = self.dx = self.dy = 0.0


FILTERS = {c.name: c for c in (Filter, MovingAverage, Deadzone, OneEuro)}


def make_filter(name: str, **params) -> Filter:
    """이름과 파라미터로 필터 생성. 예: make_filter("ma", window=5)"""
    if name not in FILTERS:
        raise ValueError(f"filter must be one of {list(FILTERS)}, got {name!r}")
    return FILTERS[name](**params)


if __name__ == "__main__":
    import random

    # 같은 입력에 4개 필터를 바꿔 끼워 본다: 정지(떨림) 1초 → 오른쪽으로 0.3/s 이동 1초
    random.seed(0)
    fps, noise = 30, 0.002
    ts = [i / fps for i in range(60)]
    true_x = [0.5 if t < 1 else 0.5 + 0.3 * (t - 1) for t in ts]
    xs = [v + random.gauss(0, noise) for v in true_x]
    ys = [0.4 + random.gauss(0, noise) for _ in ts]

    def std(v):
        m = sum(v) / len(v)
        return math.sqrt(sum((a - m) ** 2 for a in v) / len(v))

    print(f"{'filter':10s} {'still std':>10s} {'reduction':>10s} {'lag at end':>11s}")
    for f in (make_filter("none"), make_filter("ma", window=5),
              make_filter("deadzone", radius=0.004), make_filter("oneeuro", min_cutoff=1.0, beta=10.0)):
        out = [f.update(t, x, y) for t, x, y in zip(ts, xs, ys)]
        still = [o[0] for o in out[10:30]]  # 첫 프레임 영향을 피해 정지 구간 뒤쪽만
        s = std(still)
        lag = true_x[-1] - out[-1][0]
        print(f"{f.name:10s} {s:10.5f} {1 - s / std(xs[10:30]):10.0%} {lag:11.4f}")

    # 기본 동작 점검
    ma = MovingAverage(3)
    assert [ma.update(0, v, 0)[0] for v in (3, 6, 9, 12)] == [3, 4.5, 6, 9]
    ma.reset()
    assert ma.update(0, 1, 1) == (1, 1)

    dz = Deadzone(1.0)
    assert dz.update(0, 0, 0) == (0, 0)
    assert dz.update(0, 0.5, 0) == (0, 0)        # 반경 안: 무시
    assert dz.update(0, 3, 0) == (2, 0)          # 반경 밖: 경계까지만 따라감
    dz.reset()
    assert dz.update(0, 5, 5) == (5, 5)          # reset 후 첫 입력은 그대로

    oe = OneEuro()
    assert oe.update(0.0, 1, 1) == (1, 1)
    assert oe.update(0.0, 2, 2) == (1, 1)        # dt=0이면 이전 출력 유지
    for i in range(1, 300):                      # 계속 같은 값이면 그 값으로 수렴
        x, y = oe.update(i / 30, 2, 2)
    assert abs(x - 2) < 1e-6 and abs(y - 2) < 1e-6

    try:
        make_filter("kalman")
        raise AssertionError("unknown filter should fail")
    except ValueError:
        pass
    print("filters self-check OK")
