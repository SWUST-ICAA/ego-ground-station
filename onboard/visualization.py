"""Small, read-only snapshots of the onboard planner's existing ROS data."""
import bisect
import math


def polynomial_hulls(msg):
    """Exact quintic power-to-Bernstein conversion for conservative fence checks."""
    durations = list(msg.duration)
    axes = [list(msg.coef_x), list(msg.coef_y), list(msg.coef_z)]
    if msg.order != 5 or not 1 <= len(durations) <= 1000:
        raise ValueError('无效的五次多项式轨迹')
    if any(len(a) != 6*len(durations) for a in axes):
        raise ValueError('多项式系数长度不一致')
    if not all(math.isfinite(t) and 0 < t <= 300 for t in durations):
        raise ValueError('多项式时间段无效')
    if not all(math.isfinite(c) for a in axes for c in a):
        raise ValueError('多项式系数无效')
    hulls = []
    for piece, duration in enumerate(durations):
        powers = [[a[piece*6+5-i]*duration**i for i in range(6)] for a in axes]
        points = [[sum(powers[axis][i]*math.comb(k,i)/math.comb(5,i) for i in range(k+1))
                   for axis in range(3)] for k in range(6)]
        if not all(math.isfinite(v) for p in points for v in p):
            raise ValueError('多项式包络无效')
        hulls.append(points)
    return hulls


def sample_polynomial(msg, count=41):
    hulls = polynomial_hulls(msg)
    total = sum(msg.duration); result = []
    for index in range(count):
        t = total*index/(count-1); piece = 0
        while piece < len(hulls)-1 and t > msg.duration[piece]:
            t -= msg.duration[piece]; piece += 1
        u = min(1.,max(0.,t/msg.duration[piece])); work = hulls[piece]
        for _ in range(5):
            work = [[(1-u)*a+u*b for a,b in zip(p,q)] for p,q in zip(work,work[1:])]
        result.append([round(v,2) for v in work[0]])
    return result


def project_cloud(points, center, radius=8.0, limit=1800, z_min=1.0, z_max=1.4):
    """Project the inflated occupancy cloud throughout the selected map height."""
    cells = {}
    for x, y, z in points:
        if not all(math.isfinite(v) for v in (x, y, z)):
            continue
        if not (z_min <= z <= z_max and abs(x-center[0]) <= radius and abs(y-center[1]) <= radius):
            continue
        key = (math.floor(x*4), math.floor(y*4))
        old = cells.get(key)
        if old is None or z > old[2]:
            cells[key] = (x, y, z)
    values = [cells[key] for key in sorted(cells)]
    stride = max(1, math.ceil(len(values)/limit))
    return [[round(v, 2) for v in point] for point in values[::stride]]


def sample_bspline(points, knots, count=41):
    """Sample a cubic ROS Bspline message using de Boor's algorithm."""
    degree = 3
    if (len(points) < 4 or len(knots) != len(points)+degree+1 or not 2 <= count <= 101):
        return []
    if not all(math.isfinite(v) for p in points for v in p) or not all(math.isfinite(k) for k in knots):
        return []
    if any(a > b for a, b in zip(knots, knots[1:])):
        return []
    start, end = knots[degree], knots[len(points)]
    if end <= start:
        return []
    out = []
    for index in range(count):
        t = start + (end-start)*index/(count-1)
        span = min(len(points)-1, bisect.bisect_right(knots, t)-1)
        span = max(degree, span)
        work = [list(points[span-degree+j]) for j in range(degree+1)]
        for level in range(1, degree+1):
            for j in range(degree, level-1, -1):
                left, right = knots[span-degree+j], knots[span+j-level+1]
                alpha = (t-left)/(right-left) if right > left else 0.0
                work[j] = [(1-alpha)*a+alpha*b for a, b in zip(work[j-1], work[j])]
        out.append([round(v, 2) for v in work[degree]])
    return out
