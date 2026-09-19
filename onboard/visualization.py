"""Small, read-only snapshots of the onboard planner's existing ROS data."""
import bisect
import math


def project_cloud(points, center, radius=8.0, limit=1800):
    """Project the inflated occupancy cloud in the configured 1.0–1.6 m band."""
    cells = {}
    for x, y, z in points:
        if not all(math.isfinite(v) for v in (x, y, z)):
            continue
        if not (1.0 <= z <= 1.6 and abs(x-center[0]) <= radius and abs(y-center[1]) <= radius):
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
