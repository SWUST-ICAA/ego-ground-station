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


def project_cloud(points, center, radius=8.0, limit=3000, z_min=1.0, z_max=1.4):
    """Bound and downsample in XYZ, preserving vertically separated surfaces."""
    cells = {}
    for x, y, z in points:
        if not all(math.isfinite(v) for v in (x, y, z)):
            continue
        if not (z_min <= z <= z_max and abs(x-center[0]) <= radius and abs(y-center[1]) <= radius):
            continue
        key = (math.floor(x*4), math.floor(y*4), math.floor(z*4))
        if key not in cells:cells[key] = (x, y, z)
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


def pack_voxels(points, resolution, center, z_min, z_max, radius=8.):
    """Lossless occupied-cell mask in the planner's native grid, including interior cells."""
    import base64,zlib
    import numpy as np
    p=np.asarray(points,dtype=np.float32).reshape(-1,3)
    valid=np.isfinite(p).all(axis=1)&(p[:,2]>=z_min)&(p[:,2]<=z_max)
    valid&=(np.abs(p[:,0]-center[0])<=radius)&(np.abs(p[:,1]-center[1])<=radius)
    p=p[valid]
    if len(p)==0:return dict(shape=[0,0,0],origin=[0.,0.,0.],resolution=resolution,data='',count=0)
    origin=p.min(axis=0).astype(float);indices=np.rint((p-origin)/resolution).astype(np.int64)
    if np.max(np.abs(origin+indices*resolution-p))>resolution*.02:
        raise ValueError('障碍点不在规则栅格上，无法按原始尺寸显示')
    shape=indices.max(axis=0)+1
    if math.prod(int(v) for v in shape)>4_000_000:raise ValueError('当前显示窗口超过400万栅格')
    mask=np.zeros(tuple(shape),dtype=np.uint8);mask[tuple(indices.T)]=1
    return dict(shape=shape.tolist(),origin=origin.tolist(),resolution=resolution,count=int(mask.sum()),
                data=base64.b64encode(zlib.compress(np.packbits(mask.reshape(-1)).tobytes(),1)).decode('ascii'))


def unpack_voxels(packet):
    import base64,zlib
    import numpy as np
    shape=packet['shape'];size=math.prod(shape)
    if len(shape)!=3 or any(type(v)is not int or v<0 for v in shape) or not 0<=size<=4_000_000:
        raise ValueError('栅格尺寸无效')
    resolution=float(packet['resolution']);origin=np.asarray(packet['origin'],dtype=float)
    if origin.shape!=(3,) or not np.isfinite(origin).all() or not math.isfinite(resolution) or resolution<=0:
        raise ValueError('栅格坐标无效')
    if size==0:return np.zeros((0,0,0),dtype=bool),origin,resolution
    decompressor=zlib.decompressobj();raw=decompressor.decompress(base64.b64decode(packet['data'],validate=True),(size+7)//8+1)
    if len(raw)!=(size+7)//8 or not decompressor.eof:raise ValueError('栅格数据长度无效')
    mask=np.unpackbits(np.frombuffer(raw,dtype=np.uint8),count=size).reshape(shape).astype(bool)
    return mask,origin,resolution
