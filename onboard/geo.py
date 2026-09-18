"""WGS84 geodetic targets in an aircraft's own map frame; no shared origin."""
import math


def finite(value):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('坐标必须为有限数值')
    return value


def ecef(lat, lon, alt):
    lat, lon, alt = map(finite, (lat, lon, alt))
    if not -90 <= lat <= 90 or not -180 <= lon <= 180:
        raise ValueError('经纬度超出范围')
    p, l = math.radians(lat), math.radians(lon)
    n = 6378137.0 / math.sqrt(1 - 6.69437999014e-3 * math.sin(p)**2)
    return ((n+alt)*math.cos(p)*math.cos(l), (n+alt)*math.cos(p)*math.sin(l),
            (n*(1-6.69437999014e-3)+alt)*math.sin(p))


def to_local(lat, lon, anchor):
    """Anchor contains WGS84 lat/lon/alt, local x/y, ENU-to-map rotation (rad)."""
    p = ecef(lat, lon, anchor['alt'])
    a = ecef(anchor['lat'], anchor['lon'], anchor['alt'])
    dx, dy, dz = (p[i]-a[i] for i in range(3))
    phi, lam = math.radians(anchor['lat']), math.radians(anchor['lon'])
    east = -math.sin(lam)*dx + math.cos(lam)*dy
    north = -math.sin(phi)*math.cos(lam)*dx - math.sin(phi)*math.sin(lam)*dy + math.cos(phi)*dz
    r = finite(anchor['rotation'])
    return [finite(anchor['x'])+math.cos(r)*east-math.sin(r)*north,
            finite(anchor['y'])+math.sin(r)*east+math.cos(r)*north]
