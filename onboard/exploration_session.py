"""Task identity and conservative certificates for the existing explorer worker."""
import math
try:
    from .exploration_geometry import GridCertificate, remaining_hulls
except ImportError:
    from exploration_geometry import GridCertificate, remaining_hulls


class ExplorationSession:
    def __init__(self, parameters):
        self.parameters = parameters
        self.reset('', float('inf'))

    def reset(self, session, stamp):
        self.session = session
        self.started = stamp
        self.reject_id = 0
        self.target_id = None
        self.grid = None
        self.proposal = None

    def context(self, mission, stamp):
        phase = mission.phase
        index = mission.return_index if phase == 'RETURNING' else mission.index
        target = None
        if phase == 'OUTBOUND': target = mission.points[index]
        elif phase == 'RETURNING':
            target = mission.return_points[index] if mission.return_points else mission.home
        self.target_id = phase + ':' + str(index)
        p = self.parameters
        return dict(stamp=stamp, session=self.session, target_id=self.target_id,
                    reject_id=self.reject_id, active=bool(self.session and mission.fence and
                    phase in {'ARMING', 'TAKEOFF', 'OUTBOUND', 'RETURNING'}),
                    phase=phase, target=target, polygon=mission.fence.polygon if mission.fence else [],
                    margin=mission.fence.margin if mission.fence else 0.,
                    height=p['takeoff_height'], clearance=p['obstacles_inflation'])

    def update_grid(self, msg, now):
        try:
            certificate = GridCertificate(msg)
            stamp = msg.header.stamp.to_sec()
            if not math.isfinite(stamp) or stamp < self.started: return
            if self.grid and stamp < self.grid[1]: return
            height = float(msg.info.origin.position.z)
            if not math.isfinite(height) or abs(height-self.parameters['takeoff_height']) > .01: return
            self.grid = (certificate, stamp, now, msg.header.seq)
        except (ValueError, TypeError, AttributeError, OverflowError):
            self.grid = None

    def update_proposal(self, proposal, now):
        self.proposal = (proposal, now)

    def fresh_grid(self, stamp, now):
        if not self.grid: return None
        certificate, source, receipt, _ = self.grid
        if not -.1 <= stamp-source <= 1.5 or not 0 <= now-receipt <= 1.5: return None
        return certificate

    def proposal_goal(self, mission, position, stamp, now):
        self.context(mission, stamp)
        certificate = self.fresh_grid(stamp, now)
        if not certificate or not self.proposal or position is None: return None
        p, receipt = self.proposal
        try:
            if (not p.get('ready') or p.get('session') != self.session or
                p.get('target_id') != self.target_id or p.get('reject_id') != self.reject_id or
                not -.1 <= stamp-float(p['stamp']) <= 1.5 or not 0 <= now-receipt <= 1.5 or
                self.grid[3] < p['map_seq'] or p['map_stamp'] < self.started or
                not -.1 <= stamp-float(p['map_stamp']) <= 1.5): return None
            goal = list(map(float, p['goal']))
            if (len(goal) != 3 or not all(math.isfinite(v) for v in goal) or
                abs(goal[2]-self.parameters['takeoff_height']) > .01 or
                math.dist(goal[:2], position[:2]) > 3. or not certificate.point(goal) or
                not mission.fence.contains(goal[:2])): return None
            return goal
        except (KeyError, ValueError, TypeError, OverflowError): return None

    def validate_hulls(self, hulls, durations, elapsed, stamp, now):
        certificate = self.fresh_grid(stamp, now)
        if not certificate or elapsed < -.1: return False
        height = self.parameters['takeoff_height']
        lower = max(height-.15, self.parameters['ground_height'])
        upper = min(height+.15, self.parameters['ground_height']+self.parameters['map_size_z'])
        remaining = remaining_hulls(hulls, durations, max(0., elapsed))
        return bool(remaining) and all(certificate.curve(h, lower, upper) for h in remaining)
