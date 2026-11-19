"""Route and yaw planning for carrying an object with the arm held forward.

The base turns to follow the route. The held object (more exactly, a point
fixed in the base frame, normally the object's centre) rides along a smooth
path and the base trails behind it at a fixed distance, like a trailer: the
base centre stays on the same path, the base yaw points from the base to the
object. The arm and object therefore stay on the inside of every corner,
and the gripper's direction of motion is the path tangent at the object.

Everything here is geometry on the 0.1 m occupancy grid; no PyBullet calls.

Pieces:
- ClearanceField: distance from any xy point to the nearest occupied cell.
- route_cells: A* over grid cells whose clearance fits the base, with a cost
  that keeps the route on the corridor centreline.
- smooth_route: line-of-sight shortcuts and circular corner fillets, sampled
  every `ds` metres.
- CarryTrajectory: base pose (x, y, yaw) as a function of the object's arc
  length, a swept collision check of the robot+object footprint, and a speed
  profile with limits on the object speed, base speed, yaw rate and the rate
  at which the wrist must turn the gripper.
- spin_clear: collision check for turning on the spot.
"""
from heapq import heappop, heappush

import numpy as np
from scipy.ndimage import distance_transform_edt


def wrap(angle):
    return (np.asarray(angle) + np.pi) % (2 * np.pi) - np.pi


class ClearanceField:
    """Distance (m) from a point to the nearest occupied cell of `occupancy`.

    The 0.1 m grid is resampled `factor` times finer and a Euclidean distance
    transform taken; looking up the fine cell containing a point and taking
    off one fine cell diagonal makes the value a lower bound on the true
    distance to the occupied squares. Points outside the map have clearance 0.
    """

    def __init__(self, occupancy, x_min, y_max, cell_size, factor=5):
        occupied = np.kron(np.asarray(occupancy, bool), np.ones((factor, factor), bool))
        self.res = cell_size / factor
        self.x_min, self.y_max = float(x_min), float(y_max)
        dist = distance_transform_edt(~occupied) * self.res
        self.dist = np.maximum(0.0, dist - self.res * np.sqrt(2.0))
        self.dist[occupied] = 0.0
        self.shape = self.dist.shape

    def __call__(self, xy):
        xy = np.asarray(xy, float)
        col = np.floor((xy[..., 0] - self.x_min) / self.res).astype(int)
        row = np.floor((self.y_max - xy[..., 1]) / self.res).astype(int)
        inside = (row >= 0) & (row < self.shape[0]) & (col >= 0) & (col < self.shape[1])
        out = np.zeros(xy.shape[:-1])
        out[inside] = self.dist[row[inside], col[inside]]
        return out

    def segment_clear(self, a, b, clearance, step=0.02):
        a, b = np.asarray(a, float), np.asarray(b, float)
        n = max(2, int(np.ceil(np.linalg.norm(b - a) / step)) + 1)
        pts = a + np.linspace(0.0, 1.0, n)[:, None] * (b - a)
        return bool(np.all(self(pts) >= clearance))


def route_cells(field, start, goal, cell_size, x_min, y_max, shape, clearance, centre=0.48, weight=6.0):
    """A* from world point start to goal over 0.1 m cells (8-connected).

    A cell is usable when the base fits (clearance at its centre >= clearance).
    Each move costs its length times 1 + weight * (centre - clearance)+, so the
    route keeps to the middle of 1 m corridors and goes straight in the open.
    Returns world points (cell centres, start and goal included) or None.
    """
    rows, cols = shape
    rr, cc = np.mgrid[0:rows, 0:cols]
    centres = np.stack([x_min + (cc + 0.5) * cell_size, y_max - (rr + 0.5) * cell_size], axis=-1)
    clear = field(centres)
    free = clear >= clearance
    penalty = 1.0 + weight * np.maximum(0.0, centre - clear)

    def cell_of(xy):
        return (int(np.floor((y_max - xy[1]) / cell_size)), int(np.floor((xy[0] - x_min) / cell_size)))

    s, g = cell_of(start), cell_of(goal)
    for c in (s, g):
        if not (0 <= c[0] < rows and 0 <= c[1] < cols):
            return None
    free[s] = free[g] = True
    h = lambda c: np.hypot(c[0] - g[0], c[1] - g[1])
    best = {s: 0.0}
    came = {}
    frontier = [(h(s), 0.0, s)]
    moves = [(dr, dc, np.hypot(dr, dc)) for dr in (-1, 0, 1) for dc in (-1, 0, 1) if dr or dc]
    while frontier:
        _, cost, cur = heappop(frontier)
        if cost > best[cur]:
            continue
        if cur == g:
            path = [cur]
            while path[-1] in came:
                path.append(came[path[-1]])
            path.reverse()
            pts = [np.asarray(start, float)] + [centres[c] for c in path[1:-1]] + [np.asarray(goal, float)]
            return np.asarray(pts)
        for dr, dc, length in moves:
            nr, nc = cur[0] + dr, cur[1] + dc
            if not (0 <= nr < rows and 0 <= nc < cols) or not free[nr, nc]:
                continue
            if dr and dc and not (free[cur[0] + dr, cur[1]] and free[cur[0], cur[1] + dc]):
                continue
            new = cost + length * 0.5 * (penalty[cur] + penalty[nr, nc])
            if new < best.get((nr, nc), np.inf):
                best[(nr, nc)] = new
                came[(nr, nc)] = cur
                heappush(frontier, (new + h((nr, nc)), new, (nr, nc)))
    return None


def shortcut(points, field, clearance):
    """Greedy line-of-sight simplification keeping `clearance` along every segment."""
    out = [points[0]]
    i = 0
    while i < len(points) - 1:
        j = len(points) - 1
        while j > i + 1 and not field.segment_clear(points[i], points[j], clearance):
            j -= 1
        out.append(points[j])
        i = j
    return np.asarray(out)


def fillet(points, field, clearance, radius, ds=0.01):
    """Replace every corner of the polyline with a circular arc tangent to both
    legs, as large as `radius` allows: each arc may use at most half of each
    leg, and must keep `clearance` (else the radius is halved, down to 5 cm,
    then the sharp corner is kept). Returns dense samples (N, 2)."""
    pts = [np.asarray(p, float) for p in points]
    n = len(pts)
    if n < 2:
        return np.asarray(pts)
    # Tangent length available at each interior vertex.
    legs = [np.linalg.norm(pts[k + 1] - pts[k]) for k in range(n - 1)]
    pieces = []   # (entry point, exit point, centre or None, radius, turn sign)
    for k in range(1, n - 1):
        u = pts[k] - pts[k - 1]; u /= np.linalg.norm(u)
        v = pts[k + 1] - pts[k]; v /= np.linalg.norm(v)
        turn = np.arctan2(u[0] * v[1] - u[1] * v[0], u @ v)
        if abs(turn) < 1e-6:
            pieces.append((pts[k], pts[k], None, 0.0, 0.0))
            continue
        tan_half = np.tan(abs(turn) / 2)
        r = min(radius, 0.5 * legs[k - 1] / tan_half, 0.5 * legs[k] / tan_half)
        chosen = None
        while r >= 0.05:
            t = r * tan_half
            entry, exit_ = pts[k] - t * u, pts[k] + t * v
            normal = np.array([-u[1], u[0]]) * np.sign(turn)
            centre = entry + r * normal
            arc = _arc(entry, exit_, centre, r, ds)
            if np.all(field(arc) >= clearance):
                chosen = (entry, exit_, centre, r, np.sign(turn))
                break
            r *= 0.5
        pieces.append(chosen or (pts[k], pts[k], None, 0.0, 0.0))
    dense = [pts[0][None]]
    cursor = pts[0]
    for k, (entry, exit_, centre, r, _) in enumerate(pieces + [(pts[-1], pts[-1], None, 0.0, 0.0)]):
        dense.append(_line(cursor, entry, ds)[1:])
        if centre is not None:
            dense.append(_arc(entry, exit_, centre, r, ds)[1:])
        cursor = exit_
    out = np.vstack(dense)
    keep = np.r_[True, np.linalg.norm(np.diff(out, axis=0), axis=1) > 1e-9]
    return out[keep]


def _line(a, b, ds):
    n = max(1, int(np.ceil(np.linalg.norm(b - a) / ds)))
    return a + np.linspace(0, 1, n + 1)[:, None] * (b - a)


def _arc(entry, exit_, centre, r, ds):
    a0 = np.arctan2(*(entry - centre)[::-1])
    a1 = np.arctan2(*(exit_ - centre)[::-1])
    sweep = wrap(a1 - a0)
    n = max(2, int(np.ceil(abs(sweep) * r / ds)))
    ang = a0 + np.linspace(0, 1, n + 1) * sweep
    return centre + r * np.c_[np.cos(ang), np.sin(ang)]


def resample(points, ds):
    seg = np.linalg.norm(np.diff(points, axis=0), axis=1)
    s = np.r_[0.0, np.cumsum(seg)]
    grid = np.arange(0.0, s[-1], ds)
    grid = np.r_[grid, s[-1]] if s[-1] - grid[-1] > 1e-9 else grid
    return np.c_[np.interp(grid, s, points[:, 0]), np.interp(grid, s, points[:, 1])], grid


class CarryTrajectory:
    """Base pose along a path, with the carried point (base frame `offset`)
    riding on the path and the base centre trailing on it.

    path: dense (N, 2) samples from the base's start position to the goal.
    offset: (a, b), the carried point in the base frame (a forward).
    footprint: (M, 2) base-frame points outlining robot + object.
    """

    def __init__(self, path, offset, ds=0.01):
        self.P, self.s = resample(np.asarray(path, float), ds)
        self.ds = ds
        a, b = offset
        self.L = float(np.hypot(a, b))
        self.bias = float(np.arctan2(b, a))   # yaw = chord heading - bias
        P, s = self.P, self.s
        # For each object sample k, the base sample is the last one at chord
        # distance >= L behind it (then refined by interpolation).
        start = int(np.searchsorted(np.linalg.norm(P - P[0], axis=1) >= self.L, True))
        self.valid = start < len(P)
        if not self.valid:
            return
        idx = np.arange(start, len(P))
        sb = np.zeros(len(idx))
        j = 0
        for n, k in enumerate(idx):
            # advance j while the next base sample is still >= L behind k
            while j + 1 < k and np.linalg.norm(P[k] - P[j + 1]) >= self.L:
                j += 1
            d0, d1 = np.linalg.norm(P[k] - P[j]), np.linalg.norm(P[k] - P[j + 1])
            f = 0.0 if d0 <= d1 else np.clip((d0 - self.L) / (d0 - d1), 0.0, 1.0)
            sb[n] = s[j] + f * (s[j + 1] - s[j])
        self.sg = s[idx]                       # object arc length
        self.sb = np.maximum.accumulate(sb)    # base arc length
        self.obj = P[idx]
        self.base = np.c_[np.interp(self.sb, s, P[:, 0]), np.interp(self.sb, s, P[:, 1])]
        chord = self.obj - self.base
        self.yaw = np.unwrap(np.arctan2(chord[:, 1], chord[:, 0]) - self.bias)
        tang = np.gradient(P, axis=0)[idx]
        # Direction the carried point moves in the world: the path tangent.
        self.psi = np.unwrap(np.arctan2(tang[:, 1], tang[:, 0]))

    def pose(self, sg):
        """(x, y, yaw, psi) at object arc length sg (clipped to the range)."""
        x = np.interp(sg, self.sg, self.base[:, 0])
        y = np.interp(sg, self.sg, self.base[:, 1])
        return x, y, np.interp(sg, self.sg, self.yaw), np.interp(sg, self.sg, self.psi)

    def collisions(self, field, footprint, margin, stride=1):
        """Indices of object samples where the footprint comes within margin of
        an obstacle."""
        bad = []
        fp = np.asarray(footprint, float)
        for k in range(0, len(self.sg), stride):
            c, s_ = np.cos(self.yaw[k]), np.sin(self.yaw[k])
            world = fp @ np.array([[c, s_], [-s_, c]]) + self.base[k]
            if np.min(field(world)) < margin:
                bad.append(k)
        return bad

    def speed_profile(self, v_obj, v_base, yaw_rate, roll_rate, accel, v_end=0.0, v_min=0.002):
        """Largest object speed (m per step) at each sample under the per-step
        limits, slowing down in time for every later limit (backward pass)
        and to v_end at the end. Speeding up is left to the controller, which
        starts from rest; self.v also includes that forward pass, for
        duration()."""
        ds = np.gradient(self.sg)
        ds[ds < 1e-9] = 1e-9
        rate = lambda f: np.abs(np.gradient(f)) / ds   # per metre of object travel
        base_ratio = np.linalg.norm(np.gradient(self.base, axis=0), axis=1) / ds
        roll = rate(self.psi - self.yaw)
        cap = np.full(len(ds), float(v_obj))
        with np.errstate(divide="ignore"):
            cap = np.minimum(cap, v_base / np.maximum(base_ratio, 1e-9))
            cap = np.minimum(cap, yaw_rate / np.maximum(rate(self.yaw), 1e-9))
            cap = np.minimum(cap, roll_rate / np.maximum(roll, 1e-9))
        cap = np.maximum(cap, v_min)
        back = cap.copy()
        back[-1] = min(back[-1], v_end)
        for k in range(len(back) - 2, -1, -1):   # v^2 = v1^2 + 2 a s, a in m/step^2
            back[k] = min(back[k], np.sqrt(back[k + 1] ** 2 + 2 * accel * (self.sg[k + 1] - self.sg[k])))
        v = back.copy()
        v[0] = 0.0
        for k in range(1, len(v)):
            v[k] = min(v[k], np.sqrt(v[k - 1] ** 2 + 2 * accel * (self.sg[k] - self.sg[k - 1])))
        self.cap, self.back, self.v = cap, back, v
        return back

    def duration(self):
        """Approximate steps to traverse at the profile speed."""
        v = np.maximum(self.v, 1e-4)
        return float(np.sum(np.diff(self.sg) / (0.5 * (v[1:] + v[:-1]))))


def spin_clear(field, footprint, position, yaw0, yaw1, margin, step=0.01):
    """True when turning on the spot from yaw0 to yaw1 (shortest way given by
    the sign of yaw1 - yaw0) keeps the footprint `margin` from obstacles."""
    fp = np.asarray(footprint, float)
    n = max(2, int(np.ceil(abs(yaw1 - yaw0) / step)) + 1)
    for yaw in np.linspace(yaw0, yaw1, n):
        c, s_ = np.cos(yaw), np.sin(yaw)
        if np.min(field(fp @ np.array([[c, s_], [-s_, c]]) + position)) < margin:
            return False
    return True


def outline(polygons, spacing=0.01):
    """Points along the edges of convex polygons (base frame), for collision
    checks against the clearance field."""
    pts = []
    for poly in polygons:
        poly = np.asarray(poly, float)
        if len(poly) == 1:
            pts.append(poly)
            continue
        for a, b in zip(poly, np.roll(poly, -1, axis=0)):
            n = max(1, int(np.ceil(np.linalg.norm(b - a) / spacing)))
            pts.append(a + np.linspace(0, 1, n, endpoint=False)[:, None] * (b - a))
    return np.vstack(pts)
