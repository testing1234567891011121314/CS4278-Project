"""Collision-checked quadratic corner rounding for a fixed-yaw XY base.

Integration in solver.py:
    from curved_navigation import CurvedNavigator

After create_navigable_map() in Solver.__init__:
    self.curved_nav = CurvedNavigator(self)

Keep A* and its self.path/waypoint_index initialization. Replace the entire
`if self.path:` movement block in get_action() with:
    self.action[:2] = qpos[:2]   # hold measured XY on failure/completion
    if self.path:
        step_xy = self.curved_nav.step(base_xy, self.path)
        self.forward = step_xy[0] / self.action_scale[0]
        self.turn = step_xy[1] / self.action_scale[1]
        self.waypoint_index = self.curved_nav.grid_index

Keep the movement resets and unit_action calculation. Do not run the old
straight-waypoint controller as well. Grid path stays in self.path; world
curve samples are in self.curved_nav.points. Call self.curved_nav.reset()
after editing the map or replanning in place. A new path object resets it
automatically. No PyBullet calls or simulator state changes are made here.

Requires numpy only. Velocity is not required; this is a geometric position
target follower, not a speed/acceleration controller. Collision guarantees
apply to the planned geometry on the supplied inflated grid, not to physical
tracking errors, dynamic obstacles, or links absent from that footprint.
"""

import numpy as np


class CurvedNavigator:
    def __init__(self, solver, corner_distance=0.35, max_lookahead=0.5,
                 tracking_error=0.02, goal_tolerance=0.03):
        self.solver = solver
        self.corner_distance = float(corner_distance)
        self.max_lookahead = float(max_lookahead)
        self.tracking_error = float(tracking_error)
        self.goal_tolerance = float(goal_tolerance)
        if min(corner_distance, max_lookahead, tracking_error,
               goal_tolerance) <= 0:
            raise ValueError('Navigation parameters must be positive.')
        self.reset()

    def reset(self):
        self._path = None
        self.points = np.empty((0, 2))
        self.progress = 0.0
        self.grid_index = 0
        self.status = 'unplanned'

    def segment_free(self, a, b, padding=0.0):
        """Check every cell touched by the closed segment (including corners).

        Padding expands each blocked cell, used to enclose curve/chord error.
        Unlike point sampling, this cannot skip a briefly crossed blocked cell.
        """
        s = self.solver
        grid = s.inflated_occupancy
        rows, cols = grid.shape
        a, b = np.asarray(a, float), np.asarray(b, float)
        a = np.array([(a[0]-s.x_min)/s.cell_size,
                      (s.y_max-a[1])/s.cell_size])
        b = np.array([(b[0]-s.x_min)/s.cell_size,
                      (s.y_max-b[1])/s.cell_size])
        pad = padding / s.cell_size + 1e-9
        low, high = np.minimum(a, b)-pad, np.maximum(a, b)+pad
        if np.any(low < 0) or np.any(high >= [cols, rows]):
            return False
        c0, r0 = np.floor(low).astype(int)
        c1, r1 = np.floor(high).astype(int)
        occupied = np.argwhere(grid[r0:r1+1, c0:c1+1] != 0)
        delta = b-a
        for rr, cc in occupied:
            lo = np.array([cc+c0, rr+r0], float)-pad
            hi = lo+1+2*pad
            enter, leave = 0.0, 1.0
            for axis in range(2):
                if abs(delta[axis]) < 1e-14:
                    if a[axis] < lo[axis] or a[axis] > hi[axis]:
                        enter, leave = 1.0, 0.0
                        break
                else:
                    t0 = (lo[axis]-a[axis])/delta[axis]
                    t1 = (hi[axis]-a[axis])/delta[axis]
                    enter = max(enter, min(t0, t1))
                    leave = min(leave, max(t0, t1))
            if enter <= leave:
                return False
        return True

    def _curve(self, a, control, b, depth=0):
        # Exact bound between quadratic Bezier and its parameterized chord.
        error = np.linalg.norm(control-(a+b)/2)/2
        tolerance = min(0.002, self.solver.cell_size/20)
        if error <= tolerance:
            return [a, b] if self.segment_free(a, b, error) else None
        if depth >= 16:
            return None
        left, right = (a+control)/2, (control+b)/2
        middle = (left+right)/2
        first = self._curve(a, left, middle, depth+1)
        if first is None:
            return None
        second = self._curve(middle, right, b, depth+1)
        return None if second is None else first[:-1]+second

    def build(self, base, path):
        self.reset()
        self._path = path
        self._grid_world = np.asarray([
            self.solver.convert_to_world(cell) for cell in path
        ], dtype=float).reshape(-1, 2)
        if not len(path):
            self.status = 'no_path'
            return
        raw = [np.asarray(base, float)]
        for point in self._grid_world:
            if np.linalg.norm(point-raw[-1]) > 1e-9:
                raw.append(point)
        # Check the start connection as well as every A* edge.
        if any(not self.segment_free(a, b) for a, b in zip(raw, raw[1:])):
            self.status = 'invalid_grid_path_or_start'
            return
        # Remove only redundant collinear points, preserving route topology.
        route = []
        for point in raw:
            while len(route) >= 2:
                u, v = route[-1]-route[-2], point-route[-1]
                cross = u[0]*v[1]-u[1]*v[0]
                if abs(cross) > 1e-10 or np.dot(u, v) <= 0:
                    break
                route.pop()
            route.append(point)
        result = [route[0]]
        self.rounded_corners = 0
        for i in range(1, len(route)-1):
            previous, corner, following = route[i-1:i+2]
            incoming, outgoing = corner-previous, following-corner
            l1, l2 = np.linalg.norm(incoming), np.linalg.norm(outgoing)
            cut = min(self.corner_distance, 0.4*l1, 0.4*l2)
            accepted = None
            for _ in range(8):
                entry = corner-cut*incoming/l1
                exit_point = corner+cut*outgoing/l2
                curve = self._curve(entry, corner, exit_point)
                if curve is not None and self.segment_free(result[-1], entry):
                    accepted = curve
                    break
                cut *= 0.5
            if accepted is None:
                result.append(corner)
            else:
                result.extend(accepted)
                self.rounded_corners += 1
        if len(route) > 1:
            result.append(route[-1])
        points = [result[0]]
        for point in result[1:]:
            if np.linalg.norm(point-points[-1]) > 1e-9:
                points.append(point)
        self.points = np.asarray(points)
        if any(not self.segment_free(a, b)
               for a, b in zip(self.points, self.points[1:])):
            self.points = np.empty((0, 2))
            self.status = 'smoothing_failed'
            return
        self.arc = np.r_[0.0, np.cumsum(np.linalg.norm(
            np.diff(self.points, axis=0), axis=1))]
        self.status = 'following'

    def point_at(self, distance):
        return np.array([np.interp(distance, self.arc, self.points[:, axis])
                         for axis in range(2)])

    def step(self, base, path):
        base = np.asarray(base, float)
        if self._path is not path:
            self.build(base, path)
        if not len(self.points):
            return np.zeros(2)
        total = self.arc[-1]
        # Advance using measured position; never advance just because commanded.
        best = (float('inf'), self.progress)
        horizon = self.progress + max(1.25, self.max_lookahead*1.5)
        for i in range(len(self.points)-1):
            if self.arc[i+1] < self.progress or self.arc[i] > horizon:
                continue
            a, b = self.points[i:i+2]
            d = b-a
            length = self.arc[i+1]-self.arc[i]
            low = max(0.0, (self.progress-self.arc[i])/length)
            high = min(1.0, (horizon-self.arc[i])/length)
            t = np.clip(np.dot(base-a, d)/np.dot(d, d), low, high)
            candidate = a+t*d
            score = (float(np.linalg.norm(base-candidate)),
                     float(self.arc[i]+t*length))
            if score < best:
                best = score
        self.progress = best[1]
        # Diagnostic index only: curve tracking uses continuous arc progress.
        self.grid_index = int(np.argmin(np.linalg.norm(
            self._grid_world-self.point_at(self.progress), axis=1)))
        if (total-self.progress <= self.goal_tolerance
                and np.linalg.norm(base-self.points[-1]) <= self.goal_tolerance):
            self.status = 'reached'
            return np.zeros(2)
        # Prefer long straight targets; shorten where a chord cuts a curve.
        span = min(self.max_lookahead, total-self.progress)
        for _ in range(12):
            end_s = min(total, self.progress+span)
            target = self.point_at(end_s)
            interior = self.points[(self.arc > self.progress) & (self.arc < end_s)]
            route = np.vstack((self.point_at(self.progress), interior, target))
            d = target-base
            if np.dot(d, d) < 1e-16:
                deviation = 0.0
            else:
                t = np.clip((route-base) @ d / np.dot(d, d), 0, 1)
                deviation = np.linalg.norm(route-(base+t[:, None]*d), axis=1).max()
            if deviation <= self.tracking_error and self.segment_free(base, target):
                self.status = 'following'
                return d
            span *= 0.5
        self.status = 'blocked_or_off_path'
        return np.zeros(2)
