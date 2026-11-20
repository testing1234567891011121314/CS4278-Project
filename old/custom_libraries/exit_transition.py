"""Short, fixed-posture retreat from a conservative blocked exit-map cell.

For the assignment's root_x/root_y prismatic base model. Searches straight
retreats only; finite search failure is not proof of infeasibility. Collision
queries use save/reset/restore, never simulation stepping or motor commands.
Execution returns absolute targets. Sampling is not continuous collision proof.
"""
from collections import deque
from contextlib import contextmanager
import time
import numpy as np
import pybullet as p


class ExitTransitionError(RuntimeError):
    pass


def connected_cells(grid, goal):
    """Four-connected free component; conservative about diagonal corner cuts."""
    free = np.asarray(grid) == 0
    seen = np.zeros_like(free)
    r, c = goal
    if not (0 <= r < free.shape[0] and 0 <= c < free.shape[1] and free[r, c]):
        return seen
    seen[r, c] = True
    queue = deque([(r, c)])
    while queue:
        r, c = queue.popleft()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            a, b = r + dr, c + dc
            if (0 <= a < free.shape[0] and 0 <= b < free.shape[1]
                    and free[a, b] and not seen[a, b]):
                seen[a, b] = True
                queue.append((a, b))
    return seen


def inflate_payload(raw, lower_xy, upper_xy, cell_size, margin=0.05):
    """Additional C-space mask for payload AABB offsets from the base centre."""
    lower = np.asarray(lower_xy) - margin
    upper = np.asarray(upper_xy) + margin
    r0, r1 = int(np.floor(-upper[1]/cell_size)), int(np.ceil(-lower[1]/cell_size))
    c0, c1 = int(np.floor(lower[0]/cell_size)), int(np.ceil(upper[0]/cell_size))
    pad = max(abs(r0), abs(r1), abs(c0), abs(c1)) + 1
    src = np.pad(np.asarray(raw, bool), pad, constant_values=True)
    rows, cols = raw.shape
    blocked = np.zeros((rows, cols), bool)
    for dr in range(r0, r1 + 1):
        for dc in range(c0, c1 + 1):
            blocked |= src[pad+dr:pad+dr+rows, pad+dc:pad+dc+cols]
    return blocked


class RetreatToExit:
    def __init__(self, env, obs, hold_target, to_grid, to_world, *,
                 payload=None, max_distance=0.8, resolution=0.01,
                 margin=0.002, seconds=5.0, max_candidates=160):
        self.env, self.client = env, env.pb_physics_client
        self.robot, self.payload = env.robot_id, payload
        self.to_grid, self.to_world = to_grid, to_world
        self.q0 = np.asarray(obs['qpos'], float).copy()
        self.hold = np.asarray(hold_target, float).copy()
        self.max_distance, self.resolution = max_distance, resolution
        self.margin, self.seconds, self.max_candidates = margin, seconds, max_candidates
        self.lower, self.upper = np.asarray(env.joint_lower), np.asarray(env.joint_upper)
        names = [p.getJointInfo(self.robot, j, physicsClientId=self.client)[1].decode()
                 for j in env.joint_indices]
        if names[:3] != ['root_x_axis_joint', 'root_y_axis_joint', 'root_z_rotation_joint']:
            raise ExitTransitionError('Retreat requires the assignment root x/y/yaw joint order.')
        self.fingers = [i for i, name in enumerate(names) if 'gripper_finger_joint' in name]
        self.torso = names.index('torso_lift_joint')
        # Pin measured posture. Retain force-producing finger commands only.
        self.hold[:13] = self.q0[:13]
        if not np.isfinite(self.q0).all() or not np.isfinite(self.hold).all():
            raise ExitTransitionError('Nonfinite retreat configuration.')
        self.world0 = np.array([self.q0[0] - 2.0, self.q0[1]])
        self.path, self.index, self.ticks = [], 0, 0
        self.done = False
        self.support_links = set()
        for j in range(-1, p.getNumJoints(self.robot, physicsClientId=self.client)):
            name = (p.getBodyInfo(self.robot, physicsClientId=self.client)[0].decode() if j == -1
                    else p.getJointInfo(self.robot, j, physicsClientId=self.client)[12].decode())
            if name in ('base_link', 'r_wheel_link', 'l_wheel_link'):
                self.support_links.add(j)
        excluded = {self.robot, getattr(env, 'goal_marker_id', None), payload}
        self.obstacles = [p.getBodyUniqueId(i, physicsClientId=self.client)
                          for i in range(p.getNumBodies(physicsClientId=self.client))]
        self.obstacles = [b for b in self.obstacles if b not in excluded]
        self.floor_links = set()
        for body in self.obstacles:
            for link in range(-1, p.getNumJoints(body, physicsClientId=self.client)):
                shapes = p.getCollisionShapeData(body, link, physicsClientId=self.client)
                if not shapes:
                    continue
                lo, hi = p.getAABB(body, link, physicsClientId=self.client)
                if hi[2] <= 0.01 or any(s[2] == p.GEOM_PLANE for s in shapes):
                    self.floor_links.add((body, link))
        self.payload_relative = None
        if payload is not None:
            pos, orn = p.getBasePositionAndOrientation(payload, physicsClientId=self.client)
            self.payload_relative = (np.asarray(pos) - np.r_[self.world0, 0.], np.asarray(orn))

    @contextmanager
    def snapshot(self):
        saved = p.saveState(physicsClientId=self.client)
        try:
            yield
        finally:
            try:
                p.restoreState(stateId=saved, physicsClientId=self.client)
            finally:
                p.removeState(stateUniqueId=saved, physicsClientId=self.client)

    def _pose_ok(self, obs):
        q = np.asarray(obs['qpos'], float)
        difference = q - self.q0
        difference[:2] = 0
        difference[2] = (difference[2] + np.pi) % (2*np.pi) - np.pi
        difference[self.fingers] = 0
        if (np.max(np.abs(difference)) > 0.02
                or abs(difference[self.torso]) > 0.003):
            raise ExitTransitionError('Arm/torso/yaw changed; rebuild the exit transition and map.')
        if self.payload_relative is not None:
            xy = np.array([q[0]-2., q[1]])
            actual = np.asarray(obs['object_pos']) - np.r_[xy, 0.]
            quat = np.asarray(obs['object_xyzw'], float).copy()
            quat /= np.linalg.norm(quat)
            expected, expected_quat = self.payload_relative
            angle = 2*np.arccos(np.clip(abs(np.dot(quat, expected_quat)), 0., 1.))
            if np.linalg.norm(actual-expected) > 0.015 or angle > 0.10:
                raise ExitTransitionError('Payload slipped/moved; stop and replan.')

    def _valid_xy(self, xy, live_xy, payload_pose):
        command = np.asarray(xy) + [2., 0.]
        if np.any(command < self.lower[:2]) or np.any(command > self.upper[:2]):
            return False
        for i in range(2):
            p.resetJointState(self.robot, self.env.joint_indices[i], float(command[i]),
                              physicsClientId=self.client)
        if self.payload is not None:
            pos, orn = payload_pose
            moved = np.asarray(pos) + np.r_[np.asarray(xy)-live_xy, 0.]
            p.resetBasePositionAndOrientation(self.payload, moved, orn, physicsClientId=self.client)
        for body in self.obstacles:
            for c in p.getClosestPoints(self.robot, body, self.margin, physicsClientId=self.client):
                # Only ordinary shallow chassis/wheel contact with the floor.
                if ((body, c[4]) in self.floor_links and c[3] in self.support_links
                        and c[7][2] > 0.7 and c[8] >= -0.005):
                    continue
                return False
            if self.payload is not None and p.getClosestPoints(
                    self.payload, body, self.margin, physicsClientId=self.client):
                return False
        # Self geometry and robot/payload relative pose are unchanged by XY translation.
        return True

    def _edge(self, a, b, live_xy, payload_pose, deadline=float('inf')):
        count = max(1, int(np.ceil(np.linalg.norm(np.asarray(b)-a)/self.resolution)))
        for xy in np.linspace(a, b, count+1):
            if time.monotonic() >= deadline or not self._valid_xy(xy, live_xy, payload_pose):
                return False
        return True

    def _payload_pose(self):
        return (p.getBasePositionAndOrientation(self.payload, physicsClientId=self.client)
                if self.payload is not None else None)

    def plan(self, exit_grid, exit_goal):
        """Try short straight retreats into the exit-connected free component."""
        reachable = connected_cells(exit_grid, exit_goal)
        if not reachable.any():
            raise ExitTransitionError('Exit goal is blocked/outside the exit map.')
        start_cell = self.to_grid(self.world0)
        if not (0 <= start_cell[0] < exit_grid.shape[0]
                and 0 <= start_cell[1] < exit_grid.shape[1]):
            raise ExitTransitionError('Retreat start is outside the map.')
        candidates = np.asarray([self.to_world(tuple(c)) for c in np.argwhere(reachable)])
        distances = np.linalg.norm(candidates-self.world0, axis=1)
        candidates = candidates[np.argsort(distances)]
        candidates = [x for x in candidates if np.linalg.norm(x-self.world0) <= self.max_distance]
        payload_pose = self._payload_pose()
        deadline = time.monotonic() + self.seconds
        with self.snapshot():
            if not self._valid_xy(self.world0, self.world0, payload_pose):
                raise ExitTransitionError('Start has a collision/insufficient clearance; no XY escape attempted.')
            if reachable[start_cell]:
                self.done = True
                return
            for target in candidates[:self.max_candidates]:
                if time.monotonic() >= deadline:
                    break
                if self._edge(self.world0, target, self.world0, payload_pose, deadline):
                    self.path = [target.copy()]
                    return
        raise ExitTransitionError('No checked straight retreat within 0.8 m/search budget. '
                                  'A different arm posture or a more general retreat planner is required.')

    def checked_action(self, obs, desired_xy, *, require_grid=None):
        """Check the actual next segment, including after any path smoothing."""
        self._pose_ok(obs)
        q = np.asarray(obs['qpos'], float)
        xy = np.array([q[0]-2., q[1]])
        delta = np.asarray(desired_xy)-xy
        target = xy + delta * min(1., 0.01/max(np.linalg.norm(delta), 1e-12))
        if require_grid is not None:
            count = max(1, int(np.ceil(np.linalg.norm(target-xy)/0.002)))
            for pt in np.linspace(xy, target, count+1):
                cell = self.to_grid(pt)
                if not (0 <= cell[0] < require_grid.shape[0] and 0 <= cell[1] < require_grid.shape[1]
                        and require_grid[cell] == 0):
                    raise ExitTransitionError('Navigator proposed a segment through the blocked exit map.')
        payload_pose = self._payload_pose()
        with self.snapshot():
            if not self._edge(xy, target, xy, payload_pose):
                raise ExitTransitionError('Next base segment collides; holding requires replanning.')
        action = self.hold.copy()
        action[:2] = target + [2., 0.]
        return np.clip(action, self.lower, self.upper)

    def step(self, obs):
        self.ticks += 1
        if self.ticks > 1200:
            raise ExitTransitionError('Retreat tracking timed out.')
        target = self.path[-1] if self.path else self.world0
        action = self.checked_action(obs, target)
        q = np.asarray(obs['qpos'])
        xy = np.array([q[0]-2., q[1]])
        # Completion is measured; never finish just because a target was issued.
        if (np.linalg.norm(xy-target) < 0.003
                and np.max(np.abs(np.asarray(obs['qvel'])[:3])) < 0.02):
            self.done = True
        return action
