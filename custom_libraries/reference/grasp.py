"""Antipodal grasps, fixed-base PyBullet IK, and arm RRT-Connect for Fetch.

Dependencies: numpy, scipy, pybullet. Put in custom_libraries/grasp.py.
    planner = GraspPlanner(env, robot_urdf_path='assets/fetch/fetch.urdf')
    plan = planner.plan(env._get_obs())   # call after navigation has stopped
    follower = GraspFollower(planner, plan)
    action = follower.step(env._get_obs())  # pass to env.step(action)

plan() is synchronous: do not step the simulation concurrently. It temporarily
resets joints for collision queries inside saveState/restoreState (also on
failure). IK runs in a separate DIRECT client with ONLY seven arm DOFs. No
simulation step, motor command, or grasp constraint is issued by the planner.
The environment alone executes the returned absolute joint targets.

The default pad dimensions/gap match the supplied fetch.urdf, including its
modified box fingers. Gripper +X is approach; +Y is the closing axis. Positions,
point clouds and normals are world-frame; quaternions are xyzw. Collision
checking is discretized, not a continuous/dynamic safety guarantee.
"""
from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
import tempfile
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

try:
    import pybullet as p
except ImportError:  # Geometry and RRT unit tests can run without the simulator.
    p = None

ARM_NAMES = (
    'shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint',
    'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint',
    'wrist_roll_joint',
)
FINGER_NAMES = ('r_gripper_finger_joint', 'l_gripper_finger_joint')


@dataclass
class GraspConfig:
    friction_coefficient: float = 0.5
    closed_gap: float = 0.00285  # 2*(0.015425 - 0.008 - 0.012/2)
    finger_travel: float = 0.05
    pad_length: float = 0.063
    pad_height: float = 0.024
    tcp_offset: tuple = (0.025, 0.0, 0.0)  # contact near pad tips, ahead of link origin
    opening_clearance: float = 0.008  # total gap, metres
    squeeze: float = 0.001  # total gap reduction in motor command after contact
    min_width: float = 0.005
    max_cloud_points: int = 800
    max_candidates: int = 60
    pregrasp_distance: float = 0.08
    cartesian_step: float = 0.005
    lift_distance: float = 0.08  # set 0 to end after closing
    collision_margin: float = 0.002
    contact_penetration: float = 0.0015
    joint_resolution: float = 0.025  # max joint delta per collision sample, rad
    rrt_step: float = 0.20
    rrt_iterations: int = 2500
    rrt_seconds: float = 1.0  # per candidate, within total budget
    planning_seconds: float = 30.0
    ik_restarts: int = 8
    position_tolerance: float = 0.003
    orientation_tolerance: float = 0.04
    max_approach_joint_jump: float = 0.18
    seed: int = 0


@dataclass
class GraspPlan:
    waypoints: np.ndarray  # full actions in env.joint_indices order
    phases: tuple[str, ...]
    candidate: tuple  # p1, p2, contact centre, orientation, score
    contact_fingers: np.ndarray  # nominal geometric contact positions
    commanded_fingers: np.ndarray  # squeeze target; actual fingers stop on object
    object_position: np.ndarray
    object_orientation: np.ndarray
    stats: dict


class PlanningError(RuntimeError):
    pass


def edge_samples(a, b, resolution):
    """Includes both endpoints, even for a zero-length edge."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(1, int(np.ceil(np.max(np.abs(b - a)) / resolution)))
    return np.linspace(a, b, n + 1)


class RRTConnect:
    """Bidirectional RRT-Connect, bounded joint coordinates, checked edges.

    Continuous roll joints use a finite unwrapped interval around the measured
    start. Returned edges therefore never jump across +/-pi. This is a local
    covering of the joint torus; budget exhaustion does not prove infeasibility.
    """
    def __init__(self, lower, upper, is_valid, *, resolution=0.025,
                 step_size=0.20, max_iterations=2500, rng=None):
        self.lower, self.upper = np.asarray(lower, float), np.asarray(upper, float)
        if (self.lower.shape != self.upper.shape or self.lower.ndim != 1
                or not np.isfinite([self.lower, self.upper]).all()
                or np.any(self.lower >= self.upper)
                or resolution <= 0 or step_size <= 0):
            raise ValueError('Invalid RRT bounds or step size.')
        self.valid = is_valid
        self.resolution, self.step = resolution, step_size
        self.max_iterations = max_iterations
        self.rng = rng if rng is not None else np.random.default_rng(0)

    def edge_valid(self, a, b, deadline=float('inf')):
        return all(time.monotonic() < deadline and self.valid(q)
                   for q in edge_samples(a, b, self.resolution))

    def plan(self, start, goal, *, deadline=float('inf')):
        start, goal = np.asarray(start, float), np.asarray(goal, float)
        for q in (start, goal):
            if (q.shape != self.lower.shape or not np.isfinite(q).all()
                    or np.any(q < self.lower) or np.any(q > self.upper)):
                return None
        if time.monotonic() >= deadline or not self.valid(start) or not self.valid(goal):
            return None
        if self.edge_valid(start, goal, deadline):
            return edge_samples(start, goal, self.resolution)
        # Tree: configurations, parent indices, whether rooted at start.
        a, b = ([start], [-1], True), ([goal], [-1], False)

        def extend(tree, target):
            nodes, parents, _ = tree
            nearest = int(np.argmin(np.linalg.norm(np.asarray(nodes) - target, axis=1)))
            delta = target - nodes[nearest]
            distance = np.linalg.norm(delta)
            if distance < 1e-10:
                return nearest, True
            q = nodes[nearest] + delta * min(1.0, self.step / distance)
            if not self.edge_valid(nodes[nearest], q, deadline):
                return None, False
            nodes.append(q)
            parents.append(nearest)
            return len(nodes) - 1, distance <= self.step

        def trace(tree, index):
            chain = []
            while index >= 0:
                chain.append(tree[0][index])
                index = tree[1][index]
            return chain[::-1]

        for _ in range(self.max_iterations):
            if time.monotonic() >= deadline:
                return None
            sample = b[0][0] if self.rng.random() < 0.15 else self.rng.uniform(self.lower, self.upper)
            ia, _ = extend(a, sample)
            if ia is not None:
                while time.monotonic() < deadline:
                    ib, reached = extend(b, a[0][ia])
                    if ib is None:
                        break
                    if reached:
                        ca, cb = trace(a, ia), trace(b, ib)
                        sparse = ca + cb[-2::-1] if a[2] else cb + ca[-2::-1]
                        # Each constituent edge has already been checked.
                        return np.vstack([edge_samples(u, v, self.resolution)[:-1]
                                          for u, v in zip(sparse[:-1], sparse[1:])]
                                         + [np.asarray(sparse[-1])[None]])
            a, b = b, a
        return None


class _ArmIK:
    """Private kinematic URDF with all non-arm joints frozen at measured q."""
    def __init__(self, planner, measured):
        self.owner, self.client = planner, -1
        root = ET.parse(planner.urdf_path).getroot()
        by_name = dict(zip(planner.action_names, measured))
        for link in root.findall('link'):
            for element in list(link):
                if element.tag in ('visual', 'collision'):
                    link.remove(element)  # IK needs no mesh assets
        for joint in root.findall('joint'):
            name, kind = joint.attrib['name'], joint.attrib['type']
            if name in ARM_NAMES or kind == 'fixed':
                continue
            if kind not in ('revolute', 'continuous', 'prismatic') or name not in by_name:
                raise ValueError(f'Unsupported/unmapped joint: {name}')
            origin = joint.find('origin')
            if origin is None:
                origin = ET.SubElement(joint, 'origin')
            xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
            rot = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' '))
            axis_node = joint.find('axis')
            axis = np.fromstring(axis_node.get('xyz', '1 0 0') if axis_node is not None else '1 0 0', sep=' ')
            axis /= np.linalg.norm(axis)
            if kind == 'prismatic':
                xyz += rot.apply(axis * by_name[name])
            else:
                rot = rot * Rotation.from_rotvec(axis * by_name[name])
            origin.set('xyz', ' '.join(map(str, xyz)))
            origin.set('rpy', ' '.join(map(str, rot.as_euler('xyz'))))
            joint.set('type', 'fixed')
            for element in list(joint):
                if element.tag in ('axis', 'limit', 'dynamics', 'mimic'):
                    joint.remove(element)
        try:
            self.client = p.connect(p.DIRECT)
            if self.client < 0:
                raise PlanningError('Could not create the private IK client.')
            # PyBullet returns the base COM pose; URDF loading uses root link pose.
            pos, quat = p.getBasePositionAndOrientation(planner.robot, physicsClientId=planner.client)
            dyn = p.getDynamicsInfo(planner.robot, -1, physicsClientId=planner.client)
            inv = p.invertTransform(dyn[3], dyn[4])
            pos, quat = p.multiplyTransforms(pos, quat, *inv)
            with tempfile.TemporaryDirectory(prefix='fetch_arm_ik_') as folder:
                path = Path(folder) / 'arm.urdf'
                ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)
                self.robot = p.loadURDF(str(path), pos, quat, useFixedBase=True,
                                        physicsClientId=self.client)
            info = [p.getJointInfo(self.robot, j, physicsClientId=self.client)
                    for j in range(p.getNumJoints(self.robot, physicsClientId=self.client))]
            names = {v[1].decode(): v[0] for v in info}
            self.ids = [names[name] for name in ARM_NAMES]
            dofs = sorted((v for v in info if v[3] >= 0), key=lambda v: v[3])
            if len(dofs) != 7:
                raise PlanningError('Private IK model must contain exactly seven arm DOFs.')
            slots = {v[0]: i for i, v in enumerate(dofs)}
            self.slots = np.array([slots[j] for j in self.ids])
            self.ee = next(v[0] for v in info if v[12].decode() == 'gripper_link')
        except Exception:
            self.close()
            raise

    def close(self):
        if self.client >= 0:
            p.disconnect(physicsClientId=self.client)
            self.client = -1

    def solve(self, position, quaternion, seed, lower, upper, deadline, restarts=None):
        owner, cfg = self.owner, self.owner.config
        restarts = cfg.ik_restarts if restarts is None else restarts
        for attempt in range(restarts):
            q = np.asarray(seed, float).copy() if attempt == 0 else owner.rng.uniform(lower, upper)
            for _ in range(4):
                if time.monotonic() >= deadline:
                    return None
                for j, value in zip(self.ids, q):
                    p.resetJointState(self.robot, j, float(value), physicsClientId=self.client)
                # Convert every null-space vector to Bullet's DOF order.
                order = np.argsort(self.slots)
                result = p.calculateInverseKinematics(
                    self.robot, self.ee, position.tolist(), quaternion.tolist(),
                    lowerLimits=lower[order].tolist(), upperLimits=upper[order].tolist(),
                    jointRanges=(upper - lower)[order].tolist(), restPoses=q[order].tolist(),
                    jointDamping=[0.03] * 7, maxNumIterations=160,
                    residualThreshold=1e-5, physicsClientId=self.client)
                if len(result) != 7:
                    raise PlanningError('Unexpected IK output size.')
                q = np.asarray(result)[self.slots]
                q[owner.wrap] = seed[owner.wrap] + (q[owner.wrap] - seed[owner.wrap] + np.pi) % (2*np.pi) - np.pi
                if not np.isfinite(q).all() or np.any(q < lower) or np.any(q > upper):
                    break  # Never clip an invalid IK solution into a different pose.
                for j, value in zip(self.ids, q):
                    p.resetJointState(self.robot, j, float(value), physicsClientId=self.client)
                state = p.getLinkState(self.robot, self.ee, computeForwardKinematics=True,
                                       physicsClientId=self.client)
                angular_error = (Rotation.from_quat(quaternion).inv() * Rotation.from_quat(state[5])).magnitude()
                if (np.linalg.norm(np.asarray(state[4]) - position) <= cfg.position_tolerance
                        and angular_error <= cfg.orientation_tolerance):
                    return q
        return None


class GraspPlanner:
    def __init__(self, env=None, *, physics_client_id=None,
                 robot_urdf_path='assets/fetch/fetch.urdf', config=None,
                 disabled_self_pairs=()):
        self.config = config or GraspConfig()
        cfg = self.config
        if (cfg.friction_coefficient < 0 or cfg.closed_gap < 0 or cfg.lift_distance < 0
                or cfg.contact_penetration < 0 or cfg.collision_margin < 0
                or cfg.squeeze < 0 or cfg.opening_clearance <= 2*cfg.collision_margin
                or any(v <= 0 for v in (cfg.finger_travel, cfg.min_width,
                       cfg.cartesian_step, cfg.joint_resolution, cfg.rrt_step,
                       cfg.pregrasp_distance, cfg.planning_seconds, cfg.rrt_seconds,
                       cfg.position_tolerance, cfg.orientation_tolerance,
                       cfg.max_candidates, cfg.max_cloud_points, cfg.ik_restarts))):
            raise ValueError('Invalid grasp geometry, tolerances, or planning budget.')
        if not (0 <= cfg.tcp_offset[0] < cfg.pad_length/2):
            raise ValueError('Contact offset must lie within the finger pad length.')
        if not np.allclose(cfg.tcp_offset[1:], 0):
            raise ValueError('This symmetric gripper requires zero lateral/vertical TCP offsets.')
        self.rng = np.random.default_rng(self.config.seed)
        self.env = env
        self.client = physics_client_id if physics_client_id is not None else getattr(env, 'pb_physics_client', 0)
        self.urdf_path = Path(robot_urdf_path).expanduser().resolve()
        self.sample_stats, self.last_stats = {}, {}
        if env is None:  # Standalone geometric sampler (also usable by navigation).
            return
        if p is None:
            raise ImportError('Install pybullet to plan with a robot.')
        if not self.urdf_path.is_file():
            raise FileNotFoundError(f'Robot URDF not found: {self.urdf_path}')
        self.robot, self.object = env.robot_id, env.object_id
        self.action_ids = list(env.joint_indices)
        self.info = {j: p.getJointInfo(self.robot, j, physicsClientId=self.client)
                     for j in range(p.getNumJoints(self.robot, physicsClientId=self.client))}
        self.action_names = [self.info[j][1].decode() for j in self.action_ids]
        indices = {name: i for i, name in enumerate(self.action_names)}
        self.arm_indices = np.array([indices[name] for name in ARM_NAMES])
        self.finger_indices = np.array([indices[name] for name in FINGER_NAMES])
        self.arm_ids = [self.action_ids[i] for i in self.arm_indices]
        self.finger_ids = [self.action_ids[i] for i in self.finger_indices]
        self.ee = next(j for j, v in self.info.items() if v[12].decode() == 'gripper_link')
        self.lower = np.asarray(env.joint_lower, float).copy()
        self.upper = np.asarray(env.joint_upper, float).copy()
        self.wrap = np.array([self.info[j][9] - self.info[j][8] > 2*np.pi + 0.01
                              or self.info[j][9] < self.info[j][8] for j in self.arm_ids])
        # Standard continuous URDF joints may have lower=0, upper=-1.
        for i, j in enumerate(self.action_ids):
            if self.info[j][2] == p.JOINT_REVOLUTE and self.upper[i] < self.lower[i]:
                self.lower[i], self.upper[i] = -np.inf, np.inf
        self.moving_links = set(self.arm_ids + self.finger_ids)
        changed = True
        while changed:
            before = len(self.moving_links)
            self.moving_links.update(j for j, v in self.info.items() if v[16] in self.moving_links)
            changed = len(self.moving_links) != before
        link_names = {v[12].decode(): j for j, v in self.info.items()}
        link_names[p.getBodyInfo(self.robot, physicsClientId=self.client)[0].decode()] = -1
        self.disabled_pairs = {frozenset((link_names[a], link_names[b])) for a, b in disabled_self_pairs}
        # Treat a fixed cluster as one body; ignore direct joint neighbours only.
        def cluster(j):
            while j >= 0 and self.info[j][2] == p.JOINT_FIXED:
                j = self.info[j][16]
            return j
        clusters = {j: cluster(j) for j in [-1] + list(self.info)}
        adjacent = {frozenset((clusters[j], clusters[v[16]])) for j, v in self.info.items()}
        self.self_pairs = set()
        links = [-1] + list(self.info)
        for k, a in enumerate(links):
            for b in links[k+1:]:
                pair = frozenset((a, b))
                if (self.moving_links.intersection(pair) and clusters[a] != clusters[b]
                        and frozenset((clusters[a], clusters[b])) not in adjacent
                        and pair not in self.disabled_pairs):
                    self.self_pairs.add(pair)
        self._validate_fingers()

    def _validate_fingers(self):
        """Fail explicitly if the supplied model is not the modified Fetch grip."""
        root = ET.parse(self.urdf_path).getroot()
        inner = []
        for name in FINGER_NAMES:
            joint = root.find(f"joint[@name='{name}']")
            link = root.find("link[@name='%s']" % joint.find('child').get('link'))
            collision = link.find('collision')
            box = collision.find('geometry/box')
            if box is None or len(link.findall('collision')) != 1:
                raise ValueError('Expected one box collision shape per finger.')
            xyz = np.fromstring(joint.find('origin').get('xyz'), sep=' ')
            offset = np.fromstring(collision.find('origin').get('xyz'), sep=' ')
            size = np.fromstring(box.get('size'), sep=' ')
            inner.append(abs(xyz[1] + offset[1]) - size[1] / 2)
            if not np.allclose(size[[0, 2]], [self.config.pad_length, self.config.pad_height]):
                raise ValueError('Configure pad_length/pad_height for this URDF.')
        if not np.isclose(sum(inner), self.config.closed_gap):
            raise ValueError('Configure closed_gap for this URDF.')
        if not np.allclose(self.lower[self.finger_indices], 0):
            raise ValueError('Expected zero lower finger limits.')
        self.config.finger_travel = float(min(self.upper[self.finger_indices]))

    def sample_antipodal_grasps(self, object_id=None, num_samples=500, *, points, normals):
        """Sample surface pairs satisfying BOTH outward-normal friction cones.

        Returns ranked (p1,p2,centre,xyzw,score) tuples, compatible with
        NavigateToGrasp. num_samples limits returned poses, not random trials.
        Normals must be outward; the environment already supplies these.
        """
        cfg = self.config
        pts, ns = np.asarray(points, float), np.asarray(normals, float)
        if pts.ndim != 2 or pts.shape[1] != 3 or ns.shape != pts.shape:
            raise ValueError('points and normals must be matching (N,3) arrays.')
        norms = np.linalg.norm(ns, axis=1)
        ok = np.isfinite(pts).all(axis=1) & np.isfinite(ns).all(axis=1) & (norms > 1e-9)
        pts, ns = pts[ok], ns[ok] / norms[ok, None]
        self.sample_stats = {'valid_points': len(pts), 'antipodal_pairs': 0, 'candidates': 0}
        if len(pts) < 2 or num_samples <= 0:
            return []
        cloud = pts.copy()
        if len(pts) > cfg.max_cloud_points:
            ix = self.rng.choice(len(pts), cfg.max_cloud_points, replace=False)
            pts, ns = pts[ix], ns[ix]
        i, j = np.triu_indices(len(pts), 1)
        vectors = pts[j] - pts[i]
        widths = np.linalg.norm(vectors, axis=1)
        ok = (widths >= cfg.min_width) & (widths + cfg.opening_clearance <= cfg.closed_gap + 2*cfg.finger_travel)
        i, j, vectors, widths = i[ok], j[ok], vectors[ok], widths[ok]
        directions = vectors / widths[:, None]
        a = -np.einsum('ij,ij->i', ns[i], directions)
        b = np.einsum('ij,ij->i', ns[j], directions)
        threshold = 1.0 / np.sqrt(1.0 + cfg.friction_coefficient**2)
        keep = (a >= threshold) & (b >= threshold)
        i, j, directions, widths, quality = i[keep], j[keep], directions[keep], widths[keep], np.minimum(a[keep], b[keep])
        self.sample_stats['antipodal_pairs'] = len(i)
        # Include a spread of locations rather than spending the budget on a
        # thousand almost-identical contacts on the same planar face.
        order = np.argsort(-(quality - 0.05*np.abs(directions[:, 2])))
        candidates, seen = [], set()
        for k in order:
            centre = (pts[i[k]] + pts[j[k]]) / 2
            y = directions[k]
            x = np.array([0., 0., -1.])
            x -= y * np.dot(x, y)
            if np.linalg.norm(x) < 1e-6:
                x = np.array([1., 0., 0.]) - y * y[0]
            x /= np.linalg.norm(x)
            for roll in (0., np.pi/6, -np.pi/6, np.pi/2, -np.pi/2):
                approach = Rotation.from_rotvec(y * roll).apply(x)
                for flip in (1., -1.):
                    closing = flip * y
                    rotation = np.column_stack((approach, closing, np.cross(approach, closing)))
                    local = (cloud - centre) @ rotation + np.asarray(cfg.tcp_offset)
                    in_pad = (np.abs(local[:, 0]) <= cfg.pad_length/2) & (np.abs(local[:, 2]) <= cfg.pad_height/2)
                    if np.any(np.abs(local[in_pad, 1]) > widths[k]/2 + 0.002):
                        continue  # Other object points obstruct the proposed opening.
                    quat = Rotation.from_matrix(rotation).as_quat()
                    key = tuple(np.round(np.r_[centre / 0.01, approach / 0.2, closing / 0.2]).astype(int))
                    if key in seen:
                        continue
                    seen.add(key)
                    score = float(quality[k] + 0.10*(-approach[2]))
                    candidates.append((pts[i[k]].copy(), pts[j[k]].copy(), centre.copy(), quat, score))
            if len(candidates) >= num_samples:
                break
        candidates.sort(key=lambda v: v[4], reverse=True)
        self.sample_stats['candidates'] = min(len(candidates), num_samples)
        return candidates[:num_samples]

    @contextmanager
    def _snapshot(self):
        saved = p.saveState(physicsClientId=self.client)
        try:
            yield
        finally:
            try:
                p.restoreState(stateId=saved, physicsClientId=self.client)
            finally:
                p.removeState(stateUniqueId=saved, physicsClientId=self.client)

    def _set(self, q):
        for j, value in zip(self.action_ids, q):
            p.resetJointState(self.robot, j, float(value), physicsClientId=self.client)

    def _valid(self, q, *, touch=False, attached=None, deadline=float('inf')):
        cfg = self.config
        if (time.monotonic() >= deadline or not np.isfinite(q).all()
                or np.any(q < self.lower - 1e-8) or np.any(q > self.upper + 1e-8)):
            return False
        self._set(q)
        if attached is not None:
            ee = p.getLinkState(self.robot, self.ee, computeForwardKinematics=True, physicsClientId=self.client)
            pos, orn = p.multiplyTransforms(ee[4], ee[5], *attached)
            p.resetBasePositionAndOrientation(self.object, pos, orn, physicsClientId=self.client)
        for body in self.obstacles:
            contacts = p.getClosestPoints(self.robot, body, cfg.collision_margin, physicsClientId=self.client)
            for contact in contacts:
                if contact[3] not in self.moving_links and not (attached is not None and body == self.object):
                    continue  # Parked base/floor contact cannot change in arm-only motion.
                if body == self.object and touch and contact[3] in self.finger_ids:
                    if contact[8] >= -cfg.contact_penetration:
                        continue
                return False
        for contact in p.getClosestPoints(self.robot, self.robot, cfg.collision_margin, physicsClientId=self.client):
            if frozenset((contact[3], contact[4])) in self.self_pairs:
                return False
        if attached is not None:
            # Include the payload, not only the fingers, in the lift sweep.
            for body in self.obstacles:
                if body == self.object:
                    continue
                for contact in p.getClosestPoints(self.object, body, cfg.collision_margin, physicsClientId=self.client):
                    # Permit only shallow, pre-existing support contact; never
                    # permit a deeper penetration while lifting off the table.
                    allowed = self.support_contacts.get((body, contact[4]))
                    if allowed is not None and contact[8] >= allowed - 1e-4:
                        continue
                    return False
        return True

    def _full(self, parked, arm, fingers):
        q = parked.copy()
        q[self.arm_indices], q[self.finger_indices] = arm, fingers
        return q

    def _live_pose_ok(self, q, position, quaternion):
        # Guard against a source URDF that differs from the loaded robot.
        self._set(q)
        state = p.getLinkState(self.robot, self.ee, computeForwardKinematics=True,
                               physicsClientId=self.client)
        error = (Rotation.from_quat(quaternion).inv()
                 * Rotation.from_quat(state[5])).magnitude()
        return (np.linalg.norm(np.asarray(state[4])-position) <= self.config.position_tolerance
                and error <= self.config.orientation_tolerance)

    def _cartesian(self, ik, start_pos, end_pos, quat, seed, parked,
                   fingers, lower, upper, deadline, *, touch=False, attached=None):
        n = max(1, int(np.ceil(np.linalg.norm(end_pos-start_pos) / self.config.cartesian_step)))
        path, previous = [], seed.copy()
        for position in np.linspace(start_pos, end_pos, n+1)[1:]:
            q = ik.solve(position, quat, previous, lower, upper, deadline, restarts=1)
            if q is None or np.max(np.abs(q-previous)) > self.config.max_approach_joint_jump:
                return None
            if not self._live_pose_ok(self._full(parked, q, fingers), position, quat):
                raise PlanningError('IK model and loaded robot disagree; check the URDF path/scaling.')
            for mid in edge_samples(previous, q, self.config.joint_resolution):
                if not self._valid(self._full(parked, mid, fingers), touch=touch,
                                   attached=attached, deadline=deadline):
                    return None
            path.append(q)
            previous = q
        return path

    def plan(self, obs):
        """Plan from measured post-navigation state; return GraspPlan or None.

        None means the finite sampling/IK/RRT budget failed, not that the parked
        pose is proven unreachable. last_stats explains the rejection stage.
        """
        if self.env is None:
            raise ValueError('Construct GraspPlanner(env) before calling plan().')
        cfg = self.config
        parked = np.asarray(obs['qpos'], float).copy()
        if parked.shape != (len(self.action_ids),) or not np.isfinite(parked).all():
            raise ValueError('Invalid observed qpos.')
        actual = np.array([s[0] for s in p.getJointStates(self.robot, self.action_ids, physicsClientId=self.client)])
        if not np.allclose(actual, parked, atol=1e-4, rtol=0):
            raise PlanningError('Observation is stale; acquire it after navigation stops.')
        deadline = time.monotonic() + cfg.planning_seconds
        stats = self.last_stats = dict(candidates=0, tried=0, ik_fail=0, transit_fail=0,
                                      approach_fail=0, closure_fail=0, lift_fail=0, reason='search_exhausted')
        candidates = self.sample_antipodal_grasps(points=obs['object_pc'], normals=obs['object_normals'])
        stats['candidates'] = len(candidates)
        if not candidates:
            stats['reason'] = 'no_antipodal_pairs'
            return None
        start = parked[self.arm_indices]
        lower, upper = self.lower[self.arm_indices].copy(), self.upper[self.arm_indices].copy()
        lower[self.wrap] = np.maximum(lower[self.wrap], start[self.wrap] - np.pi)
        upper[self.wrap] = np.minimum(upper[self.wrap], start[self.wrap] + np.pi)
        self.obstacles = [p.getBodyUniqueId(i, physicsClientId=self.client)
                          for i in range(p.getNumBodies(physicsClientId=self.client))]
        self.obstacles.remove(self.robot)
        ik = _ArmIK(self, parked)
        try:
            with self._snapshot():
                object_pose = p.getBasePositionAndOrientation(self.object, physicsClientId=self.client)
                self.support_contacts = {}
                for body in self.obstacles:
                    if body != self.object:
                        for c in p.getClosestPoints(self.object, body, cfg.collision_margin, physicsClientId=self.client):
                            if c[7][2] > 0.7 and c[8] >= -cfg.contact_penetration:
                                key = (body, c[4])
                                self.support_contacts[key] = min(self.support_contacts.get(key, 0.), c[8])
                for candidate in candidates[:cfg.max_candidates]:
                    if time.monotonic() >= deadline:
                        stats['reason'] = 'time_budget'
                        break
                    p.resetBasePositionAndOrientation(self.object, *object_pose, physicsClientId=self.client)
                    stats['tried'] += 1
                    p1, p2, centre, quat, _ = candidate
                    width = np.linalg.norm(p2-p1)
                    nominal = np.full(2, (width-cfg.closed_gap)/2)
                    opened = nominal + cfg.opening_clearance/2
                    squeezed = np.maximum(self.lower[self.finger_indices], nominal - cfg.squeeze/2)
                    if np.any(opened > self.upper[self.finger_indices]):
                        continue
                    opening = self._full(parked, start, opened)
                    if not all(self._valid(q, deadline=deadline) for q in edge_samples(parked, opening, 0.002)):
                        stats['closure_fail'] += 1
                        continue
                    rot = Rotation.from_quat(quat)
                    grasp_pos = centre - rot.apply(cfg.tcp_offset)
                    pre_pos = grasp_pos - cfg.pregrasp_distance*rot.as_matrix()[:, 0]
                    pre = ik.solve(pre_pos, quat, start, lower, upper, deadline)
                    if pre is None:
                        stats['ik_fail'] += 1
                        continue
                    if not self._live_pose_ok(self._full(parked, pre, opened), pre_pos, quat):
                        raise PlanningError('IK model and loaded robot disagree; check the URDF path/scaling.')
                    valid = lambda arm: self._valid(self._full(parked, arm, opened), deadline=deadline)
                    rrt = RRTConnect(lower, upper, valid, resolution=cfg.joint_resolution,
                                     step_size=cfg.rrt_step, max_iterations=cfg.rrt_iterations, rng=self.rng)
                    transit = rrt.plan(start, pre, deadline=min(deadline, time.monotonic()+cfg.rrt_seconds))
                    if transit is None:
                        stats['transit_fail'] += 1
                        continue
                    approach = self._cartesian(ik, pre_pos, grasp_pos, quat, pre, parked,
                                               opened, lower, upper, deadline)
                    if approach is None:
                        stats['approach_fail'] += 1
                        continue
                    reach = approach[-1]
                    # Check closing up to nominal contact. Squeeze is a force-
                    # generating controller target, not a collision-free pose.
                    if not all(self._valid(self._full(parked, reach, f), touch=True, deadline=deadline)
                               for f in edge_samples(opened, nominal, 0.001)):
                        stats['closure_fail'] += 1
                        continue
                    lift = []
                    if cfg.lift_distance > 0:
                        self._set(self._full(parked, reach, nominal))
                        ee = p.getLinkState(self.robot, self.ee, computeForwardKinematics=True, physicsClientId=self.client)
                        inv = p.invertTransform(ee[4], ee[5])
                        attached = p.multiplyTransforms(*inv, *object_pose)
                        lift = self._cartesian(ik, grasp_pos, grasp_pos + [0, 0, cfg.lift_distance],
                                               quat, reach, parked, nominal, lower, upper, deadline,
                                               touch=True, attached=attached)
                        if lift is None:
                            stats['lift_fail'] += 1
                            continue
                    waypoints = [opening]
                    phases = ['open']
                    for phase, path, fingers in [('transit', transit[1:], opened),
                                                  ('approach', approach, opened),
                                                  ('close', [reach], squeezed),
                                                  ('lift', lift, squeezed)]:
                        for arm in path:
                            waypoints.append(self._full(parked, arm, fingers))
                            phases.append(phase)
                    stats['reason'] = 'success'
                    return GraspPlan(np.asarray(waypoints), tuple(phases), candidate,
                                     nominal, squeezed, np.asarray(obs['object_pos']).copy(),
                                     np.asarray(object_pose[1]).copy(), stats.copy())
        finally:
            ik.close()
        return None


class GraspFollower:
    """Measured-state waypoint follower; no planning or teleportation in step().

    status: running / done / failed; error explains failure. On failure hold
    measured arm/base and retain a closed grasp if already closing/carrying.
    Do not automatically navigate away with an extended arm after failure.
    """
    def __init__(self, planner, plan):
        self.planner, self.plan = planner, plan
        self.index = self.ticks = self.stable = 0
        self.status, self.error = 'running', None
        self.hold = None
        self.pinned = np.array([i for i in range(len(planner.action_ids))
                                if i not in set(planner.arm_indices) | set(planner.finger_indices)])
        self.contact_stable = 0
        self.lost_contact_ticks = 0

    def _contacts(self):
        owner = self.planner
        contacts = p.getContactPoints(owner.robot, owner.object, physicsClientId=owner.client)
        # Opposite finger contacts, positive normal force; palm contact alone
        # is not evidence that an object has been grasped.
        return set(owner.finger_ids).issubset({c[3] for c in contacts if c[9] > 0.01})

    def _fail(self, message, measured):
        self.status, self.error = 'failed', message
        self.hold = measured.copy()
        if self.plan.phases[self.index] in ('close', 'lift'):
            self.hold[self.planner.finger_indices] = self.plan.commanded_fingers
        return self.hold.copy()

    def step(self, obs):
        owner = self.planner
        q = np.asarray(obs['qpos'], float)
        if self.status != 'running':
            return self.hold.copy()
        target = self.plan.waypoints[self.index]
        phase = self.plan.phases[self.index]
        self.ticks += 1
        if np.max(np.abs(q[self.pinned]-self.plan.waypoints[0, self.pinned])) > 0.03:
            return self._fail('Base/torso/head moved away from the planned parked state.', q)
        if phase in ('open', 'transit', 'approach'):
            rotated = (Rotation.from_quat(self.plan.object_orientation).inv()
                       * Rotation.from_quat(obs['object_xyzw'])).magnitude()
            if np.linalg.norm(np.asarray(obs['object_pos'])-self.plan.object_position) > 0.012 or rotated > 0.10:
                return self._fail('Object moved; stop and obtain a fresh plan.', q)
        # Common interpolation factor keeps commanded targets on the checked
        # joint-space segment; independent component clipping would bend it.
        limits = np.full(len(q), 0.015)
        limits[owner.finger_indices] = 0.0004 if phase == 'close' else 0.002
        error = target-q
        scale = max(1., float(np.max(np.abs(error)/limits)))
        action = q + error/scale
        action[self.pinned] = target[self.pinned]
        if phase == 'lift':
            action[owner.finger_indices] = target[owner.finger_indices]
        reached = np.max(np.abs(error[owner.arm_indices])) < 0.008
        if phase == 'open':
            reached &= np.max(np.abs(error[owner.finger_indices])) < 0.001
        elif phase == 'close':
            self.contact_stable = self.contact_stable + 1 if self._contacts() else 0
            reached &= self.contact_stable >= 12 and self.ticks >= 30
        elif phase == 'lift':
            self.lost_contact_ticks = 0 if self._contacts() else self.lost_contact_ticks + 1
            if self.lost_contact_ticks > 20:
                return self._fail('Lost bilateral finger contact during lift.', q)
        if self.ticks > 600:
            return self._fail(f'Timed out during {phase} at waypoint {self.index}.', q)
        if reached:
            if self.index + 1 == len(self.plan.waypoints):
                if not self._contacts():
                    return self._fail('No bilateral finger contact at completion.', q)
                if phase == 'lift' and float(obs['object_pos'][2]) < self.plan.object_position[2] + 0.5*owner.config.lift_distance:
                    return self._fail('Arm reached lift pose but the object did not rise.', q)
                self.status = 'done'
                self.hold = target.copy()
            else:
                self.index += 1
                self.ticks = self.contact_stable = 0
        return np.clip(action, owner.lower, owner.upper)


# Name alias for geometric sampling callers. Use plan(obs), not the old
# select_optimal_grasp/make_collision_checker interface, for motion planning.
AntipodalGraspPlanner = GraspPlanner
