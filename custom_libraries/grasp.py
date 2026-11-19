"""Antipodal grasps, fixed-base PyBullet IK, and torso/arm RRT-Connect for Fetch.

Dependencies: numpy, scipy, pybullet. Put in custom_libraries/grasp.py.
    planner = GraspPlanner(env, robot_urdf_path='assets/fetch/fetch.urdf')
    plan = planner.plan(env._get_obs())   # call after navigation has stopped
    follower = GraspFollower(planner, plan)
    action = follower.step(env._get_obs())  # pass to env.step(action)

plan() is synchronous: do not step the simulation concurrently. It temporarily
resets joints for collision queries inside saveState/restoreState (also on
failure). IK runs in a separate DIRECT client with eight DOFs (torso lift + seven arm joints). No
simulation step, motor command, or grasp constraint is issued by the planner.
The environment alone executes the returned absolute joint targets.

The default pad dimensions/gap match the supplied fetch.urdf, including its
modified box fingers. Gripper +X is approach; +Y is the closing axis. Positions,
point clouds and normals are world-frame; quaternions are xyzw. Collision
checking is discretized, not a continuous/dynamic safety guarantee.

Robot planning requires the sibling SRDF (fetch.srdf by default). Pass
robot_srdf_path explicitly if it lives elsewhere. Its named self-collision
exclusions are validated against the URDF and live robot before use. Additional
disabled_self_pairs augment those exclusions. require_srdf=False is an explicit
opt-out for a deliberately SRDF-free setup; geometric sampling needs no SRDF.
plan() validates the measured start once before sampling or IK, and reports
named collisions in last_stats['start_state_error'] if that start is invalid.
"""
from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
import tempfile
import time
import warnings
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
MANIPULATOR_NAMES = ('torso_lift_joint',) + ARM_NAMES
FINGER_NAMES = ('r_gripper_finger_joint', 'l_gripper_finger_joint')


@dataclass
class GraspConfig:
    friction_coefficient: float = 0.5
    closed_gap: float = 0.00285  # 2*(0.015425 - 0.008 - 0.012/2)
    finger_travel: float = 0.05
    pad_length: float = 0.063
    pad_height: float = 0.024
    tcp_offset: tuple = (0.010, 0.0, 0.0)  # contact point ahead of the finger link origin (pad centre)
    # Depths tried in order by plan(). Contacts 6.5 mm from the pad tip (0.025)
    # let the object creep onto the tip edge during the lift and hang there;
    # 0.010 leaves ~21 mm of pad beyond the contact. Fetch's own grasp frame is
    # the pad centre. The shallow depth remains as a fallback for low objects
    # where deep fingertips would hit the table.
    tcp_offsets: tuple = (0.010, 0.025)
    opening_clearance: float = 0.024  # total gap, metres
    min_opening_clearance: float = 0.008  # used when travel cannot fit the full clearance (wide objects)
    # Geometric pre-check in plan(): candidates it flags are tried after all
    # the others (never dropped), in the sampler's order. It flags
    # - a width that leaves less total opening than preferred_opening_clearance.
    #   The collision shape (convex decomposition) is about 3 mm per side
    #   fatter than the point cloud and the check keeps a 2 mm margin, so the
    #   mustard bottle across its 94 mm body (4-5 mm per side) failed the
    #   approach on all 60 candidates;
    # - the point cloud inside the palm box (gripper_link collision mesh in
    #   the gripper frame: x -0.154..-0.032, |y| <= 0.063, |z| <= 0.040) plus
    #   palm_margin, at the current depth. The sampler's obstruction test
    #   covers only the pads;
    # - a finger box or the palm box reaching below the lowest cloud point
    #   (the support surface) at the grasp pose, e.g. a vertical closing axis
    #   on an object lying on the table, which puts one finger in the table.
    preferred_opening_clearance: float = 0.016
    palm_box: tuple = (-0.154, -0.032, 0.063, 0.040)
    palm_margin: float = 0.006
    # When fewer than min_clean_candidates pass the pre-check, sample again
    # with this larger cap (same sampler and scores) and insert the new clean
    # candidates after the first pass's clean ones, before its flagged ones.
    # The mustard bottle had 4 clean candidates of 500 at both depths, all one
    # top-down grasp; at 2000 the shallow depth gives 52 with varied approach
    # tilts. Objects with enough clean candidates never pay for the retry.
    min_clean_candidates: int = 8
    retry_num_samples: int = 2000
    # Rim grasps, generated only when the antipodal sampler finds nothing
    # (bowl, plate: thin shells whose cloud has no antipodal pairs). Around
    # the object's vertical axis, each azimuth sector the pad height wide
    # gives the outermost (rim edge) point; the wall direction there is the
    # principal direction of the sector points within rim_span of it, in the
    # radial-vertical plane. The approach runs along the wall into the object,
    # the closing axis is the wall normal, and the width is the extent of the
    # cloud along the closing axis inside the pad footprint (so a bowl's
    # outer finger clears the lip and the inner one meets the inner wall).
    rim_fallback: bool = True
    rim_azimuths: int = 24
    rim_span: float = 0.025
    rim_depths: tuple = (0.006, 0.012)  # contact centre beyond the rim edge, along the wall
    # Total gap reduction the close phase commands past the point-cloud width.
    # Grip force is linear in it (6.6 N per mm per finger): 0.012 gave ~50 N,
    # which let heavy objects (sugar box, hammer) slide 10-28 mm in the
    # fingers during the lift itself.
    squeeze: float = 0.060
    # Score penalty per metre of distance between the object's centre of mass
    # and the grasp line. Finger contacts have no spinning friction, so an
    # off-centre grasp lets gravity pivot the object out of the fingers.
    com_weight: float = 4.0
    min_width: float = 0.005
    finger_thickness: float = 0.015  # finger body beyond the pad surface, for the obstruction test
    max_cloud_points: int = 800
    max_candidates: int = 60
    pregrasp_distance: float = 0.08
    cartesian_step: float = 0.005
    # The rubric wants the object >= 10 cm above the table; plan 15 cm so the
    # margin covers controller sag and the object settling in the fingers.
    lift_distance: float = 0.15  # set 0 to end after closing
    min_lift_fraction: float = 0.8  # of lift_distance the object must rise for the lift to count
    collision_margin: float = 0.002
    contact_penetration: float = 0.006
    joint_resolution: float = 0.025  # max joint delta per collision sample, rad
    torso_resolution: float = 0.005  # metres per collision sample
    torso_metric_scale: float = 0.20  # metres equivalent to one radian in RRT
    max_approach_torso_jump: float = 0.02  # metres per Cartesian waypoint
    torso_action_step: float = 0.002  # metres per follower call; tune to timestep
    torso_goal_tolerance: float = 0.001  # metres
    # GraspFollower moves a command point ("carrot") along the planned path at
    # these per-call joint speeds instead of converging on every densified
    # waypoint. It only stops (waits for the measured state to converge) at
    # phase ends and at corners of the joint-space path, so the arm never
    # cuts a corner of the collision-checked polyline by more than the
    # corner tolerance. Speeds are per env.step (5 physics substeps).
    follow_arm_step: float = 0.04  # rad per call in open/transit
    follow_torso_step: float = 0.006  # metres per call in open/transit
    follow_cartesian_arm_step: float = 0.01  # rad per call, approach/close
    follow_cartesian_torso_step: float = 0.003  # metres per call, approach/close
    follow_lift_arm_step: float = 0.01  # rad per call while lifting
    follow_lift_torso_step: float = 0.003  # metres per call while lifting
    follow_corner_cos: float = 0.995  # direction cosine below which a waypoint is a corner
    follow_corner_tolerance: float = 0.02  # rad (torso: x torso_metric_scale) at corners
    follow_close_rate: float = 0.0005  # metres per call the finger command closes
    follow_close_stable: int = 12  # ticks of bilateral contact before the lift
    follow_ramp_ticks: int = 8  # carrot speed ramps up over this many calls per phase
    rrt_step: float = 0.20
    rrt_iterations: int = 2500
    # Budgets are CPU seconds and cost no simulation steps. One collision check
    # takes ~30 ms, so even the straight start-to-pregrasp edge (50-70 samples)
    # needs 1-2.5 s on a slow or busy machine; at 1 s per candidate every transit
    # was rejected there and the plan failed or fell back to a worse grasp.
    rrt_seconds: float = 8.0  # per candidate, within total budget
    planning_seconds: float = 60.0  # per grasp depth in tcp_offsets
    ik_restarts: int = 8
    position_tolerance: float = 0.003
    orientation_tolerance: float = 0.04
    max_approach_joint_jump: float = 0.18
    seed: int = 0
    measured_torso_limit_tolerance: float = 0.0002  # metres, measured state only
    measured_finger_limit_tolerance: float = 0.005  # metres, measured state only (2.1 mm seen after a base turn)


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


# Clock for every planning budget (seconds). CPU time of this process, not
# wall time: with wall-clock deadlines a busy machine (parallel trials, a
# loaded grader) ran out of budget after ~20 candidates and returned no plan
# for grasps that plan in a few seconds on an idle one.
planning_clock = time.process_time


class PlanningError(RuntimeError):
    pass


def _validated_link_pairs(pairs, link_ids, *, source):
    """Validate exact link names and canonicalise unordered pairs."""
    result = set()
    for pair in pairs:
        if isinstance(pair, (str, bytes)):
            raise ValueError(f'{source}: expected a pair of link names, got {pair!r}.')
        try:
            names = tuple(pair)
        except TypeError as exc:
            raise ValueError(f'{source}: expected a pair of link names.') from exc
        if len(names) != 2 or any(not isinstance(n, str) or not n.strip() for n in names):
            raise ValueError(f'{source}: each exclusion needs two nonempty link names.')
        if names[0] == names[1]:
            raise ValueError(f'{source}: cannot exclude a link against itself: {names[0]!r}.')
        unknown = [name for name in names if name not in link_ids]
        if unknown:
            raise ValueError(f'{source}: unknown robot link(s): {", ".join(unknown)}.')
        result.add(tuple(sorted(names)))
    return result


def _read_srdf_pairs(path, urdf_path, link_ids):
    """Read only explicit disable_collisions entries; never infer extra pairs."""
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as exc:
        raise ValueError(f'Malformed SRDF {path}: {exc}') from exc
    urdf = ET.parse(urdf_path).getroot()
    robot_name = urdf.get('name')
    if root.tag != 'robot' or not robot_name or root.get('name') != robot_name:
        raise ValueError(
            f'SRDF {path} must have <robot name={robot_name!r}> matching the URDF; '
            f'got <{root.tag} name={root.get("name")!r}>.')
    pairs = [(entry.get('link1'), entry.get('link2'))
             for entry in root.findall('disable_collisions')]
    urdf_links = {link.get('name') for link in urdf.findall('link')}
    # An entry must exist in both the source model and the loaded simulator.
    _validated_link_pairs(pairs, urdf_links, source=f'SRDF {path} / URDF')
    return _validated_link_pairs(pairs, link_ids, source=f'SRDF {path} / live robot')


def edge_samples(a, b, resolution):
    """Includes both endpoints, even for a zero-length edge."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(1, int(np.ceil(np.max(np.abs(b - a) / np.asarray(resolution, float)))))
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
        return all(planning_clock() < deadline and self.valid(q)
                   for q in edge_samples(a, b, self.resolution))

    def plan(self, start, goal, *, deadline=float('inf')):
        start, goal = np.asarray(start, float), np.asarray(goal, float)
        for q in (start, goal):
            if (q.shape != self.lower.shape or not np.isfinite(q).all()
                    or np.any(q < self.lower) or np.any(q > self.upper)):
                return None
        if planning_clock() >= deadline or not self.valid(start) or not self.valid(goal):
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
            if planning_clock() >= deadline:
                return None
            sample = b[0][0] if self.rng.random() < 0.15 else self.rng.uniform(self.lower, self.upper)
            ia, _ = extend(a, sample)
            if ia is not None:
                while planning_clock() < deadline:
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


class _ManipulatorIK:
    """Private kinematic URDF with all non-manipulator joints frozen at measured q."""
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
            if name in MANIPULATOR_NAMES or kind == 'fixed':
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
            self.ids = [names[name] for name in MANIPULATOR_NAMES]
            dofs = sorted((v for v in info if v[3] >= 0), key=lambda v: v[3])
            if len(dofs) != len(MANIPULATOR_NAMES):
                raise PlanningError('Private IK model must contain exactly eight torso/arm DOFs.')
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
                if planning_clock() >= deadline:
                    return None
                for j, value in zip(self.ids, q):
                    p.resetJointState(self.robot, j, float(value), physicsClientId=self.client)
                # Convert every null-space vector to Bullet's DOF order.
                order = np.argsort(self.slots)
                result = p.calculateInverseKinematics(
                    self.robot, self.ee, position.tolist(), quaternion.tolist(),
                    lowerLimits=lower[order].tolist(), upperLimits=upper[order].tolist(),
                    jointRanges=(upper - lower)[order].tolist(), restPoses=q[order].tolist(),
                    jointDamping=[0.03] * len(self.ids), maxNumIterations=160,
                    residualThreshold=1e-5, physicsClientId=self.client)
                if len(result) != len(self.ids):
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
                 disabled_self_pairs=(), robot_srdf_path=None, require_srdf=True):
        self.config = config or GraspConfig()
        cfg = self.config
        torso_settings = np.array([
            cfg.torso_resolution, cfg.torso_metric_scale,
            cfg.max_approach_torso_jump, cfg.torso_action_step,
            cfg.torso_goal_tolerance,
        ], dtype=float)
        if not np.isfinite(torso_settings).all() or np.any(torso_settings <= 0):
            raise ValueError('Torso resolutions, scale and tolerances must be finite and positive.')
        tolerances = np.array([cfg.measured_torso_limit_tolerance,
                               cfg.measured_finger_limit_tolerance], dtype=float)
        if not np.isfinite(tolerances).all() or np.any(tolerances < 0):
            raise ValueError('Measured joint-limit tolerances must be finite and nonnegative.')
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
        self.torso_index = indices['torso_lift_joint']
        self.motion_indices = np.array([indices[name] for name in MANIPULATOR_NAMES])
        self.motion_ids = [self.action_ids[i] for i in self.motion_indices]
        if self.info[self.motion_ids[0]][2] != p.JOINT_PRISMATIC:
            raise ValueError('torso_lift_joint must be prismatic.')
        # Internal motion order is torso first, then the seven arm joints.
        self.motion_resolution = np.r_[cfg.torso_resolution, np.full(7, cfg.joint_resolution)]
        self.motion_scale = np.r_[cfg.torso_metric_scale, np.ones(7)]
        self.finger_indices = np.array([indices[name] for name in FINGER_NAMES])
        self.arm_ids = [self.action_ids[i] for i in self.arm_indices]
        self.finger_ids = [self.action_ids[i] for i in self.finger_indices]
        self.ee = next(j for j, v in self.info.items() if v[12].decode() == 'gripper_link')
        self.lower = np.asarray(env.joint_lower, float).copy()
        self.upper = np.asarray(env.joint_upper, float).copy()
        self.wrap = np.array([self.info[j][2] == p.JOINT_REVOLUTE and (self.info[j][9] - self.info[j][8] > 2*np.pi + 0.01
                              or self.info[j][9] < self.info[j][8]) for j in self.motion_ids])
        # Standard continuous URDF joints may have lower=0, upper=-1.
        for i, j in enumerate(self.action_ids):
            if self.info[j][2] == p.JOINT_REVOLUTE and self.upper[i] < self.lower[i]:
                self.lower[i], self.upper[i] = -np.inf, np.inf
        torso_bounds = [self.lower[self.torso_index], self.upper[self.torso_index]]
        if not np.isfinite(torso_bounds).all() or torso_bounds[0] >= torso_bounds[1]:
            raise ValueError('Torso requires finite, increasing joint limits.')
        # Descendants include the head: it moves in space when the torso rises,
        # even though its own pan/tilt joints remain fixed.
        self.moving_links = set(self.motion_ids + self.finger_ids)
        changed = True
        while changed:
            before = len(self.moving_links)
            self.moving_links.update(j for j, v in self.info.items() if v[16] in self.moving_links)
            changed = len(self.moving_links) != before
        link_names = {v[12].decode(): j for j, v in self.info.items()}
        link_names[p.getBodyInfo(self.robot, physicsClientId=self.client)[0].decode()] = -1
        if len(link_names) != len(self.info) + 1:
            raise ValueError('Live robot link names must be unique for SRDF mapping.')
        self.link_names = {j: name for name, j in link_names.items()}
        self.srdf_path = (Path(robot_srdf_path).expanduser().resolve()
                          if robot_srdf_path is not None else self.urdf_path.with_suffix('.srdf'))
        srdf_pairs = set()
        if self.srdf_path.is_file():
            srdf_pairs = _read_srdf_pairs(self.srdf_path, self.urdf_path, link_names)
        elif robot_srdf_path is not None or require_srdf:
            raise FileNotFoundError(
                f'Robot SRDF not found: {self.srdf_path}. Put the matching SRDF beside '
                'the URDF or pass robot_srdf_path. For an intentionally SRDF-free '
                'model only, use require_srdf=False without an explicit SRDF path.')
        else:
            warnings.warn(
                f'No SRDF loaded ({self.srdf_path}); using adjacency and explicit '
                'disabled_self_pairs only.', RuntimeWarning, stacklevel=2)
            self.srdf_path = None
        explicit_pairs = _validated_link_pairs(
            disabled_self_pairs, link_names, source='disabled_self_pairs')
        self.disabled_pairs = {frozenset((link_names[a], link_names[b]))
                               for a, b in srdf_pairs | explicit_pairs}
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
        self._self_pair_list = sorted(tuple(sorted(pair)) for pair in self.self_pairs)
        self.collision_policy = {
            'srdf_path': str(self.srdf_path) if self.srdf_path is not None else None,
            'srdf_pair_count': len(srdf_pairs),
            'explicit_pair_count': len(explicit_pairs),
            'disabled_pair_count': len(self.disabled_pairs),
            'checked_self_pair_count': len(self.self_pairs),
        }
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

    def sample_antipodal_grasps(self, object_id=None, num_samples=500, *, points, normals, com=None):
        """Sample surface pairs satisfying BOTH outward-normal friction cones.

        Returns ranked (p1,p2,centre,xyzw,score) tuples, compatible with
        NavigateToGrasp. num_samples limits returned poses, not random trials.
        Normals must be outward; the environment already supplies these.
        com is the object's centre of mass (world); candidates whose grasp line
        passes far from it are ranked lower. Defaults to the cloud centroid.
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
        cloud, cloud_normals = pts.copy(), ns.copy()
        if len(pts) > cfg.max_cloud_points:
            ix = self.rng.choice(len(pts), cfg.max_cloud_points, replace=False)
            pts, ns = pts[ix], ns[ix]
        i, j = np.triu_indices(len(pts), 1)
        vectors = pts[j] - pts[i]
        widths = np.linalg.norm(vectors, axis=1)
        ok = (widths >= cfg.min_width) & (widths + cfg.min_opening_clearance <= cfg.closed_gap + 2*cfg.finger_travel)
        i, j, vectors, widths = i[ok], j[ok], vectors[ok], widths[ok]
        directions = vectors / widths[:, None]
        a = -np.einsum('ij,ij->i', ns[i], directions)
        b = np.einsum('ij,ij->i', ns[j], directions)
        threshold = 1.0 / np.sqrt(1.0 + cfg.friction_coefficient**2)
        keep = (a >= threshold) & (b >= threshold)
        i, j, directions, widths, quality = i[keep], j[keep], directions[keep], widths[keep], np.minimum(a[keep], b[keep])
        self.sample_stats['antipodal_pairs'] = len(i)
        com = cloud.mean(axis=0) if com is None else np.asarray(com, float)
        rel = com - pts[i]
        com_dist = np.linalg.norm(rel - np.einsum('ij,ij->i', rel, directions)[:, None]*directions, axis=1)
        # Include a spread of locations rather than spending the budget on a
        # thousand almost-identical contacts on the same planar face.
        order = np.argsort(-(quality - 0.05*np.abs(directions[:, 2]) - cfg.com_weight*com_dist))
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
                    # Only points the open fingers would sweep through obstruct the grasp:
                    # just outside the contact span, within opening clearance + finger
                    # thickness. Points further out (e.g. the far wall of a bowl when
                    # pinching its rim) are beyond the fingers and do not collide.
                    side = np.abs(local[in_pad, 1])
                    reach = widths[k]/2 + cfg.opening_clearance/2 + cfg.finger_thickness
                    if np.any((side > widths[k]/2 + 0.002) & (side < reach)):
                        continue  # Other object points obstruct the proposed opening.
                    quat = Rotation.from_matrix(rotation).as_quat()
                    key = tuple(np.round(np.r_[centre / 0.01, approach / 0.2, closing / 0.2]).astype(int))
                    if key in seen:
                        continue
                    seen.add(key)
                    score = float(quality[k] + 0.10*(-approach[2]) - cfg.com_weight*com_dist[k])
                    candidates.append((pts[i[k]].copy(), pts[j[k]].copy(), centre.copy(), quat, score))
            if len(candidates) >= num_samples:
                break
        if not candidates and cfg.rim_fallback:
            candidates = self.sample_rim_grasps(cloud, cloud_normals)
            self.sample_stats['rim_candidates'] = len(candidates)
        candidates.sort(key=lambda v: v[4], reverse=True)
        self.sample_stats['candidates'] = min(len(candidates), num_samples)
        return candidates[:num_samples]

    def sample_rim_grasps(self, points, normals):
        """Rim candidates (see GraspConfig.rim_fallback), as sampler tuples."""
        cfg = self.config
        pts, ns = np.asarray(points, float), np.asarray(normals, float)
        if len(pts) < 20:
            return []
        tcp = float(cfg.tcp_offset[0])
        threshold = 1.0 / np.sqrt(1.0 + cfg.friction_coefficient**2)
        axis = pts[:, :2].mean(axis=0)
        rel = pts[:, :2] - axis
        up = np.array([0., 0., 1.])
        candidates = []
        for theta in np.linspace(0., 2*np.pi, cfg.rim_azimuths, endpoint=False):
            u = np.array([np.cos(theta), np.sin(theta), 0.])
            t = np.array([-np.sin(theta), np.cos(theta), 0.])
            rad, tang = rel @ u[:2], rel @ t[:2]
            sector = (np.abs(tang) <= cfg.pad_height/2) & (rad > 0)
            if sector.sum() < 10:
                continue
            q = np.column_stack((rad[sector], pts[sector, 2]))
            edge = q[np.argmax(q[:, 0])]
            near = np.linalg.norm(q - edge, axis=1) <= cfg.rim_span
            if near.sum() < 5:
                continue
            centred = q[near] - q[near].mean(axis=0)
            s2 = np.linalg.svd(centred, full_matrices=False)[2][0]
            if np.dot(q[near].mean(axis=0) - edge, s2) < 0:
                s2 = -s2
            w2 = np.array([-s2[1], s2[0]])
            approach = s2[0]*u + s2[1]*up
            closing = w2[0]*u + w2[1]*up
            for depth in cfg.rim_depths:
                c2 = edge + depth*s2
                along = (q - c2) @ s2
                foot = (along >= -cfg.pad_length/2 - tcp) & (along <= cfg.pad_length/2 - tcp)
                if foot.sum() < 3:
                    continue
                across = (q[foot] - c2) @ w2
                lo, hi = across.min(), across.max()
                width = hi - lo
                if width < cfg.min_width or width + cfg.min_opening_clearance > cfg.closed_gap + 2*cfg.finger_travel:
                    continue
                idx = np.flatnonzero(sector)[foot]
                n_lo = ns[idx[across <= lo + 0.0015]] @ -closing
                n_hi = ns[idx[across >= hi - 0.0015]] @ closing
                quality = min(n_lo.max(), n_hi.max())
                if quality < threshold:
                    continue
                mid = c2 + w2*(lo + hi)/2
                centre = axis[0]*np.array([1., 0., 0.]) + axis[1]*np.array([0., 1., 0.]) + mid[0]*u + mid[1]*up
                p1, p2 = centre - closing*width/2, centre + closing*width/2
                for roll in (0., np.pi/6, -np.pi/6):
                    a = Rotation.from_rotvec(closing*roll).apply(approach)
                    for flip in (1., -1.):
                        y = flip*closing
                        rotation = np.column_stack((a, y, np.cross(a, y)))
                        quat = Rotation.from_matrix(rotation).as_quat()
                        score = float(quality + 0.10*(-a[2]))
                        candidates.append((p1.copy(), p2.copy(), centre.copy(), quat, score))
        return candidates

    def _precheck_flags(self, candidate, cloud):
        """Reasons (tight / palm / support) a candidate is likely to fail,
        from the point cloud alone, at the current tcp_offset."""
        cfg = self.config
        p1, p2, centre, quat = candidate[:4]
        width = float(np.linalg.norm(p2 - p1))
        flags = []
        if width > cfg.closed_gap + 2*cfg.finger_travel - cfg.preferred_opening_clearance:
            flags.append('tight')
        rotation = Rotation.from_quat(quat).as_matrix()
        tcp = np.asarray(cfg.tcp_offset, float)
        local = (cloud - centre) @ rotation + tcp
        x0, x1, half_y, half_z = cfg.palm_box
        m = cfg.palm_margin
        if np.any((local[:, 0] >= x0 - m) & (local[:, 0] <= x1 + m)
                  & (np.abs(local[:, 1]) <= half_y + m) & (np.abs(local[:, 2]) <= half_z + m)):
            flags.append('palm')
        opened = min((width - cfg.closed_gap)/2 + cfg.opening_clearance/2, cfg.finger_travel)
        inner = cfg.closed_gap/2 + opened  # pad surface from the gripper axis
        half_x, half_z_pad = cfg.pad_length/2, cfg.pad_height/2
        corners = []
        for side in (1., -1.):
            for y in (inner, inner + 0.012):  # finger box thickness 12 mm
                for x in (-half_x, half_x):
                    for z in (-half_z_pad, half_z_pad):
                        corners.append((x, side*y, z))
        for x in (x0, x1):
            for y in (-half_y, half_y):
                for z in (-half_z, half_z):
                    corners.append((x, y, z))
        world_z = (centre + (np.asarray(corners) - tcp) @ rotation.T)[:, 2]
        if world_z.min() < cloud[:, 2].min() + 0.001:
            flags.append('support')
        return flags

    def _order_by_precheck(self, candidates, cloud):
        """Stable partition: candidates without pre-check flags come first."""
        cloud = np.asarray(cloud, float)
        cloud = cloud[np.isfinite(cloud).all(axis=1)]
        good, flagged, counts = [], [], {}
        for c in candidates:
            flags = self._precheck_flags(c, cloud)
            for f in flags:
                counts[f] = counts.get(f, 0) + 1
            (flagged if flags else good).append(c)
        self.last_stats['precheck_flagged'] = dict(counts, total=len(flagged))
        return good + flagged

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

    def _configure_measured_limits(self, measured):
        """Permit only the observed, small prismatic overshoot for this plan.

        Arm/IK limits stay unchanged. Bounds expand only as far as the actual
        observed overshoot, not by the full tolerance, and reset on each plan.
        Collision queries still use the original, unclipped measured geometry.
        """
        tolerance = np.full(len(self.action_ids), 1e-8)
        for i, name in enumerate(self.action_names):
            if name == 'torso_lift_joint':
                tolerance[i] = self.config.measured_torso_limit_tolerance + 1e-8
            elif name in FINGER_NAMES:
                tolerance[i] = self.config.measured_finger_limit_tolerance + 1e-8
        excess = np.maximum(self.lower - measured, measured - self.upper)
        bad = np.flatnonzero(excess > tolerance)
        # Reset before checking so an earlier attempt cannot leave loose bounds.
        self._state_lower, self._state_upper = self.lower.copy(), self.upper.copy()
        if len(bad):
            i = int(bad[0])
            self.last_stats['reason'] = 'start_joint_limit_violation'
            self.last_stats['start_limit_error'] = {
                'joint': self.action_names[i], 'position': float(measured[i]),
                'lower': float(self.lower[i]), 'upper': float(self.upper[i]),
                'excess': float(excess[i]), 'allowed': float(tolerance[i]),
            }
            return False
        self._state_lower = np.minimum(self.lower, measured)
        self._state_upper = np.maximum(self.upper, measured)
        self.last_stats['measured_limit_overshoot'] = {
            self.action_names[i]: float(excess[i])
            for i in np.flatnonzero(excess > 1e-8)
        }
        return True

    def _record_invalid(self, stats, stage):
        examples = stats.setdefault('invalid_examples', [])
        if len(examples) < 3:
            examples.append({'stage': stage, **(self._last_invalid or {})})

    def _valid(self, q, *, touch=False, attached=None, deadline=float('inf')):
        cfg = self.config
        self._last_invalid = None
        if planning_clock() >= deadline:
            self._last_invalid = {'reason': 'time_budget'}
            return False
        if not np.isfinite(q).all():
            self._last_invalid = {'reason': 'nonfinite_configuration'}
            return False
        lower = getattr(self, '_state_lower', self.lower)
        upper = getattr(self, '_state_upper', self.upper)
        bad = np.flatnonzero((q < lower - 1e-8) | (q > upper + 1e-8))
        if len(bad):
            i = int(bad[0])
            self._last_invalid = {
                'reason': 'joint_limit', 'joint': self.action_names[i],
                'position': float(q[i]), 'lower': float(lower[i]),
                'upper': float(upper[i]),
            }
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
                    continue  # Parked base/floor contact cannot change in fixed-base torso/arm motion.
                if body == self.object and touch and contact[3] in self.finger_ids:
                    if contact[8] >= -cfg.contact_penetration:
                        continue
                self._last_invalid = {
                    'reason': 'obstacle_collision', 'body': body,
                    'robot_link': contact[3], 'obstacle_link': contact[4],
                    'robot_link_name': self.link_names[contact[3]],
                    'distance': float(contact[8]),
                }
                return False
        # Self-collision, pair by pair, skipping pairs whose bounding boxes are
        # apart. One robot-vs-robot query over all links cost ~20 ms and was 95 %
        # of the time of a validity check; this gives the same answer.
        boxes = {}
        reach = cfg.collision_margin + 0.01
        for a, b in self._self_pair_list:
            if a not in boxes:
                boxes[a] = p.getAABB(self.robot, a, physicsClientId=self.client)
            if b not in boxes:
                boxes[b] = p.getAABB(self.robot, b, physicsClientId=self.client)
            (alo, ahi), (blo, bhi) = boxes[a], boxes[b]
            if (alo[0] > bhi[0] + reach or blo[0] > ahi[0] + reach
                    or alo[1] > bhi[1] + reach or blo[1] > ahi[1] + reach
                    or alo[2] > bhi[2] + reach or blo[2] > ahi[2] + reach):
                continue
            for contact in p.getClosestPoints(self.robot, self.robot, cfg.collision_margin,
                                              linkIndexA=a, linkIndexB=b, physicsClientId=self.client):
                self._last_invalid = {
                    'reason': 'self_collision', 'links': (contact[3], contact[4]),
                    'link_names': (self.link_names[contact[3]], self.link_names[contact[4]]),
                    'distance': float(contact[8]),
                }
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

                    self._last_invalid = {
                        "reason": "payload_collision",
                        "body": body,
                        "obstacle_link": contact[4],
                        "distance": float(contact[8]),
                    }
                    return False
        return True

    def _full(self, parked, arm, fingers):
        q = parked.copy()
        q[self.motion_indices], q[self.finger_indices] = arm, fingers
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

            stage = "lift" if attached is not None else "approach"

            if q is None:
                print(
                    f"[FAIL {stage}] Cartesian IK",
                    "position=", np.round(position, 4),
                    "budget_left=", round(deadline - planning_clock(), 2),
                    flush=True,
                )
                return None

            jump = float(np.max(np.abs(q[1:] - previous[1:])))
            torso_jump = float(abs(q[0] - previous[0]))
            if (jump > self.config.max_approach_joint_jump
                    or torso_jump > self.config.max_approach_torso_jump):
                print(
                    f"[FAIL {stage}] Joint jump",
                    "arm_radians=", round(jump, 4),
                    "torso_metres=", round(torso_jump, 4),
                    "maximum=", self.config.max_approach_joint_jump,
                    flush=True,
                )
                return None

            if not self._live_pose_ok(self._full(parked, q, fingers), position, quat):
                raise PlanningError('IK model and loaded robot disagree; check the URDF path/scaling.')
            for mid in edge_samples(previous, q, self.motion_resolution):
                if not self._valid(self._full(parked, mid, fingers), touch=touch,
                                   attached=attached, deadline=deadline):
                    print(f"[FAIL {stage}] Invalid path state:", self._last_invalid, flush=True)
                    return None
            path.append(q)
            previous = q
        return path

    def plan(self, obs):
        """Plan from measured post-navigation state; return GraspPlan or None.

        Tries each depth in config.tcp_offsets in turn (deep first), so the
        object sits well inside the finger pads whenever that is feasible.
        None means the finite sampling/IK/RRT budget failed, not that the parked
        pose is proven unreachable. last_stats explains the rejection stage.
        """
        cfg = self.config
        original = cfg.tcp_offset
        attempts = []
        try:
            for depth in cfg.tcp_offsets:
                cfg.tcp_offset = (float(depth), 0.0, 0.0)
                plan = self._plan_once(obs)
                attempts.append((float(depth), self.last_stats.get('reason')))
                self.last_stats['tcp_attempts'] = list(attempts)
                if plan is not None:
                    self.last_stats['tcp_offset'] = float(depth)
                    return plan
                if self.last_stats.get('reason') == 'invalid_start_state':
                    return None  # a deeper/shallower grasp cannot fix the start state
            return None
        finally:
            cfg.tcp_offset = original

    def _plan_once(self, obs):
        """One planning attempt at the current config.tcp_offset."""
        if self.env is None:
            raise ValueError('Construct GraspPlanner(env) before calling plan().')
        cfg = self.config
        parked = np.asarray(obs['qpos'], float).copy()
        if parked.shape != (len(self.action_ids),) or not np.isfinite(parked).all():
            raise ValueError('Invalid observed qpos.')
        actual = np.array([s[0] for s in p.getJointStates(self.robot, self.action_ids, physicsClientId=self.client)])
        if not np.allclose(actual, parked, atol=1e-4, rtol=0):
            raise PlanningError('Observation is stale; acquire it after navigation stops.')
        deadline = planning_clock() + cfg.planning_seconds
        stats = self.last_stats = dict(candidates=0, tried=0, ik_fail=0, transit_fail=0,
                                      approach_fail=0, opening_fail=0, closure_fail=0,
                                      lift_fail=0, reason='search_exhausted',
                                      start_valid=False, collision_policy=self.collision_policy.copy())
        if not self._configure_measured_limits(parked):
            return None
        self.obstacles = [p.getBodyUniqueId(i, physicsClientId=self.client)
                          for i in range(p.getNumBodies(physicsClientId=self.client))]
        self.obstacles.remove(self.robot)
        # This is the same strict validity policy used at the start of opening.
        # Run it once, before expensive sampling/IK, and restore joint positions
        # AND velocities even on early return or an exception.
        with self._snapshot():
            if not self._valid(parked, deadline=deadline):
                error = dict(self._last_invalid or {'reason': 'unknown'})
                stats['reason'] = ('time_budget' if error['reason'] == 'time_budget'
                                   else 'invalid_start_state')
                stats['start_state_error'] = error
                self._record_invalid(stats, 'start')
                print('[FAIL start]', error, flush=True)
                return None
        stats['start_valid'] = True
        candidates = self.sample_antipodal_grasps(points=obs['object_pc'], normals=obs['object_normals'],
                                                  com=obs['object_pos'])
        stats['candidates'] = len(candidates)
        if not candidates:
            stats['reason'] = 'no_antipodal_pairs'
            return None
        candidates = self._order_by_precheck(candidates, obs['object_pc'])
        clean = len(candidates) - stats['precheck_flagged']['total']
        if clean < cfg.min_clean_candidates and cfg.retry_num_samples > len(candidates):
            more = self.sample_antipodal_grasps(points=obs['object_pc'], normals=obs['object_normals'],
                                                com=obs['object_pos'], num_samples=cfg.retry_num_samples)
            flagged_first = stats['precheck_flagged']
            more = self._order_by_precheck(more, obs['object_pc'])
            more_clean = more[:len(more) - stats['precheck_flagged']['total']]
            stats['precheck_flagged'] = flagged_first
            key = lambda c: tuple(np.round(np.r_[c[2] / 0.002, c[3] / 0.02]).astype(int))
            known = {key(c) for c in candidates[:clean]}
            added = [c for c in more_clean if key(c) not in known]
            candidates = candidates[:clean] + added + candidates[clean:]
            stats['candidates'] = len(candidates)
            stats['precheck_retry'] = dict(clean_before=clean, sampled=len(more), clean_added=len(added))
        lower = self.lower[self.motion_indices].copy()
        upper = self.upper[self.motion_indices].copy()
        start = parked[self.motion_indices].copy()
        # A measured torso may overshoot its stop slightly. Recover to the
        # legal boundary during the collision-checked opening segment; never
        # expand IK/RRT limits or silently clip an IK solution.
        start[0] = np.clip(start[0], lower[0], upper[0])
        lower[self.wrap] = np.maximum(lower[self.wrap], start[self.wrap] - np.pi)
        upper[self.wrap] = np.minimum(upper[self.wrap], start[self.wrap] + np.pi)
        ik = _ManipulatorIK(self, parked)
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
                    if planning_clock() >= deadline:
                        stats['reason'] = 'time_budget'
                        break
                    p.resetBasePositionAndOrientation(self.object, *object_pose, physicsClientId=self.client)
                    stats['tried'] += 1
                    
                    print(f"[CANDIDATE {stats['tried']}] Testing", flush=True)

                    p1, p2, centre, quat, _ = candidate
                    width = np.linalg.norm(p2-p1)
                    nominal = np.full(2, (width-cfg.closed_gap)/2)
                    # Open by the full clearance when the fingers can; wide objects fall
                    # back to whatever travel is left (at least min_opening_clearance).
                    opened = np.minimum(nominal + cfg.opening_clearance/2,
                                        self.upper[self.finger_indices])
                    if np.any(opened - nominal < cfg.min_opening_clearance/2 - 1e-9):
                        continue
                    squeezed = np.maximum(self.lower[self.finger_indices], nominal - cfg.squeeze/2)
                    if np.any(opened > self.upper[self.finger_indices]):
                        continue
                    opening = self._full(parked, start, opened)
                    if not all(self._valid(q, deadline=deadline) for q in edge_samples(parked, opening, 0.002)):
                        stats['opening_fail'] += 1
                        print("[FAIL opening]", self._last_invalid, flush=True)
                        self._record_invalid(stats, 'opening')
                        continue
                    rot = Rotation.from_quat(quat)
                    grasp_pos = centre - rot.apply(cfg.tcp_offset)
                    pre_pos = grasp_pos - cfg.pregrasp_distance*rot.as_matrix()[:, 0]
                    pre = ik.solve(pre_pos, quat, start, lower, upper, deadline)
                    if pre is None:
                        stats['ik_fail'] += 1
                        print(
                            "[FAIL pregrasp IK]",
                            "target=", np.round(pre_pos, 4),
                            "budget_left=", round(deadline - planning_clock(), 2),
                            flush=True,
                        )
                        continue
                    if not self._live_pose_ok(self._full(parked, pre, opened), pre_pos, quat):
                        raise PlanningError('IK model and loaded robot disagree; check the URDF path/scaling.')
                    # RRT operates in scaled coordinates so metres and radians
                    # have an explicit relative weight. Collision samples obey
                    # both the torso and arm resolution limits.
                    scale = self.motion_scale
                    valid = lambda scaled: self._valid(
                        self._full(parked, scaled * scale, opened), deadline=deadline)
                    resolution = float(np.min(self.motion_resolution / scale))
                    rrt = RRTConnect(lower / scale, upper / scale, valid,
                                     resolution=resolution, step_size=cfg.rrt_step,
                                     max_iterations=cfg.rrt_iterations, rng=self.rng)
                    transit = rrt.plan(start / scale, pre / scale,
                                      deadline=min(deadline, planning_clock()+cfg.rrt_seconds))
                    if transit is not None:
                        transit = transit * scale
                    if transit is None:
                        stats['transit_fail'] += 1
                        print(
                            "[FAIL transit] RRT returned no path",
                            "last_state_rejection=", self._last_invalid,
                            flush=True,
                        )
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
                        print("[FAIL closure]", self._last_invalid, flush=True)
                        self._record_invalid(stats, 'closure')
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
                    # Only measured/query poses get the small overshoot allowance.
                    # Returned motor targets obey the original URDF limits.
                    commands = np.clip(np.asarray(waypoints), self.lower, self.upper)
                    return GraspPlan(commands, tuple(phases), candidate,
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
                                if i not in set(planner.motion_indices) | set(planner.finger_indices)])
        self.contact_stable = 0
        self.lost_contact_ticks = 0
        # Extra (metres, radians) for the object watch before contact: how far
        # the object moved by itself just before planning. The mustard bottle
        # lying on its side rocks +-0.05 rad without ever settling, which
        # tripped the fixed 0.10 rad limit with no robot contact at all.
        self.watch_allowance = (0.0, 0.0)
        # The environment's P-only position controller sags under gravity; an
        # integral term on the commanded arm/torso targets removes the offset.
        self.integral = np.zeros(len(planner.action_ids))
        # Waypoints where the carrot waits for the measured state: phase ends
        # and joint-space corners. Everything between two stops is (nearly) a
        # straight checked segment, and an equal-gain P controller tracking a
        # point moving along a straight line stays on that line.
        cfg = planner.config
        waypoints, phases = plan.waypoints, plan.phases
        self.motion = np.r_[planner.torso_index, planner.arm_indices]
        self.motion_scale = np.r_[cfg.torso_metric_scale, np.ones(len(planner.arm_indices))]
        n = len(waypoints)
        self.stops = np.zeros(n, bool)
        self.stops[-1] = True
        for i in range(n - 1):
            if phases[i] != phases[i + 1]:
                self.stops[i] = True
        for i in range(1, n - 1):
            a = (waypoints[i] - waypoints[i - 1])[self.motion] / self.motion_scale
            b = (waypoints[i + 1] - waypoints[i])[self.motion] / self.motion_scale
            na, nb = np.linalg.norm(a), np.linalg.norm(b)
            if na > 1e-9 and nb > 1e-9 and np.dot(a, b) / (na * nb) < cfg.follow_corner_cos:
                self.stops[i] = True
        self.carrot = None  # commanded point on the path (full action vector)
        self.finger_command = None  # closing ramp
        self.phase_ticks, self.phase = 0, None
        self.finger_previous, self.finger_still = None, 0

    def _contacts(self):
        owner = self.planner
        contacts = p.getContactPoints(owner.robot, owner.object, physicsClientId=owner.client)
        # Opposite finger contacts, positive normal force; palm contact alone
        # is not evidence that an object has been grasped.
        return set(owner.finger_ids).issubset({c[3] for c in contacts if c[9] > 0.01})

    def _fail(self, message, measured):
        print(
            "[EXEC FAILED]",
            "phase=", self.plan.phases[self.index],
            "waypoint=", self.index,
            "reason=", message,
            flush=True,
        )
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
        if self.ticks == 1 or self.ticks % 50 == 0:
            print(
                f"[EXEC] phase={phase} "
                f"waypoint={self.index}/{len(self.plan.waypoints) - 1} "
                f"ticks={self.ticks}",
                "arm_error=", round(float(
                    np.max(np.abs(target[owner.arm_indices] - q[owner.arm_indices]))
                ), 4),
                "torso_error_m=", round(float(abs(target[owner.torso_index] - q[owner.torso_index])), 5),
                "fingers=", np.round(q[owner.finger_indices], 4),
                flush=True,
            )

        if np.max(np.abs(q[self.pinned]-self.plan.waypoints[0, self.pinned])) > 0.03:
            return self._fail('Base/head joints moved away from the planned parked state.', q)
        if phase in ('open', 'transit', 'approach'):
            rotated = (Rotation.from_quat(self.plan.object_orientation).inv()
                       * Rotation.from_quat(obs['object_xyzw'])).magnitude()
            moved = np.linalg.norm(np.asarray(obs['object_pos'])-self.plan.object_position)
            if (moved > 0.012 + self.watch_allowance[0]
                    or rotated > 0.10 + self.watch_allowance[1]):
                return self._fail('Object moved; stop and obtain a fresh plan.', q)
        cfg, motion = owner.config, self.motion
        waypoints, phases = self.plan.waypoints, self.plan.phases
        # Advance the carrot along the polyline by one call's worth of joint
        # motion (common factor across joints, so it stays on each segment),
        # passing through intermediate waypoints but never past a stop.
        if self.carrot is None:
            self.carrot = q.copy()
        if phases[self.index] != self.phase:
            self.phase, self.phase_ticks = phases[self.index], 0
        self.phase_ticks += 1
        # Ramp the speed up at the start of each phase (no step in velocity).
        budget = min(1.0, self.phase_ticks / max(1, cfg.follow_ramp_ticks))
        while True:
            arm_step, torso_step = {
                'approach': (cfg.follow_cartesian_arm_step, cfg.follow_cartesian_torso_step),
                'close': (cfg.follow_cartesian_arm_step, cfg.follow_cartesian_torso_step),
                'lift': (cfg.follow_lift_arm_step, cfg.follow_lift_torso_step),
            }.get(phases[self.index], (cfg.follow_arm_step, cfg.follow_torso_step))
            limits = np.full(len(motion), arm_step)
            limits[0] = torso_step
            delta = (waypoints[self.index] - self.carrot)[motion]
            need = float(np.max(np.abs(delta) / limits))
            if need > budget:
                self.carrot[motion] += delta * (budget / need)
                break
            self.carrot[motion] = waypoints[self.index][motion]
            budget -= need
            if self.stops[self.index]:
                break
            self.index += 1
            self.ticks = 0
        target = waypoints[self.index]
        phase = phases[self.index]
        at_stop = self.stops[self.index] and np.array_equal(self.carrot[motion], target[motion])

        error = self.carrot - q
        action = self.carrot.copy()
        # While the carrot moves, the measured state lags behind it along the
        # segment; integrating that lag winds up and overshoots the segment
        # end. Integrate only the off-path (sag) part while moving.
        integrated = error[motion] / self.motion_scale
        if not at_stop and self.index > 0:
            direction = (target - waypoints[self.index - 1])[motion] / self.motion_scale
            norm = np.linalg.norm(direction)
            if norm > 1e-9:
                direction /= norm
                integrated = integrated - np.dot(integrated, direction) * direction
        self.integral[motion] = np.clip(self.integral[motion] + 0.05*integrated*self.motion_scale, -0.15, 0.15)
        action[motion] += self.integral[motion]
        action[self.pinned] = target[self.pinned]
        # Fingers get their final target directly (their gain is 10x lower
        # than the arm's, so a rate-limited command made opening take ~200
        # calls); they open during the transit, before the approach starts.
        action[owner.finger_indices] = target[owner.finger_indices]
        if phase == 'close':
            if self.finger_command is None:
                self.finger_command = waypoints[self.index - 1][owner.finger_indices].copy()
            step = target[owner.finger_indices] - self.finger_command
            self.finger_command += np.clip(step, -cfg.follow_close_rate, cfg.follow_close_rate)
            action[owner.finger_indices] = self.finger_command

        final_error = target - q
        last_of_phase = self.index + 1 == len(waypoints) or phases[self.index + 1] != phase
        if last_of_phase:
            arm_tolerance, torso_tolerance = 0.008, cfg.torso_goal_tolerance
        else:  # joint-space corner inside a phase
            arm_tolerance = cfg.follow_corner_tolerance
            torso_tolerance = cfg.follow_corner_tolerance * cfg.torso_metric_scale
        reached = (at_stop and np.max(np.abs(final_error[owner.arm_indices])) < arm_tolerance
                   and abs(final_error[owner.torso_index]) < torso_tolerance)
        if last_of_phase and self.index + 1 < len(waypoints) and phases[self.index + 1] == 'approach':
            # Enter the approach only with the gripper open as planned. Check
            # the total opening: with the closing axis tilted, gravity shifts
            # both weak finger servos the same way by several millimetres.
            reached &= abs(float(np.sum(final_error[owner.finger_indices]))) < 0.004
        if phase == 'close':
            # The arm converged at the end of the approach; once the fingers
            # squeeze, the object pushes back on the arm, so the arm/torso
            # tolerance is not required here (it kept `reached` false until
            # the 600-tick timeout on some banana/hammer grasps).
            contact = self._contacts()
            self.contact_stable = self.contact_stable + 1 if contact else 0
            fingers = q[owner.finger_indices]
            moving = (self.finger_previous is not None
                      and np.max(np.abs(fingers - self.finger_previous)) > 1e-4)
            self.finger_previous = fingers.copy()
            commanded = np.allclose(self.finger_command, target[owner.finger_indices])
            self.finger_still = self.finger_still + 1 if commanded and not moving else 0
            stalled = self.finger_still >= cfg.follow_close_stable
            if stalled and not contact and self.contact_stable == 0 and self.ticks > 60:
                return self._fail('Fingers stopped closing without bilateral contact.', q)
            # Done once contact is stable, or once the fingers have stopped
            # moving with both fingers touching (contact force may flicker
            # around the detection threshold on some shapes).
            reached = commanded and (self.contact_stable >= cfg.follow_close_stable
                                     or (stalled and contact))
        elif phase == 'lift':
            self.lost_contact_ticks = 0 if self._contacts() else self.lost_contact_ticks + 1
            if self.lost_contact_ticks > 20:
                return self._fail('Lost bilateral finger contact during lift.', q)
        if self.ticks > 600:
            return self._fail(f'Timed out during {phase} at waypoint {self.index}.', q)
        if reached:
            if self.index + 1 == len(waypoints):
                if not self._contacts():
                    if phase == 'lift':
                        # A one-tick contact flicker at the top is not a drop;
                        # wait (lost_contact_ticks fails a real one above).
                        return np.clip(action, owner.lower, owner.upper)
                    return self._fail('No bilateral finger contact at completion.', q)
                if phase == 'lift' and float(obs['object_pos'][2]) < self.plan.object_position[2] + cfg.min_lift_fraction*cfg.lift_distance:
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
