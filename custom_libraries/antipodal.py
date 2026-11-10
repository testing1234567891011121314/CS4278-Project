"""CS4278 antipodal planner: mathematical-model implementation scaffold.

NO second robot/client is created. PyBullet is used only for read-only queries
on the live environment and stateless quaternion conversions. No hypothetical
configuration is applied to the live robot.

THIS FILE IS NOT YET A RUNNABLE GRASP CONTROLLER.
Implement the four numbered TODO functions, in order:
    1. _load_robot_model()       Parse URDF into the data classes below.
    2. _forward_kinematics()     Link transforms from arbitrary joint values.
    3. _read_scene_geometry()    Read-only scene snapshot as geometry records.
    4. _geometry_distance()      Signed distances between geometric primitives.
They deliberately raise NotImplementedError. Never replace them with a dummy
identity transform or an unconditional collision-free result.

Already implemented: contact sampling, frame construction, finite-difference
arm-only IK, collision-policy orchestration, edge sampling, RRT-Connect,
Cartesian approach/lift, and the per-step execution state machine. These depend
on your TODO implementations before they can execute a complete grasp.

Recommended implementation milestones:
    A. Parse the URDF and test transforms on a small known two-joint chain.
    B. At the live qpos, compare every FK link pose with getLinkState()[4:6].
       _snapshot() also verifies the end-effector automatically.
    C. Implement a minimal primitive distance backend and test separated,
       touching, penetrating, and rotated shapes. Unsupported pairs must raise.
    D. Test robot-table, self-collision and finger-object contact policies.
    E. Integrate with solver.py and validate one slow grasp before carrying.

Geometry conventions:
    * All distances are metres; angles are radians; quaternions use xyzw.
    * A transform T_A_B maps coordinates from frame B into frame A.
    * Robot geometry is stored in its URDF link frame, never the inertial frame.
    * Scene obstacles are world-frame. Target geometry is object-frame.
    * Joint IDs/link IDs match PyBullet; action indices may have another order.
    * Fetch +x is forward along fingers and y is the closing direction.

Integration AFTER implementing the TODOs::
    planner = AntipodalGraspPlanner(
        robot_id, gripper_link_index, "assets/fetch/fetch.urdf",
        physics_client_id=client_id, action_joint_ids=action_joint_ids)
    ok = planner.start(obs, object_id)  # once, parked and settled
    action = planner.step(obs)         # once each simulation timestep
    # Return this ABSOLUTE position target to the environment.
    # Enter CARRY only when planner.done; inspect failure_reason on failure.

Dependencies: NumPy and PyBullet; stdlib XML parsing is sufficient for TODO 1.
No SciPy is required. Mesh collision handling may need an optional library or
conservative primitive approximations. Use accurate finger/palm geometry.
The model assumes a stationary scene and fixed base/torso during each grasp.
Discrete edge sampling is not continuous collision detection. Grasp holding
force still depends on the environment's position-motor settings.
"""
from dataclasses import dataclass, replace
from pathlib import Path
import time
import numpy as np
import pybullet as p


@dataclass
class GraspConfig:
    mu: float = 0.5
    max_width: float = 0.08
    min_width: float = 0.012
    max_points: int = 1024
    max_candidates: int = 20
    max_axis_z: float = 0.25
    normal_angle_deg: float = 40.0
    pad_center_offset: float = 0.025
    pregrasp_distance: float = 0.08
    lift_distance: float = 0.10
    cartesian_step: float = 0.008
    joint_resolution: float = 0.035
    prismatic_resolution: float = 0.002
    rrt_step: float = 0.18
    rrt_iterations: int = 1200
    planning_seconds: float = 25.0
    ik_iterations: int = 100
    ik_random_seeds: int = 5
    position_tolerance: float = 0.003
    rotation_tolerance: float = np.deg2rad(5.0)
    max_ik_jump: float = 0.35
    collision_margin: float = 0.001
    allowed_pad_penetration: float = 0.002
    contact_distance: float = 0.0015
    arm_step: float = 0.012
    finger_step: float = 0.0004
    joint_tolerance: float = 0.025
    finger_tolerance: float = 0.0015
    waypoint_timeout: int = 350
    close_timeout: int = 240
    contact_hold_steps: int = 12
    verify_steps: int = 20
    minimum_lift: float = 0.04
    slip_tolerance: float = 0.025
    object_move_tolerance: float = 0.015
    max_settle_speed: float = 0.03
    max_settle_angular_speed: float = 0.15
    require_support_contact: bool = True
    seed: int = 0

    def __post_init__(self):
        positive = ('mu', 'max_width', 'min_width', 'max_points',
                    'max_candidates', 'pregrasp_distance', 'lift_distance',
                    'cartesian_step', 'joint_resolution', 'prismatic_resolution', 'rrt_step',
                    'planning_seconds', 'ik_iterations', 'position_tolerance',
                    'rotation_tolerance', 'max_ik_jump', 'arm_step',
                    'finger_step', 'waypoint_timeout', 'close_timeout',
                    'contact_hold_steps', 'verify_steps', 'minimum_lift')
        for name in positive:
            if not np.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(name + ' must be finite and positive')
        if not 0 <= self.max_axis_z < 1 or self.min_width >= self.max_width:
            raise ValueError('Invalid closing-axis/width limits')
        if self.minimum_lift > self.lift_distance:
            raise ValueError('minimum_lift cannot exceed lift_distance')


def _unit(v):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if not np.isfinite(n) or n < 1e-10:
        raise ValueError('Degenerate vector')
    return v / n


def _rot(q):
    return np.asarray(p.getMatrixFromQuaternion(q), dtype=float).reshape(3, 3)


def _quat(r):
    """Rotation matrix to xyzw; stable at pi rotations."""
    r = np.asarray(r, dtype=float)
    t = np.trace(r)
    if t > 0:
        s = 2 * np.sqrt(t + 1)
        q = [(r[2, 1]-r[1, 2])/s, (r[0, 2]-r[2, 0])/s,
             (r[1, 0]-r[0, 1])/s, s/4]
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i+1) % 3, (i+2) % 3
        s = 2 * np.sqrt(max(1+r[i, i]-r[j, j]-r[k, k], 0))
        q = np.zeros(4)
        q[i], q[j], q[k], q[3] = s/4, (r[j, i]+r[i, j])/s, (r[k, i]+r[i, k])/s, (r[k, j]-r[j, k])/s
    return _unit(q)


def _rotation_error(target, actual):
    """World-frame SO(3) log; unlike sin(theta), nonzero at 180 degrees."""
    q = _quat(target @ actual.T)
    if q[3] < 0:
        q = -q
    n = np.linalg.norm(q[:3])
    if n < 1e-10:
        return 2*q[:3]
    return q[:3] * (2*np.arctan2(n, max(q[3], 0)) / n)


def _pose(position, quaternion):
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = _rot(quaternion), position
    return t


@dataclass
class JointModel:
    """One URDF joint, in parent-before-child order in RobotModel.joints.

    origin: T_parent_joint at q=0, including URDF xyz AND rpy.
    axis: unit motion axis expressed in the joint frame.
    action_index: index in obs['qpos']; None for fixed joints.
    kind: 'fixed', 'revolute', 'continuous', or 'prismatic'.
    child_link is normally the PyBullet joint index; the root link is -1.
    """
    joint_id: int
    parent_link: int
    child_link: int
    kind: str
    origin: np.ndarray
    axis: np.ndarray
    action_index: object


@dataclass
class CollisionPrimitive:
    """A shape plus its pose in the containing frame.

    kind: choose supported kinds such as 'sphere', 'capsule', 'box', 'plane'.
    parameters examples:
      sphere:  {'radius': float}
      capsule: {'radius': float, 'half_length': float} along local z
      box:     {'half_extents': np.ndarray of shape (3,)}
      plane:   local z=0, free side +z (define distance semantics consistently)
    pose: 4x4 transform, local or world as documented by the containing list.
    link_id/body_id identify contact exemptions; they are NOT array indices.
    Arrays are treated as immutable. Transforming a shape creates a new record.
    """
    kind: str
    parameters: dict
    pose: np.ndarray
    link_id: int
    body_id: int


@dataclass
class RobotModel:
    joints: list  # JointModel records, parent before child, INCLUDING fixed joints
    collisions: list  # CollisionPrimitive records in their own URDF link frames


@dataclass
class SceneSnapshot:
    obstacles: list  # CollisionPrimitive records with WORLD poses
    target: list  # CollisionPrimitive records with OBJECT-LOCAL poses
    object_pose: np.ndarray  # T_world_object (use object's base/COM frame)


class _PlanningTimeout(Exception):
    pass


class AntipodalGraspPlanner:
    """Scaffold for planning plus OPEN/TRANSIT/APPROACH/CLOSE/LIFT/VERIFY execution.

    This is a new execution interface, not a drop-in replacement for the old
    select_optimal_grasp-only API. Read the integration example above.
    Explicit arm_joint_ids/finger_joint_ids override Fetch name discovery.
    allowed_self_pairs contains live link-index pairs, e.g. (link_a, link_b).
    obstacle_ids=None snapshots every other collision body; [] means none.
    """
    def __init__(self, robot_id, ee_link_index, robot_urdf_path,
                 physics_client_id=0, action_joint_ids=None,
                 arm_joint_ids=None, finger_joint_ids=None,
                 obstacle_ids=None, allowed_self_pairs=(), config=None):
        self.client = int(physics_client_id)
        self.robot_id = int(robot_id)
        self.ee_link_index = int(ee_link_index)
        self.cfg = config or GraspConfig()
        self.rng = np.random.default_rng(self.cfg.seed)
        self.obstacle_ids = obstacle_ids
        self.info = [p.getJointInfo(robot_id, j, physicsClientId=self.client)
                     for j in range(p.getNumJoints(robot_id, physicsClientId=self.client))]
        movable = sorted((i for i in self.info if i[3] >= 0), key=lambda i: i[3])
        if any(i[2] not in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC) for i in movable):
            raise ValueError('Only single-DOF joints are supported')
        self.action_joint_ids = list(action_joint_ids if action_joint_ids is not None
                                     else [i[0] for i in movable])
        if (len(set(self.action_joint_ids)) != len(self.action_joint_ids)
                or set(self.action_joint_ids) != {i[0] for i in movable}):
            raise ValueError('action_joint_ids must contain every movable joint exactly once')
        names = {i[0]: i[1].decode() for i in self.info}
        self.arm_ids = list(arm_joint_ids if arm_joint_ids is not None else
                            [j for j in self.action_joint_ids if names[j].startswith(
                                ('shoulder', 'upperarm', 'elbow', 'forearm', 'wrist'))])
        self.finger_ids = list(finger_joint_ids if finger_joint_ids is not None else
                               [j for j in self.action_joint_ids if 'gripper_finger_joint' in names[j]])
        if len(self.arm_ids) != 7 or len(self.finger_ids) != 2:
            raise ValueError('Expected 7 arm and 2 finger joints; pass explicit IDs')
        if len(set(self.arm_ids + self.finger_ids)) != 9:
            raise ValueError('Arm/finger IDs must be distinct')
        self.arm_ix = np.array([self.action_joint_ids.index(j) for j in self.arm_ids])
        self.finger_ix = np.array([self.action_joint_ids.index(j) for j in self.finger_ids])
        self.lower, self.upper = [], []
        for j in self.action_joint_ids:
            lo, hi = self.info[j][8:10]
            if hi < lo:  # URDF continuous revolute joint
                if self.info[j][2] != p.JOINT_REVOLUTE:
                    raise ValueError('Invalid prismatic limits')
                lo, hi = -np.inf, np.inf
            self.lower.append(lo)
            self.upper.append(hi)
        self.lower, self.upper = np.array(self.lower), np.array(self.upper)
        self.prismatic_ix = np.array([k for k, j in enumerate(self.action_joint_ids)
                                      if self.info[j][2] == p.JOINT_PRISMATIC], dtype=int)
        if not np.all(np.isfinite(self.upper[self.finger_ix])):
            raise ValueError('Finger limits must be finite')
        if any(self.info[j][2] != p.JOINT_PRISMATIC for j in self.finger_ids):
            raise ValueError('Expected Fetch prismatic fingers (lower=closed, upper=open)')
        self.fixed_ix = np.array([i for i in range(len(self.action_joint_ids))
                                  if i not in set(self.arm_ix) | set(self.finger_ix)], dtype=int)
        self.allowed_self_pairs = {tuple(sorted(pair)) for pair in allowed_self_pairs}
        if not 0 <= self.ee_link_index < len(self.info):
            raise ValueError('ee_link_index must identify the gripper link')
        self.robot_urdf_path = str(Path(robot_urdf_path).resolve())
        # Lazy loading allows sampling to be explored before TODO 1 is complete.
        self.model = None
        self.scene = None
        self._attachment = None
        self._closed = False
        self.state = 'IDLE'
        self.failure_reason = ''
        self.diagnostics = {}
        self.plan = None
        self.command = None
        self._deadline = np.inf

    @property
    def done(self):
        return self.state == 'DONE'

    @property
    def failed(self):
        return self.state == 'FAILED'

    def close(self):
        """Release Python references only; there is no simulator to disconnect."""
        self.model, self.scene = None, None
        self._closed = True

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def _check_time(self):
        if time.monotonic() > self._deadline:
            raise _PlanningTimeout()

    def _read_q(self):
        return np.array([p.getJointState(self.robot_id, j, physicsClientId=self.client)[0]
                         for j in self.action_joint_ids])

    def _obs_q(self, obs):
        q = np.asarray(obs['qpos'], dtype=float)
        if q.shape != self.lower.shape or not np.all(np.isfinite(q)):
            raise ValueError('obs[qpos] shape/values do not match action_joint_ids')
        return q

    # ================================================================
    # TODO 1-4: implement your own kinematic and collision model here.
    # The remainder of the planner calls these interfaces directly.
    # ================================================================

    def _load_robot_model(self, urdf_path: str) -> RobotModel:
        """TODO 1: parse the URDF without loading a PyBullet body.

        Suggested steps (xml.etree.ElementTree is sufficient):
        1. Read all <link> and <joint> elements. Match joint NAME to
           self.info[j][1].decode(); never assume XML order equals joint IDs.
           Use child-link names to map links to those IDs, with root=-1.
        2. Parse <origin xyz=... rpy=...>. URDF rpy gives Rz(yaw) @ Ry(pitch)
           @ Rx(roll). Missing origin means identity. Parse and normalize
           each motion axis; handle URDF defaults. Retain fixed joints.
        3. Map a movable joint to self.action_joint_ids.index(joint_id).
           Topologically sort joints so parents precede their children.
        4. Read ALL <collision> elements, including each collision's origin.
           Convert into CollisionPrimitive records in the URDF link frame.
           For a box use size/2 as half_extents. Respect mesh scale/origin.
           Approximate meshes conservatively, or use an exact geometry backend.
           Do not silently drop geometry you cannot parse.
        5. Return RobotModel(joints, collisions). Include base/torso/head/finger
           links: even stationary links can be self-collision obstacles.

        Do not include inertial-origin transforms in the kinematic joint chain;
        URDF joint/link frames and COM frames are different. _snapshot handles
        the root COM offset explicitly. Reject a mismatched robot description.
        """
        raise NotImplementedError('TODO 1: implement _load_robot_model')

    def _forward_kinematics(self, q: np.ndarray,
                            root_pose: np.ndarray) -> dict:
        """TODO 2: return {link_id: T_world_link} for EVERY robot link.

        Inputs:
          q: full action-order joint vector (not only the seven arm joints).
          root_pose: T_world_root_URDF_link, already corrected for COM offset.
          self.model.joints: parsed parent-before-child JointModel records.

        Algorithm:
          poses = {-1: root_pose.copy()}
          for joint in self.model.joints:
              motion = identity(4)
              fixed:      leave motion as identity
              revolute/continuous:
                  theta = q[joint.action_index]
                  rotation = Rodrigues(joint.axis, theta)
              prismatic:
                  translation = joint.axis * q[joint.action_index]
              poses[joint.child_link] = (
                  poses[joint.parent_link] @ joint.origin @ motion)
          return poses

        Rodrigues: R = I + sin(theta)*K + (1-cos(theta))*K@K,
        where K@v = axis cross v. Joint axes are LOCAL to the joint frame.
        Never read the current gripper pose as a substitute: q is hypothetical.
        Never reset joints or advance physics. _ik already finite-differences
        this function, so an analytical Jacobian is optional, not required.
        """
        raise NotImplementedError('TODO 2: implement _forward_kinematics')

    def _read_scene_geometry(self, object_id: int) -> SceneSnapshot:
        """TODO 3: describe the CURRENT scene using read-only queries.

        1. Enumerate self.obstacle_ids, or all live bodies if it is None.
           Exclude self.robot_id and object_id from the obstacle list.
        2. Read per-link collision geometry with getCollisionShapeData.
           Convert collision-local poses to world using the relevant COM frame:
             base: getBasePositionAndOrientation
             link: getLinkState(..., computeForwardKinematics=True)[0:2]
           Collision-shape offsets are in inertial/COM frames; do not multiply
           them directly by the URDF link pose from [4:6].
        3. If using getAABB as a conservative box fallback: center=(lo+hi)/2,
           half_extents=(hi-lo)/2, WORLD axes. This may fill free space under
           a whole-table AABB; prefer individual tabletop/leg shapes.
           Record such approximations in diagnostics['geometry_fallbacks'].
        4. Read object_pose = _pose(*getBasePositionAndOrientation(object_id)).
           Build target geometry in this OBJECT frame:
             T_object_shape = inverse(object_pose) @ T_world_shape.
           Preserve target geometry even while excluding it from obstacles.
        5. Return SceneSnapshot(obstacles, target, object_pose).

        Each record must use the same primitive conventions as TODO 4.
        Do not create collision bodies/shapes, load another URDF, or modify
        the live scene. Static obstacle data may be cached between attempts.
        """
        raise NotImplementedError('TODO 3: implement _read_scene_geometry')

    def _geometry_distance(self, a: CollisionPrimitive,
                           b: CollisionPrimitive) -> float:
        """TODO 4: signed distance between two WORLD-POSE primitives, metres.

        Return >0 for separation, 0 for touching, <0 for penetration.
        Suggested first backend: capsules for arm links, boxes for palm,
        fingers/table/walls; spheres for simple obstacles. Implement EVERY
        shape-pair combination emitted by TODOs 1 and 3 (both argument orders).

        Examples:
          sphere/sphere = norm(center_a-center_b) - radius_a - radius_b
          capsule/capsule = segment_distance(seg_a, seg_b) - radius_a-radius_b
        For capsule endpoints, transform local (0,0,+/-half_length) by pose.
        For box pairs, use oriented geometry, not only their centers. SAT can
        test box overlap, but a boolean alone does not provide signed distance:
        the caller uses metric clearance and small penetration allowances.
        Define and test consistent conservative distances or use a geometry
        library. Include any plane/mesh combinations you choose to emit.

        An AABB broad phase can accelerate rejection of distant shape pairs;
        its lower-bound distance alone does NOT resolve overlapping AABBs.
        Do not call getClosestPoints on the live robot: it would test the live
        joint configuration, not the hypothetical poses supplied here.
        Raise NotImplementedError for unsupported pairs; never report them
        collision-free. Test rotated boxes, degenerate/parallel capsules,
        containment, contact, and small penetration before planning motions.
        """
        raise NotImplementedError('TODO 4: implement _geometry_distance')

    # Completed adapters: keep these unless changing the contracts above.
    def _fk(self, q):
        """Provide the existing IK solver with the hypothetical gripper pose."""
        poses = self._forward_kinematics(q, self.root_pose)
        t = poses[self.ee_link_index]
        return t[:3, 3].copy(), t[:3, :3].copy()

    def _robot_geometry(self, poses):
        """Convert link-local primitives to world poses using YOUR FK."""
        return [replace(shape, pose=poses[shape.link_id] @ shape.pose)
                for shape in self.model.collisions]

    def _live_ee(self):
        s = p.getLinkState(self.robot_id, self.ee_link_index,
                          computeForwardKinematics=True, physicsClientId=self.client)
        return _pose(s[4], s[5])

    def _build_collision_pairs(self):
        # Collapse fixed joints into rigid groups. Adjacent groups normally
        # overlap at their joint housings; other pairs remain collision-tested.
        groups = {-1: -1}
        for i in self.info:
            groups[i[0]] = groups[i[16]] if i[2] == p.JOINT_FIXED else i[0]
        adjacent = {tuple(sorted((groups[i[0]], groups[i[16]])))
                    for i in self.info if groups[i[0]] != groups[i[16]]}
        self.moving_links = set()
        for j in range(len(self.info)):
            k = j
            while k >= 0:
                if k in self.arm_ids or k in self.finger_ids:
                    self.moving_links.add(j)
                    break
                k = self.info[k][16]
        self.collision_links = sorted({shape.link_id for shape in self.model.collisions})
        self.self_pairs = []
        for i, a in enumerate(self.collision_links):
            for b in self.collision_links[i+1:]:
                if a not in self.moving_links and b not in self.moving_links:
                    continue
                pair = tuple(sorted((a, b)))
                if (groups[a] != groups[b]
                        and tuple(sorted((groups[a], groups[b]))) not in adjacent
                        and pair not in self.allowed_self_pairs):
                    self.self_pairs.append(pair)

    def _snapshot(self, object_id):
        """Read current frames and scene; all hypothetical poses stay in NumPy."""
        if self.model is None:
            self.model = self._load_robot_model(self.robot_urdf_path)
            if not self.model.collisions:
                raise ValueError('Robot model must contain collision geometry')
            self._build_collision_pairs()
        # PyBullet base pose is the COM/inertial frame. URDF FK starts from the
        # root LINK frame, so remove the local inertial offset exactly once.
        base_com = _pose(*p.getBasePositionAndOrientation(
            self.robot_id, physicsClientId=self.client))
        dynamics = p.getDynamicsInfo(self.robot_id, -1, physicsClientId=self.client)
        link_to_com = _pose(dynamics[3], dynamics[4])
        self.root_pose = base_com @ np.linalg.inv(link_to_com)
        live = self._live_ee()
        pos, rot = self._fk(self.start_q)
        if (np.linalg.norm(pos-live[:3, 3]) > 1e-4
                or np.linalg.norm(_rotation_error(live[:3, :3], rot)) > 1e-3):
            raise ValueError('Mathematical FK differs from live robot; check frames/joint mapping')
        self.scene = self._read_scene_geometry(object_id)
        if not self.scene.target:
            raise ValueError('Scene model must contain target collision geometry')
        self._attachment = None

    def _target_geometry(self, ee=None):
        """Transform target shapes without moving any simulator object."""
        world_object = self.scene.object_pose
        if ee is not None:
            if self._attachment is None:
                raise ValueError('Attached-object prediction requires an attachment transform')
            world_object = ee @ self._attachment
        return [replace(shape, pose=world_object @ shape.pose) for shape in self.scene.target]

    def _valid(self, q, allow_contact=False, attached=False, closing=False):
        """Completed collision policy; TODO 4 supplies the geometric distances.

        Exemptions depend on execution phase. Never globally ignore the object
        or the table just to obtain a path. Edge checks call this repeatedly.
        """
        self._check_time()
        if (not np.all(np.isfinite(q)) or np.any(q < self.lower-1e-7)
                or np.any(q > self.upper+1e-7)):
            return False
        poses = self._forward_kinematics(q, self.root_pose)
        robot = self._robot_geometry(poses)
        target = self._target_geometry(poses[self.ee_link_index] if attached else None)
        by_link = {}
        for shape in robot:
            by_link.setdefault(shape.link_id, []).append(shape)
        margin = self.cfg.collision_margin
        # Signed distance semantics are enforced: NaN is an implementation
        # error, not evidence that a configuration is free of collisions.
        def distance(a, b):
            d = float(self._geometry_distance(a, b))
            if not np.isfinite(d):
                raise ValueError('Geometry backend returned a non-finite distance')
            return d
        for a, b in self.self_pairs:
            if any(distance(x, y) <= margin for x in by_link[a] for y in by_link[b]):
                return False
        for shape in robot:
            if shape.link_id in self.moving_links:
                if any(distance(shape, obstacle) <= margin for obstacle in self.scene.obstacles):
                    return False
            for obj in target:
                d = distance(shape, obj)
                if shape.link_id not in self.moving_links:
                    if d < 0:
                        return False
                elif allow_contact and shape.link_id in self.finger_ids:
                    # During closure only, overlap predicts where real object
                    # contact will stop the fingers. Table/self checks still run.
                    if not closing and d < -self.cfg.allowed_pad_penetration:
                        return False
                elif d <= margin:
                    return False
        if attached:
            # The object initially touches its support: allow numerical-scale
            # penetration but no meaningful intersection during the lift.
            if any(distance(obj, obstacle) < -0.0005
                   for obj in target for obstacle in self.scene.obstacles):
                return False
        return True

    def _edge(self, a, b, **kw):
        resolution = np.full(len(a), self.cfg.joint_resolution)
        resolution[getattr(self, 'prismatic_ix', self.finger_ix)] = self.cfg.prismatic_resolution
        n = max(1, int(np.ceil(np.max(np.abs(b-a)/resolution))))
        return all(self._valid(a+(b-a)*(i/n), **kw) for i in range(n+1))

    def _ik(self, position, rotation, seed):
        """Bounded arm-only DLS with a finite-difference geometric Jacobian."""
        q = seed.copy()
        lo, hi = self.search_lower, self.search_upper
        q[self.arm_ix] = np.clip(q[self.arm_ix], lo, hi)
        weight, damping = 0.12, 0.025
        for _ in range(self.cfg.ik_iterations):
            self._check_time()
            pos, rot = self._fk(q)
            ep, er = position-pos, _rotation_error(rotation, rot)
            if (np.linalg.norm(ep) <= self.cfg.position_tolerance
                    and np.linalg.norm(er) <= self.cfg.rotation_tolerance):
                return q
            err = np.r_[ep, weight*er]
            jac = np.empty((6, 7))
            for k, ix in enumerate(self.arm_ix):
                eps = 1e-4 if q[ix]+1e-4 <= hi[k] else -1e-4
                if q[ix]+eps < lo[k]:
                    jac[:, k] = 0
                    continue
                other = q.copy()
                other[ix] += eps
                pp, rr = self._fk(other)
                jac[:, k] = np.r_[(pp-pos)/eps, weight*_rotation_error(rr, rot)/eps]
            dq = jac.T @ np.linalg.solve(jac@jac.T+damping*damping*np.eye(6), err)
            dq *= min(1.0, 0.25/max(np.max(np.abs(dq)), 1e-12))
            accepted = False
            for alpha in (1, .5, .25, .125, .0625):
                trial = q.copy()
                trial[self.arm_ix] = np.clip(q[self.arm_ix]+alpha*dq, lo, hi)
                pp, rr = self._fk(trial)
                cost = np.linalg.norm(np.r_[position-pp, weight*_rotation_error(rotation, rr)])
                if cost < np.linalg.norm(err)-1e-10:
                    q, accepted = trial, True
                    break
            if not accepted:
                return None
        return None

    def sample_antipodal_grasps(self, object_id=None, num_samples=None,
                                points=None, normals=None):
        """Return (p1,p2,center,xyzw,score), best first; outward world normals.

        Uses width, opposing friction cones and near-horizontal closing axes.
        The old global convex-shell veto is intentionally absent: it rejects
        legitimate concavities and tilted surfaces. Collision tests validate
        gripper occupancy; execution verifies that both pads actually engage.
        """
        pts, ns = np.asarray(points, float), np.asarray(normals, float)
        if pts.ndim != 2 or pts.shape[1:] != (3,) or ns.shape != pts.shape:
            raise ValueError('points and normals must have matching (N,3) shape')
        good = np.isfinite(pts).all(axis=1) & np.isfinite(ns).all(axis=1)
        good &= np.linalg.norm(ns, axis=1) > 1e-8
        pts, ns = pts[good], ns[good]
        if len(pts) > self.cfg.max_points:
            ix = self.rng.choice(len(pts), self.cfg.max_points, replace=False)
            pts, ns = pts[ix], ns[ix]
        self.sample_stats = {'points': len(pts)}
        if len(pts) < 2:
            return []
        ns /= np.linalg.norm(ns, axis=1, keepdims=True)
        ii, jj = np.triu_indices(len(pts), 1)
        d = pts[jj]-pts[ii]
        width = np.linalg.norm(d, axis=1)
        axis = d/np.maximum(width[:, None], 1e-12)
        cone = 1/np.sqrt(1+self.cfg.mu**2)
        keep = ((width >= self.cfg.min_width) & (width <= self.cfg.max_width)
                & (np.abs(axis[:, 2]) <= self.cfg.max_axis_z)
                & (np.sum(-ns[ii]*axis, axis=1) >= cone)
                & (np.sum(ns[jj]*axis, axis=1) >= cone)
                & (np.sum(ns[ii]*ns[jj], axis=1) <= -np.cos(np.deg2rad(self.cfg.normal_angle_deg))))
        ix = np.flatnonzero(keep)
        self.sample_stats['antipodal_pairs'] = len(ix)
        center = (pts[ii]+pts[jj])/2
        mid = (pts.min(axis=0)+pts.max(axis=0))/2
        scale = max(np.linalg.norm(np.ptp(pts, axis=0)), .01)
        scores = (np.linalg.norm(center-mid, axis=1)/scale
                  + 0.5*np.abs(axis[:, 2])
                  + 0.3*np.abs(center[:, 2]-mid[2])/scale)
        result = []
        for k in ix[np.argsort(scores[ix])]:
            y = axis[k]
            x = _unit(np.array([0., 0., -1.])-y*(-y[2]))
            rot = np.column_stack((x, y, np.cross(x, y)))
            if any(np.linalg.norm(center[k]-c[2]) < .008
                   and abs(y @ _unit(c[1]-c[0])) > np.cos(np.deg2rad(15)) for c in result):
                continue
            result.append((pts[ii[k]].copy(), pts[jj[k]].copy(), center[k].copy(),
                           _quat(rot), float(scores[k])))
            if len(result) >= (num_samples or self.cfg.max_candidates):
                break
        self.sample_stats['diverse_candidates'] = len(result)
        return result

    def _seeds(self, q):
        yield q.copy()
        for arm in ([0, 1, 0, -1.6, 0, 1.2, 0],
                    [0, -.6, 1.1, .9, -.7, 1.9, -1.7],
                    [-.4, -.6, -1.1, 1.6, -2.8, -1., -1.8]):
            candidate = q.copy()
            candidate[self.arm_ix] = np.clip(arm, self.search_lower, self.search_upper)
            yield candidate
        for _ in range(self.cfg.ik_random_seeds):
            candidate = q.copy()
            candidate[self.arm_ix] = self.rng.uniform(self.search_lower, self.search_upper)
            yield candidate

    def _cartesian(self, seed, start, end, rotation, **kw):
        n = max(1, int(np.ceil(np.linalg.norm(end-start)/self.cfg.cartesian_step)))
        path, previous = [], seed
        for i in range(1, n+1):
            q = self._ik(start+(end-start)*i/n, rotation, previous)
            if (q is None or np.max(np.abs(q[self.arm_ix]-previous[self.arm_ix])) > self.cfg.max_ik_jump
                    or not self._edge(previous, q, **kw)):
                return None
            path.append(q)
            previous = q
        return path

    def _transit(self, start, goal):
        if self._edge(start, goal):
            return [start.copy(), goal.copy()]
        # Bidirectional RRT-Connect. Each tree stores full q but samples arm only.
        trees = [([start.copy()], [-1]), ([goal.copy()], [-1])]
        def extend(tree, target):
            nodes, parents = tree
            near = int(np.argmin([np.linalg.norm(q[self.arm_ix]-target[self.arm_ix]) for q in nodes]))
            diff = target-nodes[near]
            norm = np.linalg.norm(diff[self.arm_ix])
            if norm < 1e-8:
                return near, True
            q = nodes[near]+diff*min(1., self.cfg.rrt_step/norm)
            if not self._edge(nodes[near], q):
                return None, False
            nodes.append(q)
            parents.append(near)
            return len(nodes)-1, norm <= self.cfg.rrt_step
        def trace(tree, index):
            nodes, parents = tree
            out = []
            while index != -1:
                out.append(nodes[index])
                index = parents[index]
            return out[::-1]
        for it in range(self.cfg.rrt_iterations):
            self._check_time()
            side = it % 2
            a, b = trees[side], trees[1-side]
            sample = b[0][0].copy()
            if self.rng.random() > .2:
                sample[self.arm_ix] = self.rng.uniform(self.search_lower, self.search_upper)
            ia, _ = extend(a, sample)
            if ia is None:
                continue
            for _ in range(100):
                ib, reached = extend(b, a[0][ia])
                if ib is None:
                    break
                if reached:
                    first, second = trace(a, ia), trace(b, ib)
                    path = first+second[::-1][1:]
                    return path if side == 0 else path[::-1]
        return None

    def _fail(self, reason, q=None):
        self.failure_reason, self.state = str(reason), 'FAILED'
        if q is not None:
            self.command = q.copy()
        return False

    def start(self, obs, object_id, candidates=None):
        """Plan once at a stationary parked base. Returns bool; reason on failure.

        The four TODOs raise NotImplementedError until implemented. Other
        missing/invalid interfaces raise ValueError. Ordinary infeasibility,
        unsettled objects and planning timeout return False. A failed attempt
        does not prove the parking cell is impossible; inspect diagnostics.
        """
        if self._closed:
            raise RuntimeError('Planner is closed')
        self.state, self.failure_reason = 'IDLE', ''
        self.plan = None
        self.start_q = self._obs_q(obs).copy()
        self.command = self.start_q.copy()
        if not np.allclose(self.start_q, self._read_q(), atol=1e-4, rtol=0):
            raise ValueError('obs[qpos] is stale or action_joint_ids order is wrong')
        self.object_id = int(object_id)
        if self.object_id == self.robot_id:
            raise ValueError('Object and robot IDs must differ')
        self.diagnostics = {'geometry_fallbacks': [], 'candidate_failures': [], 'ik_calls': 0}
        speed, angular = p.getBaseVelocity(object_id, physicsClientId=self.client)
        if (np.linalg.norm(speed) > self.cfg.max_settle_speed
                or np.linalg.norm(angular) > self.cfg.max_settle_angular_speed):
            return self._fail('Object is still moving; wait for it to settle')
        if self.cfg.require_support_contact:
            support = p.getContactPoints(bodyA=object_id, physicsClientId=self.client)
            if not any(h[2] not in (object_id, self.robot_id) and h[9] > 0 for h in support):
                return self._fail('Object has no support contact yet; wait for the spawn/drop to settle')
        self.object_start = np.asarray(p.getBasePositionAndOrientation(object_id, physicsClientId=self.client)[0])
        self._snapshot(object_id)
        self.search_lower = np.maximum(self.lower[self.arm_ix], self.start_q[self.arm_ix]-np.pi)
        self.search_upper = np.minimum(self.upper[self.arm_ix], self.start_q[self.arm_ix]+np.pi)
        self._deadline = time.monotonic()+self.cfg.planning_seconds
        opened = self.start_q.copy()
        opened[self.finger_ix] = self.upper[self.finger_ix]
        try:
            if not self._edge(self.start_q, opened):
                return self._fail('Opening fingers at current pose is obstructed')
            if candidates is None:
                candidates = self.sample_antipodal_grasps(points=obs['object_pc'], normals=obs['object_normals'])
            if not candidates:
                return self._fail('No antipodal candidates; inspect sample_stats')
            for index, cand in enumerate(candidates[:self.cfg.max_candidates]):
                self._check_time()
                center, rotation = np.asarray(cand[2]), _rot(cand[3])
                grasp_pos = center-self.cfg.pad_center_offset*rotation[:, 0]
                pre_pos = grasp_pos-self.cfg.pregrasp_distance*rotation[:, 0]
                lift_pos = grasp_pos+np.array([0., 0., self.cfg.lift_distance])
                reason = 'No valid pregrasp IK'
                # Parallel jaws admit a second orientation, with y,z both flipped.
                for symmetric in (False, True):
                    rot = rotation @ np.diag([1., -1., -1.]) if symmetric else rotation
                    for seed in self._seeds(opened):
                        self.diagnostics['ik_calls'] += 1
                        pre = self._ik(pre_pos, rot, seed)
                        if pre is None or not self._valid(pre):
                            continue
                        approach = self._cartesian(pre, pre_pos, grasp_pos, rot)
                        if approach is None:
                            reason = 'Approach IK/continuity/collision failure'
                            continue
                        closed = approach[-1].copy()
                        closed[self.finger_ix] = self.lower[self.finger_ix]
                        # Validate the whole potential closing sweep against the
                        # table, other links and obstacles. Finger-object overlap
                        # is exempt ONLY here: real contact stops the fingers.
                        if not self._edge(approach[-1], closed,
                                          allow_contact=True, closing=True):
                            reason = 'Finger closing sweep intersects robot/environment'
                            continue
                        transit = self._transit(opened, pre)
                        if transit is None:
                            reason = 'No collision-free transit within budget'
                            continue
                        # Initial lift feasibility with open fingers. Replanned
                        # after closure using measured aperture and object pose.
                        lift = self._cartesian(approach[-1], grasp_pos, lift_pos, rot)
                        if lift is None:
                            reason = 'Lift endpoint/path unreachable'
                            continue
                        self.plan = {'candidate': cand, 'open': opened,
                                     'transit': transit[1:], 'approach': approach,
                                     'grasp_position': grasp_pos, 'rotation': rot}
                        self._enter('OPEN', [opened])
                        return True
                self.diagnostics['candidate_failures'].append({'candidate': index, 'reason': reason})
            return self._fail('No candidate has a valid approach, transit and lift')
        except _PlanningTimeout:
            return self._fail('Planning time budget exhausted; retry/reposition or increase budget')
        finally:
            self._deadline = np.inf

    def _enter(self, state, path=()):
        self.state, self.path = state, list(path)
        self.path_index, self.ticks, self.stable_ticks = 0, 0, 0

    def _contacts(self):
        hits = p.getContactPoints(self.robot_id, self.object_id, physicsClientId=self.client)
        return {h[3] for h in hits if h[3] in self.finger_ids
                and h[8] <= self.cfg.contact_distance and h[9] > 0}

    def _ramp(self, target):
        limits = np.zeros(len(target))
        limits[self.arm_ix], limits[self.finger_ix] = self.cfg.arm_step, self.cfg.finger_step
        self.command += np.clip(target-self.command, -limits, limits)
        self.command[self.fixed_ix] = self.start_q[self.fixed_ix]
        self.command = np.clip(self.command, self.lower, self.upper)
        return self.command.copy()

    def _plan_lift(self, q):
        # Refresh scene data at actual contact. Predict rigid object motion
        # mathematically for collision checks; no physical attachment is made.
        self.start_q = q.copy()
        self._snapshot(self.object_id)
        pos, rot = self._fk(q)
        ee = np.eye(4)
        ee[:3, 3], ee[:3, :3] = pos, rot
        self._attachment = np.linalg.inv(ee) @ self.scene.object_pose
        self.grasp_relative = np.linalg.inv(self._live_ee()) @ np.r_[p.getBasePositionAndOrientation(
            self.object_id, physicsClientId=self.client)[0], 1.]
        self.lift_start_z = p.getBasePositionAndOrientation(self.object_id, physicsClientId=self.client)[0][2]
        self._deadline = time.monotonic()+self.cfg.planning_seconds
        try:
            path = self._cartesian(q, pos, pos+np.array([0, 0, self.cfg.lift_distance]), rot,
                                   allow_contact=True, attached=True)
        except _PlanningTimeout:
            path = None
        finally:
            self._deadline = np.inf
        if path is None:
            return self._fail('Measured grasp has no collision-free attached-object lift', q)
        # Geometry used the MEASURED aperture; command keeps closing pressure.
        for waypoint in path:
            waypoint[self.finger_ix] = self.lower[self.finger_ix]
        self._enter('LIFT', path)
        return True

    def step(self, obs):
        """Return one absolute action. Caller alone advances live simulation.

        On FAILED: hold measured configuration (including current finger width).
        On DONE: hold the successful lift and commanded closure. Caller owns
        transition to carry and subsequent slip monitoring.
        """
        q = self._obs_q(obs)
        if self.state == 'IDLE':
            raise RuntimeError('Call start(obs, object_id) before step')
        if self.failed or self.done:
            return self.command.copy()
        self.ticks += 1
        obj = np.asarray(p.getBasePositionAndOrientation(self.object_id, physicsClientId=self.client)[0])
        if self.state in ('OPEN', 'TRANSIT', 'APPROACH'):
            if np.linalg.norm(obj-self.object_start) > self.cfg.object_move_tolerance:
                self._fail('Object moved before closure; generate a fresh plan', q)
                return self.command.copy()
        if len(self.fixed_ix) and np.max(np.abs(q[self.fixed_ix]-self.start_q[self.fixed_ix])) > .025:
            self._fail('Base/torso/head moved during fixed-base grasp execution', q)
            return self.command.copy()
        if self.state == 'CLOSE':
            target = self.command.copy()
            target[self.finger_ix] = self.lower[self.finger_ix]
            action = self._ramp(target)
            both = set(self.finger_ids).issubset(self._contacts())
            self.stable_ticks = self.stable_ticks+1 if both else 0
            # Wait until the command has reached closed, even if the object
            # prevents measured joints from reaching zero.
            closed_command = np.max(np.abs(action[self.finger_ix]-self.lower[self.finger_ix])) < 1e-6
            if both and closed_command and self.stable_ticks >= self.cfg.contact_hold_steps:
                if not self._plan_lift(q):
                    return self.command.copy()
            elif self.ticks > self.cfg.close_timeout:
                self._fail('Closure did not establish sustained contact on both fingers', q)
                return self.command.copy()
            return action
        if self.state in ('LIFT', 'VERIFY'):
            rel = np.linalg.inv(self._live_ee()) @ np.r_[obj, 1.]
            if np.linalg.norm(rel[:3]-self.grasp_relative[:3]) > self.cfg.slip_tolerance:
                self._fail('Object slipped relative to the gripper during lift', q)
                return self.command.copy()
        if self.state == 'VERIFY':
            good = (obj[2]-self.lift_start_z >= self.cfg.minimum_lift
                    and set(self.finger_ids).issubset(self._contacts()))
            self.stable_ticks = self.stable_ticks+1 if good else 0
            if self.stable_ticks >= self.cfg.verify_steps:
                self.state = 'DONE'
            elif self.ticks > self.cfg.waypoint_timeout:
                self._fail('Lift verification failed: object height/contact not sustained', q)
            return self.command.copy()
        target = self.path[self.path_index]
        arm_ok = np.max(np.abs(q[self.arm_ix]-target[self.arm_ix])) < self.cfg.joint_tolerance
        fingers_ok = (self.state == 'LIFT' or np.max(np.abs(q[self.finger_ix]-target[self.finger_ix]))
                      < self.cfg.finger_tolerance)
        # Compare command as well as measured q: never skip a waypoint merely
        # because measured q entered a broad tolerance before the ramp got there.
        command_ok = np.max(np.abs(self.command-target)) < 1e-6
        if arm_ok and fingers_ok and command_ok:
            self.stable_ticks += 1
        else:
            self.stable_ticks = 0
        if self.stable_ticks >= 2:
            self.path_index += 1
            self.ticks, self.stable_ticks = 0, 0
            if self.path_index == len(self.path):
                if self.state == 'OPEN':
                    self._enter('TRANSIT', self.plan['transit'])
                elif self.state == 'TRANSIT':
                    self._enter('APPROACH', self.plan['approach'])
                elif self.state == 'APPROACH':
                    self._enter('CLOSE')
                elif self.state == 'LIFT':
                    self._enter('VERIFY')
                return self.command.copy()
            target = self.path[self.path_index]
        if self.ticks > self.cfg.waypoint_timeout:
            self._fail('Timed out tracking '+self.state+' waypoint', q)
            return self.command.copy()
        return self._ramp(target)
