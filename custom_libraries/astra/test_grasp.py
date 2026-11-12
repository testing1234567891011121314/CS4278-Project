"""Run: python -m unittest discover -s . -p test_grasp.py -v

Pure geometry/RRT/controller tests run without Bullet. Bullet smoke tests use
fetch.urdf (set FETCH_URDF or place it beside this file); mesh geometry is removed
in that smoke fixture, so it tests kinematics, not full Fetch collision geometry.

Added FullGeometryTests preserves all collision meshes and reproduces the logged
parked pose. Its clearance tests should fail if that posture remains colliding.
From project root (test_grasp.py at root):
  FETCH_URDF=assets/fetch/fetch.urdf python -m unittest test_grasp.FullGeometryTests -v
If the file is in custom_libraries, use custom_libraries.test_grasp.FullGeometryTests.
Requires the earlier grasp_limit_tolerance.patch. Missing dependencies/assets
produce skips (or a failure for a missing explicit FETCH_URDF), not validation.
"""
import importlib.util
import os
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
from contextlib import nullcontext
import json
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
# Support both project-root and custom_libraries test locations.
if os.environ.get('GRASP_MODULE'):
    g = importlib.import_module(os.environ['GRASP_MODULE'])
else:
    try:
        import grasp as g
    except ModuleNotFoundError as exc:
        if exc.name != 'grasp':
            raise
        g = importlib.import_module('custom_libraries.grasp')


def cloud():
    # Dense opposing faces of a 40 x 40 x 40 mm box.
    x, z = np.meshgrid(np.linspace(-0.02, .02, 9), np.linspace(.64, .68, 9))
    face = np.c_[x.ravel(), np.full(x.size, -.02), z.ravel()]
    other = face.copy(); other[:, 1] = .02
    return np.r_[face, other], np.r_[np.tile([0, -1, 0], (len(face), 1)),
                                            np.tile([0, 1, 0], (len(face), 1))]


class GeometryTests(unittest.TestCase):
    def test_antipodal_cones_frames_and_finger_width(self):
        planner = g.GraspPlanner()
        pts, normals = cloud()
        candidates = planner.sample_antipodal_grasps(points=pts, normals=normals)
        self.assertGreater(len(candidates), 0)
        threshold = 1 / np.sqrt(1 + planner.config.friction_coefficient**2)
        for a, b, centre, quat, score in candidates:
            d = (b-a) / np.linalg.norm(b-a)
            na = normals[np.argmin(np.linalg.norm(pts-a, axis=1))]
            nb = normals[np.argmin(np.linalg.norm(pts-b, axis=1))]
            self.assertGreaterEqual(-na@d + 1e-9, threshold)
            self.assertGreaterEqual(nb@d + 1e-9, threshold)
            r = Rotation.from_quat(quat).as_matrix()
            np.testing.assert_allclose(np.abs(r[:, 1] @ d), 1, atol=1e-8)
            np.testing.assert_allclose(r.T@r, np.eye(3), atol=1e-8)
            self.assertAlmostEqual(np.linalg.det(r), 1)
            np.testing.assert_allclose(centre, (a+b)/2)
            fingers = (np.linalg.norm(b-a)-planner.config.closed_gap)/2
            self.assertGreaterEqual(fingers, 0)
            self.assertLess(fingers, .05)

    def test_same_direction_normals_not_antipodal(self):
        pts, normals = cloud(); normals[:] = [0, 1, 0]
        self.assertEqual(g.GraspPlanner().sample_antipodal_grasps(points=pts, normals=normals), [])

    def test_invalid_and_empty_cloud(self):
        planner = g.GraspPlanner()
        self.assertEqual(planner.sample_antipodal_grasps(points=np.empty((0, 3)), normals=np.empty((0, 3))), [])
        self.assertEqual(planner.sample_antipodal_grasps(points=[[np.nan, 0, 0]], normals=[[0, 0, 0]]), [])
        with self.assertRaises(ValueError):
            planner.sample_antipodal_grasps(points=[1, 2, 3], normals=[1, 2, 3])

    def test_object_wider_than_gripper_rejected(self):
        pts, normals = cloud(); pts[:, 1] *= 4
        self.assertEqual(g.GraspPlanner().sample_antipodal_grasps(points=pts, normals=normals), [])


class RRTTests(unittest.TestCase):
    def test_direct_edge_checks_interior_not_only_endpoints(self):
        valid = lambda q: abs(q[0]) >= .1
        planner = g.RRTConnect([-1], [1], valid, resolution=.01, max_iterations=200)
        self.assertIsNone(planner.plan([-.8], [.8]))

    def test_connect_routes_around_blocking_wall(self):
        valid = lambda q: not (-.12 < q[0] < .12 and q[1] < .45)
        planner = g.RRTConnect([-1, -1], [1, 1], valid, resolution=.02,
                              step_size=.2, rng=np.random.default_rng(19))
        path = planner.plan([-.8, -.5], [.8, -.5])
        self.assertIsNotNone(path)
        np.testing.assert_allclose(path[0], [-.8, -.5])
        np.testing.assert_allclose(path[-1], [.8, -.5])
        self.assertTrue(all(valid(q) for q in path))
        self.assertLessEqual(np.abs(np.diff(path, axis=0)).max(), .0200001)

    def test_zero_length_and_expired_deadline(self):
        planner = g.RRTConnect([-1, -1], [1, 1], lambda q: True)
        np.testing.assert_allclose(planner.plan([0, 0], [0, 0]), np.zeros((2, 2)))
        self.assertIsNone(planner.plan([0, 0], [1, 1], deadline=time.monotonic()-1))
        self.assertIsNone(planner.plan([0, 0], [2, 0]))


class FollowerTests(unittest.TestCase):
    def setup_follower(self, phase='close'):
        owner = SimpleNamespace(action_ids=list(range(15)), arm_indices=np.arange(6,13),
                                finger_indices=np.array([13,14]), finger_ids=[30,31],
                                lower=np.full(15,-10), upper=np.full(15,10),
                                config=g.GraspConfig(), robot=1, object=2, client=3)
        plan = g.GraspPlan(np.zeros((1,15)), (phase,), (), np.zeros(2), np.zeros(2),
                          np.array([0.,0.,.65]), np.array([0.,0.,0.,1.]), {})
        obs = dict(qpos=np.zeros(15), qvel=np.zeros(15), object_pos=np.array([0.,0.,.65]),
                   object_xyzw=np.array([0.,0.,0.,1.]))
        return g.GraspFollower(owner, plan), obs

    def test_no_contact_does_not_count_as_success(self):
        follower, obs = self.setup_follower()
        with patch.object(follower, '_contacts', return_value=False):
            for _ in range(601): follower.step(obs)
        self.assertEqual(follower.status, 'failed')

    def test_bilateral_contact_finishes_close(self):
        follower, obs = self.setup_follower()
        with patch.object(follower, '_contacts', return_value=True):
            for _ in range(30): follower.step(obs)
        self.assertEqual(follower.status, 'done')

    def test_lift_without_object_rising_fails(self):
        follower, obs = self.setup_follower('lift')
        with patch.object(follower, '_contacts', return_value=True): follower.step(obs)
        self.assertEqual(follower.status, 'failed')

    def test_object_rotation_invalidates_cached_plan(self):
        follower, obs = self.setup_follower('transit')
        obs['object_xyzw'] = Rotation.from_euler('z', .2).as_quat()
        follower.step(obs)
        self.assertEqual(follower.status, 'failed')

    def test_parked_base_drift_stops_execution(self):
        follower, obs = self.setup_follower()
        obs['qpos'][0] = .1
        follower.step(obs)
        self.assertEqual(follower.status, 'failed')


@unittest.skipIf(g.p is None, 'PyBullet not installed')
class BulletTests(unittest.TestCase):
    def setUp(self):
        self.path = _fetch_path(self)
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        root = ET.parse(self.path).getroot()
        # Keep exact kinematics/inertials and the two supplied box fingers.
        for link in root.findall('link'):
            for child in list(link):
                if child.tag == 'visual' or (child.tag == 'collision' and child.find('geometry/mesh') is not None):
                    link.remove(child)
        urdf = Path(self.folder.name)/'test_fetch.urdf'
        ET.ElementTree(root).write(urdf)
        p = g.p
        client = p.connect(p.DIRECT)
        self.addCleanup(lambda: p.disconnect(physicsClientId=client))
        robot = p.loadURDF(str(urdf), [-2,0,0], useFixedBase=True, physicsClientId=client)
        info = [p.getJointInfo(robot,j,physicsClientId=client) for j in range(p.getNumJoints(robot,physicsClientId=client))]
        active = [v for v in info if v[3]>=0]
        obj = p.createMultiBody(baseMass=1, basePosition=[4,4,4], physicsClientId=client)
        env = SimpleNamespace(robot_id=robot, object_id=obj, pb_physics_client=client,
                              joint_indices=[v[0] for v in active],
                              joint_lower=np.array([v[8] for v in active]),
                              joint_upper=np.array([v[9] for v in active]))
        self.owner = g.GraspPlanner(env, robot_urdf_path=self.path)
        self.q = np.zeros(len(active)); self.q[:6] = [1.3, .2, .6, .2, .1, .2]
        self.q[self.owner.arm_indices] = [0., -.8, .2, 1.4, -.1, -.6, .1]
        self.owner._set(self.q)

    def test_frozen_ik_matches_real_fk_and_roundtrips(self):
        p, owner = g.p, self.owner
        ik = g._ArmIK(owner, self.q)
        self.addCleanup(ik.close)
        seed = self.q[owner.arm_indices]
        lower = np.maximum(owner.lower[owner.arm_indices], seed-np.pi)
        upper = np.minimum(owner.upper[owner.arm_indices], seed+np.pi)
        live = p.getLinkState(owner.robot,owner.ee,computeForwardKinematics=True,physicsClientId=owner.client)
        result = ik.solve(np.array(live[4]), np.array(live[5]), seed+.01, lower, upper, time.monotonic()+10)
        self.assertIsNotNone(result)
        q = self.q.copy(); q[owner.arm_indices] = result
        owner._set(q)
        reached = p.getLinkState(owner.robot,owner.ee,computeForwardKinematics=True,physicsClientId=owner.client)
        self.assertLess(np.linalg.norm(np.array(live[4])-reached[4]), .003)
        np.testing.assert_allclose(q[:6], self.q[:6])

    def test_snapshot_restores_joint_positions_and_velocities_on_error(self):
        p, owner = g.p, self.owner
        for joint in owner.action_ids:
            p.resetJointState(owner.robot,joint,p.getJointState(owner.robot,joint,physicsClientId=owner.client)[0],
                              targetVelocity=.123,physicsClientId=owner.client)
        before = np.array(p.getJointStates(owner.robot,owner.action_ids,physicsClientId=owner.client), dtype=object)[:,:2].astype(float)
        with self.assertRaises(RuntimeError):
            with owner._snapshot():
                owner._set(np.zeros_like(self.q))
                raise RuntimeError('intentional test failure')
        after = np.array(p.getJointStates(owner.robot,owner.action_ids,physicsClientId=owner.client), dtype=object)[:,:2].astype(float)
        np.testing.assert_allclose(before,after)


# Additional diagnostics. These do not run the environment, renderer, or video
# recorder. FullGeometryTests uses real collision meshes in a private DIRECT
# client. Its posture-clearance tests are EXPECTED TO FAIL if the logged overlap
# still exists: a failure prints the actual link names and signed distances.
# None of these tests changes collision exclusions or commands the live robot.

ACTION_NAMES = (
    'root_x_axis_joint', 'root_y_axis_joint', 'root_z_rotation_joint',
    'torso_lift_joint', 'head_pan_joint', 'head_tilt_joint',
    *g.ARM_NAMES, *g.FINGER_NAMES,
)
# Step 276 in the supplied November 11 log: measured state used for planning.
LOGGED_Q = dict(zip(ACTION_NAMES, (
    1.03411627, -0.0375583172, 0.00006749043, -0.00010379203,
    0.0, 0.00614505028, 0.00392274233, -0.963520348,
    0.0410806052, -2.00681472, 0.039035365, 1.09552920,
    -0.0350206085, -0.000749000115, 0.0143650118,
)))


def _fetch_path(case):
    explicit = os.environ.get('FETCH_URDF')
    if explicit:
        path = Path(explicit).expanduser().resolve()
        case.assertTrue(path.is_file(), f'FETCH_URDF does not exist: {path}')
        return path
    for path in (Path('assets/fetch/fetch.urdf'),
                 Path(__file__).with_name('fetch.urdf'),
                 Path(__file__).resolve().parent.parent / 'assets/fetch/fetch.urdf'):
        if path.is_file():
            return path.resolve()
    case.skipTest('Set FETCH_URDF to assets/fetch/fetch.urdf')


def _bare_planner(case):
    """Unit-test fixture; real collision geometry is covered separately below."""
    owner = g.GraspPlanner()
    case.assertTrue(hasattr(owner, '_configure_measured_limits'),
                    'Apply grasp_limit_tolerance.patch to the imported grasp module first.')
    owner.action_names = list(ACTION_NAMES)
    owner.action_ids = list(range(15))
    owner.arm_indices, owner.finger_indices = np.arange(6, 13), np.array([13, 14])
    owner.arm_ids, owner.finger_ids = list(range(6, 13)), [13, 14]
    owner.lower, owner.upper = np.full(15, -3.), np.full(15, 3.)
    owner.lower[[3, 13, 14]], owner.upper[[3, 13, 14]] = 0., [.38615, .05, .05]
    owner.robot, owner.object, owner.client = 1, 2, 0
    owner.obstacles, owner.self_pairs = [], set()
    owner.moving_links = set(range(6, 15))
    owner._set = Mock()
    return owner


def _contact(a, b, distance=-.01, force=1.):
    # The Bullet fields consumed by the collision/contact code.
    return (0, 1, 2, a, b, (0, 0, 0), (0, 0, 0), (0, 0, 1), distance, force)


class MeasuredLimitTests(unittest.TestCase):
    def setUp(self):
        self.owner = _bare_planner(self)
        self.q = np.array([LOGGED_Q[name] for name in ACTION_NAMES])

    def test_logged_overshoot_reaches_collision_queries_without_clipping(self):
        o = self.owner
        self.assertTrue(o._configure_measured_limits(self.q))
        fake = SimpleNamespace(getClosestPoints=Mock(return_value=[]))
        with patch.object(g, 'p', fake):
            self.assertTrue(o._valid(self.q))
        np.testing.assert_array_equal(o._set.call_args.args[0], self.q)
        fake.getClosestPoints.assert_called_once()

    def test_tolerance_does_not_allow_worsening_the_observed_overshoot(self):
        o = self.owner
        o._configure_measured_limits(self.q)
        q = self.q.copy(); q[13] -= .0001
        self.assertFalse(o._valid(q))
        self.assertEqual(o._last_invalid['reason'], 'joint_limit')
        o._set.assert_not_called()

    def test_arm_limit_error_is_not_treated_as_finger_tolerance(self):
        q = self.q.copy(); q[6] = self.owner.lower[6] - .00001
        self.assertFalse(self.owner._configure_measured_limits(q))
        self.assertEqual(self.owner.last_stats['start_limit_error']['joint'], g.ARM_NAMES[0])

    def test_large_finger_error_stops_before_ik(self):
        q = self.q.copy(); q[13] = -.003
        self.assertFalse(self.owner._configure_measured_limits(q))
        self.assertEqual(self.owner.last_stats['reason'], 'start_joint_limit_violation')

    def test_next_attempt_does_not_inherit_previous_tolerance_bounds(self):
        o = self.owner
        o._configure_measured_limits(self.q)
        o._configure_measured_limits(np.zeros(15))
        np.testing.assert_array_equal(o._state_lower, o.lower)
        self.assertFalse(o._valid(self.q))


class CollisionAndStageTests(unittest.TestCase):
    def setUp(self):
        self.owner = _bare_planner(self)
        self.q = np.array([LOGGED_Q[name] for name in ACTION_NAMES])
        self.owner._configure_measured_limits(self.q)

    def test_forbidden_self_pair_is_reported_after_limit_check(self):
        o = self.owner; o.self_pairs = {frozenset((6, 17))}
        fake = SimpleNamespace(getClosestPoints=Mock(return_value=[_contact(6, 17, -.031357)]))
        with patch.object(g, 'p', fake): self.assertFalse(o._valid(self.q))
        self.assertEqual(o._last_invalid['reason'], 'self_collision')
        self.assertEqual(set(o._last_invalid['links']), {6, 17})

    def test_touch_permission_is_limited_to_fingers_and_target(self):
        o = self.owner; o.obstacles = [o.object]
        def query(a, b, *args, **kwargs):
            return [_contact(6, -1, -.0001)] if b == o.object else []
        with patch.object(g, 'p', SimpleNamespace(getClosestPoints=query)):
            self.assertFalse(o._valid(self.q, touch=True))
        self.assertEqual(o._last_invalid['reason'], 'obstacle_collision')

    def test_shallow_finger_contact_allowed_only_when_touch_enabled(self):
        o = self.owner; o.obstacles = [o.object]
        def query(a, b, *args, **kwargs):
            return [_contact(13, -1, -.0001)] if b == o.object else []
        with patch.object(g, 'p', SimpleNamespace(getClosestPoints=query)):
            self.assertTrue(o._valid(self.q, touch=True))
            self.assertFalse(o._valid(self.q, touch=False))

    def test_opening_failure_is_counted_before_any_ik_call(self):
        o = self.owner
        o.env, o.wrap, o.ee = object(), np.zeros(7, dtype=bool), 12
        o.config.max_candidates = 2
        o.sample_antipodal_grasps = Mock(return_value=[(
            np.array([0, -.02, .65]), np.array([0, .02, .65]),
            np.array([0, 0, .65]), np.array([0, 0, 0, 1]), 1.)] * 2)
        o._snapshot = lambda: nullcontext()
        o.self_pairs = {frozenset((6, 17))}
        fake = SimpleNamespace(
            getJointStates=lambda *a, **k: [(v, 0, (), 0) for v in self.q],
            getNumBodies=lambda **k: 2,
            getBodyUniqueId=lambda i, **k: i + 1,
            getBasePositionAndOrientation=lambda *a, **k: ([0, 0, .65], [0, 0, 0, 1]),
            resetBasePositionAndOrientation=Mock(),
            getClosestPoints=lambda a, b, *args, **k: [_contact(6, 17, -.031357)] if a == b else [])
        ik = SimpleNamespace(close=Mock(), solve=Mock())
        with patch.object(g, 'p', fake), patch.object(g, '_ArmIK', return_value=ik):
            self.assertIsNone(o.plan(dict(qpos=self.q, object_pc=[], object_normals=[])))
        self.assertEqual(o.last_stats['opening_fail'], 2)
        self.assertEqual(o.last_stats['closure_fail'], 0)
        self.assertEqual(o.last_stats['invalid_examples'][0]['reason'], 'self_collision')
        ik.solve.assert_not_called()

    def test_cartesian_ik_failure_does_not_become_a_path(self):
        self._cartesian_case(None, expected=False)

    def test_cartesian_joint_branch_jump_is_rejected(self):
        self._cartesian_case(np.full(7, .4), expected=False)

    def test_cartesian_collision_between_waypoints_is_rejected(self):
        self._cartesian_case(np.full(7, .06), expected=False, collision=True)

    def test_cartesian_small_valid_segment_is_accepted(self):
        self._cartesian_case(np.full(7, .01), expected=True)

    def _cartesian_case(self, result, *, expected, collision=False):
        o = self.owner
        o._live_pose_ok = Mock(return_value=True)
        # Endpoints clear, interior blocked: this must not pass an endpoint-only check.
        o._valid = Mock(side_effect=lambda q, **kw: not (collision and .015 < q[6] < .045))
        o._last_invalid = {'reason': 'test_obstacle'}
        ik = SimpleNamespace(solve=Mock(return_value=result))
        path = o._cartesian(ik, np.zeros(3), np.array([.004, 0, 0]),
                            np.array([0., 0., 0., 1.]), np.zeros(7), np.zeros(15),
                            np.array([.02, .02]), np.full(7, -3.), np.full(7, 3.),
                            time.monotonic() + 10)
        self.assertEqual(path is not None, expected)


class ContactEvidenceTests(unittest.TestCase):
    def test_contact_requires_both_fingers_with_positive_force(self):
        follower, _ = FollowerTests().setup_follower()
        for contacts, expected in (
            ([_contact(12, -1)], False),  # Palm alone.
            ([_contact(30, -1)], False),  # One finger alone.
            ([_contact(30, -1), _contact(31, -1, force=0.)], False),
            ([_contact(30, -1), _contact(31, -1)], True),
        ):
            with self.subTest(contacts=contacts), patch.object(
                g, 'p', SimpleNamespace(getContactPoints=Mock(return_value=contacts))
            ):
                self.assertEqual(follower._contacts(), expected)


@unittest.skipIf(g.p is None, 'PyBullet not installed; real collision diagnostics NOT RUN')
class FullGeometryTests(unittest.TestCase):
    """Independent reproducer: real collision shapes, no EGL and no rendering.

    Run this class alone first when investigating links (6,17). The clearance
    assertions deliberately fail if the real model has the reported overlap.
    Pose-sensitivity output is diagnostic; it does not certify a safe recovery.
    Set FETCH_QPOS_JSON to a JSON {joint_name: value} mapping to use a new pose;
    all 15 ACTION_NAMES are required. Otherwise the logged pose above is used.
    """
    def setUp(self):
        self.path = _fetch_path(self)
        root = ET.parse(self.path).getroot()
        # Preserve ALL collision geometry and transforms. Only rendering data
        # is removed; absolute mesh paths let the temporary URDF load reliably.
        for link in root.findall('link'):
            for visual in link.findall('visual'): link.remove(visual)
        missing = []
        for mesh in root.findall('.//collision/geometry/mesh'):
            name = mesh.get('filename')
            if name.startswith('package://'):
                self.fail('Resolve package:// collision mesh paths in the test URDF first: ' + name)
            path = (self.path.parent / name).resolve()
            if not path.is_file(): missing.append(str(path))
            mesh.set('filename', str(path))
        if missing:
            message = 'Full collision tests need the actual meshes; missing: ' + ', '.join(missing[:3])
            if os.environ.get('FETCH_URDF'): self.fail(message)
            self.skipTest(message)
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        path = Path(folder.name) / 'full_collision_fetch.urdf'
        ET.ElementTree(root).write(path)
        p = g.p
        client = p.connect(p.DIRECT)
        self.assertGreaterEqual(client, 0, 'Could not open private DIRECT client')
        self.addCleanup(lambda: p.disconnect(physicsClientId=client))
        # Match the supplied environment's import flags. Explicit closest-point
        # queries are exercised independently of simulation collision response.
        robot = p.loadURDF(str(path), [-2, 0, 0], useFixedBase=True, physicsClientId=client)
        info = [p.getJointInfo(robot, j, physicsClientId=client)
                for j in range(p.getNumJoints(robot, physicsClientId=client))]
        active = [v for v in info if v[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC)]
        obj = p.createMultiBody(baseMass=1, basePosition=[4, 4, 4], physicsClientId=client)
        env = SimpleNamespace(robot_id=robot, object_id=obj, pb_physics_client=client,
                              joint_indices=[v[0] for v in active],
                              joint_lower=np.array([v[8] for v in active]),
                              joint_upper=np.array([v[9] for v in active]))
        self.owner = g.GraspPlanner(env, robot_urdf_path=self.path)
        o = self.owner
        self.by_name = {v[1].decode(): v[0] for v in active}
        self.names = {v[0]: v[12].decode() for v in info}
        self.names[-1] = p.getBodyInfo(robot, physicsClientId=client)[0].decode()
        values = LOGGED_Q.copy()
        if os.environ.get('FETCH_QPOS_JSON'):
            values = json.loads(Path(os.environ['FETCH_QPOS_JSON']).read_text())
            self.assertEqual(set(values), set(ACTION_NAMES), 'Supply all 15 named joint values')
        self.q = np.array([values[name] for name in o.action_names], dtype=float)
        self.assertTrue(np.isfinite(self.q).all())
        # Set independently by joint NAME, not through planner._set().
        for name, value in zip(o.action_names, self.q):
            p.resetJointState(robot, self.by_name[name], float(value), physicsClientId=client)
        self.p = p
        self.owner.obstacles = [obj]

    def _pairs(self):
        o = self.owner
        result = {}
        for c in self.p.getClosestPoints(o.robot, o.robot, .10, physicsClientId=o.client):
            key = tuple(sorted((c[3], c[4])))
            if frozenset(key) in o.self_pairs:
                result[key] = min(result.get(key, float('inf')), float(c[8]))
        return result

    def _description(self, pairs):
        rows = sorted(pairs.items(), key=lambda row: row[1])[:8]
        return '; '.join(f'{a}:{self.names[a]} <-> {b}:{self.names[b]} = {d*1000:.3f} mm'
                         for (a, b), d in rows) or 'no checked pairs within 100 mm'

    def _blocked(self):
        return {pair: d for pair, d in self._pairs().items()
                if d < self.owner.config.collision_margin}

    def test_logged_parked_pose_is_self_collision_free(self):
        pairs = self._blocked()
        print('\n[REAL PARKED POSE]', self._description(pairs), flush=True)
        self.assertFalse(pairs, 'Starting pose violates clearance: ' + self._description(pairs))

    def test_requested_travel_pose_is_self_collision_free(self):
        q = self.q.copy()
        # q[self.owner.arm_indices] = [0., -1., 0., -2., 0., 1., 0.]
        # Matches your updated moving_pose(): elbow_flex = -1.0.
        q[self.owner.arm_indices] = [0., -1., 0., -1., 0., 1., 0.]
        self.owner._set(q)
        pairs = self._blocked()
        print('\n[REAL TRAVEL POSE]', self._description(pairs), flush=True)
        self.assertFalse(pairs, 'Travel target violates clearance: ' + self._description(pairs))

    def test_joint_mapping_roundtrip_preserves_geometry(self):
        o, p = self.owner, self.p
        before = {j: p.getLinkState(o.robot, j, computeForwardKinematics=True,
                                    physicsClientId=o.client)[4:6] for j in o.info}
        pairs = self._pairs()
        o._set(self.q.copy())
        actual = [s[0] for s in p.getJointStates(o.robot, o.action_ids, physicsClientId=o.client)]
        np.testing.assert_allclose(actual, self.q, atol=1e-10, rtol=0)
        for j in o.info:
            after = p.getLinkState(o.robot, j, computeForwardKinematics=True,
                                  physicsClientId=o.client)[4:6]
            np.testing.assert_allclose(after[0], before[j][0], atol=1e-7, rtol=0,
                                       err_msg=f'Link shifted after _set: {self.names[j]}')
            self.assertLess((Rotation.from_quat(before[j][1]).inv()*Rotation.from_quat(after[1])).magnitude(), 1e-6)
        self._assert_same_pairs(pairs, self._pairs())

    def _assert_same_pairs(self, before, after):
        self.assertEqual(set(before), set(after), 'Near-contact pair set changed')
        for pair in before:
            self.assertAlmostEqual(before[pair], after[pair], delta=1e-5,
                                   msg=f'Distance changed for {pair}: {self._description(after)}')

    def test_base_xy_offset_does_not_change_self_collision(self):
        before = self._pairs()
        q = self.q.copy()
        q[self.owner.action_names.index('root_x_axis_joint')] += 1.0
        q[self.owner.action_names.index('root_y_axis_joint')] -= .5
        self.owner._set(q)
        self._assert_same_pairs(before, self._pairs())

    def test_opening_sweep_identifies_first_blocked_sample(self):
        o = self.owner
        goal = self.q.copy(); goal[o.finger_indices] = o.upper[o.finger_indices]
        for i, q in enumerate(g.edge_samples(self.q, goal, .002)):
            o._set(q)
            pairs = self._blocked()
            self.assertFalse(pairs, f'Opening sweep sample {i} (0 = unchanged start): '
                             + self._description(pairs))

    def test_planner_agrees_with_direct_self_collision_query(self):
        o = self.owner
        self.assertTrue(hasattr(o, '_configure_measured_limits'), 'Apply the limit-tolerance patch first')
        self.assertTrue(o._configure_measured_limits(self.q), str(o.last_stats))
        blocked = self._blocked()
        self.assertEqual(o._valid(self.q), not bool(blocked),
                         'Planner/direct-query mismatch: ' + self._description(blocked))

    def test_ik_can_reproduce_current_pose_with_base_and_torso_frozen(self):
        o, p = self.owner, self.p
        ik = g._ArmIK(o, self.q); self.addCleanup(ik.close)
        before = p.getLinkState(o.robot, o.ee, computeForwardKinematics=True, physicsClientId=o.client)
        seed = self.q[o.arm_indices]
        lower = np.maximum(o.lower[o.arm_indices], seed-np.pi)
        upper = np.minimum(o.upper[o.arm_indices], seed+np.pi)
        answer = ik.solve(np.array(before[4]), np.array(before[5]), seed.copy(), lower, upper,
                          time.monotonic()+10)
        self.assertIsNotNone(answer, 'IK cannot reproduce a known achievable FK pose; check frozen transforms/DOF order')
        q = self.q.copy(); q[o.arm_indices] = answer
        self.assertTrue(o._live_pose_ok(q, np.array(before[4]), np.array(before[5])),
                        'Private IK and real URDF FK disagree')
        measured = np.array([s[0] for s in p.getJointStates(o.robot, o.action_ids, physicsClientId=o.client)])
        pinned = [i for i in range(len(q)) if i not in o.arm_indices]
        np.testing.assert_allclose(measured[pinned], self.q[pinned], atol=1e-8, rtol=0)

    def test_report_arm_sensitivity_without_executing_motion(self):
        # Report-only: candidates are diagnostic probes, NOT safe replacement
        # poses. No path to them is certified; no simulation steps are taken.
        o = self.owner
        print('\n[POSE SENSITIVITY; metres, NOT a recovery plan]', flush=True)
        print('baseline:', self._description(self._pairs()), flush=True)
        for name in ('shoulder_lift_joint', 'elbow_flex_joint', 'wrist_flex_joint'):
            index = o.action_names.index(name)
            for delta in (-.15, .15):
                q = self.q.copy(); q[index] += delta
                if not o.lower[index] <= q[index] <= o.upper[index]: continue
                o._set(q)
                print(f'{name} {delta:+.2f} rad:', self._description(self._pairs()), flush=True)
        o._set(self.q)


# The original BulletTests above intentionally strips mesh geometry and remains
# a kinematic smoke test. FullGeometryTests is the only mesh-collision reproducer.


if __name__ == '__main__':
    unittest.main()
