"""Run: python -m unittest discover -s . -p test_grasp.py -v

Pure geometry/RRT/controller tests run without Bullet. Bullet smoke tests use
fetch.urdf (set FETCH_URDF or place it beside this file); mesh geometry is removed
in that smoke fixture, so it tests kinematics, not full Fetch collision geometry.
"""
import importlib.util
import os
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import custom_libraries.grasp as g


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
        self.path = Path(os.environ.get('FETCH_URDF', Path(__file__).with_name('fetch.urdf')))
        if not self.path.is_file(): self.skipTest('Set FETCH_URDF to the supplied fetch.urdf')
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


if __name__ == '__main__':
    unittest.main()
