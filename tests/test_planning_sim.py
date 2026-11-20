"""The planning simulator against the live one (R30, R31). Segment 2."""
import unittest

import numpy as np
import pybullet as p

from tests.support import live_state, make_env, sim_test
from tools.trial import close_env, quiet


@sim_test
class PlanningSimulator(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from custom_libraries.planning_sim import PlanningSim
        cls.env = make_env(object_idx=9)
        target = cls.env._get_obs()["qpos"].astype(float)
        target[[0, 2, 3, 7, 9, 11]] = [0.4, 0.3, 0.2, 0.6, 1.0, -0.5]   # a pose away from all zeros
        with quiet():
            for _ in range(120):                                       # the object lands meanwhile
                cls.obs, _ = cls.env.step(target)
        cls.before = live_state(cls.env)
        cls.sim = PlanningSim(cls.env)

    @classmethod
    def tearDownClass(cls):
        cls.sim.close()
        close_env(cls.env)

    def test_copy_has_the_scene(self):
        """R31: robot, object, floor, table and maze."""
        self.assertEqual(len(self.sim.copies_of), 3)
        self.assertIsNotNone(self.sim.floor_id)

    def test_sync_reproduces_the_observed_state(self):
        """R31: after `sync`, every link and the object are where the live ones are."""
        env, sim, live = self.env, self.sim, self.env.pb_physics_client
        sim.sync(self.obs)
        for link in range(p.getNumJoints(env.robot_id, physicsClientId=live)):
            a = p.getLinkState(env.robot_id, link, computeForwardKinematics=True, physicsClientId=live)[4]
            np.testing.assert_allclose(sim.link_pose(link)[0], a, atol=1e-5)
        a = p.getBasePositionAndOrientation(env.object_id, physicsClientId=live)
        b = p.getBasePositionAndOrientation(sim.object_id, physicsClientId=sim.client)
        np.testing.assert_allclose(b[0], a[0], atol=1e-6)
        np.testing.assert_allclose(b[1], a[1], atol=1e-6)

    def test_clearances_agree_with_the_live_simulator(self):
        """Distances from the robot to each other body are the same in the copy."""
        env, sim, live = self.env, self.sim, self.env.pb_physics_client
        sim.sync(self.obs)

        def nearest(client, robot, bodies):
            return round(min(c[8] for b in bodies for c in p.getClosestPoints(robot, b, 30.0, physicsClientId=client)), 4)

        self.assertEqual(nearest(sim.client, sim.robot_id, [sim.object_id]), nearest(live, env.robot_id, [env.object_id]))
        for body, copies in sim.copies_of.items():
            self.assertEqual(nearest(sim.client, sim.robot_id, copies), nearest(live, env.robot_id, [body]), body)

    def test_planning_leaves_the_live_simulator_untouched(self):
        """R30"""
        sim = self.sim
        rng = np.random.default_rng(0)
        for _ in range(50):
            q = self.obs["qpos"].astype(float)
            q[list(sim.robot.plan_joints)] = sim.robot.sample(rng, sim.robot.plan_joints)
            sim.gripper_pose(q)
            sim.set_object(rng.uniform(-1, 1, 3), [0, 0, 0, 1])
            p.getClosestPoints(sim.robot_id, sim.object_id, 1.0, physicsClientId=sim.client)
        self.assertEqual(live_state(self.env), self.before)

    def test_gripper_is_measured(self):
        """R4: the hand's dimensions come from the collision shapes."""
        g = self.sim.gripper
        self.assertAlmostEqual(g.max_opening, 0.10285, places=4)
        self.assertAlmostEqual(g.min_opening, 0.00285, places=4)
        self.assertAlmostEqual(g.pad_tip - g.pad_back, 0.063, places=4)
        self.assertAlmostEqual(g.pad_width, 0.024, places=4)
        self.assertAlmostEqual(g.pad_tip, 0.0315, places=4)
        self.assertAlmostEqual(g.palm_front, -0.032, places=3)
        self.assertAlmostEqual(g.palm_half[0], 0.063, places=3)
        self.assertAlmostEqual(g.finger_joint(g.max_opening), g.finger_travel, places=6)

    def test_robot_layout(self):
        robot = self.sim.robot
        self.assertEqual(robot.base, (0, 1, 2))
        self.assertEqual(robot.plan_joints, (3, 6, 7, 8, 9, 10, 11, 12))
        self.assertEqual(robot.fingers, (13, 14))
        np.testing.assert_allclose(robot.base_origin, [-2.0, 0.0, 0.0], atol=1e-9)
        q = robot.with_base(np.zeros(15), [1.0, -0.5, 0.7])
        np.testing.assert_allclose(robot.base_pose(q), [1.0, -0.5, 0.7], atol=1e-9)


if __name__ == "__main__":
    unittest.main()
