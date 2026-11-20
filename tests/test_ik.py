"""Inverse kinematics against the planning simulator's link poses. Segment 2."""
import unittest

import numpy as np

from custom_libraries.common import Budget
from custom_libraries.grasping import ik
from tests.support import make_env, sim_test
from tools.trial import close_env


class RotationVector(unittest.TestCase):
    def test_round_trip(self):
        rng = np.random.default_rng(0)
        for _ in range(50):
            v = rng.normal(size=3)
            v *= rng.uniform(0.0, np.pi - 1e-3) / np.linalg.norm(v)
            angle = np.linalg.norm(v)
            k = v / angle
            cross = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
            matrix = np.eye(3) + np.sin(angle) * cross + (1 - np.cos(angle)) * cross @ cross
            np.testing.assert_allclose(ik.rotation_vector(matrix), v, atol=1e-7)

    def test_half_turn(self):
        matrix = np.diag([1.0, -1.0, -1.0])
        np.testing.assert_allclose(np.abs(ik.rotation_vector(matrix)), [np.pi, 0, 0], atol=1e-6)


@sim_test
class InverseKinematics(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from custom_libraries.planning_sim import PlanningSim
        cls.env = make_env(object_idx=14)
        cls.sim = PlanningSim(cls.env)
        cls.robot = cls.sim.robot
        cls.home = cls.env._get_obs()["qpos"].astype(float)

    @classmethod
    def tearDownClass(cls):
        cls.sim.close()
        close_env(cls.env)

    def random_pose(self, rng):
        q = self.home.copy()
        q[list(self.robot.plan_joints)] = self.robot.sample(rng, self.robot.plan_joints)
        return q, self.sim.gripper_pose(q)

    def test_jacobian_matches_finite_differences(self):
        rng = np.random.default_rng(1)
        q, (position, turn) = self.random_pose(rng)
        j = ik.jacobian(self.sim, q)
        for column, k in enumerate(self.robot.plan_joints):
            moved = q.copy()
            moved[k] += 1e-3                # link poses come back in single precision
            there, facing = self.sim.gripper_pose(moved)
            np.testing.assert_allclose(j[:3, column], (there - position) / 1e-3, atol=3e-3)
            np.testing.assert_allclose(j[3:, column], ik.rotation_vector(facing @ turn.T) / 1e-3, atol=3e-3)

    def test_solution_reproduces_a_pose_made_by_forward_kinematics(self):
        """R8: within 1 mm and 0.005 rad, for 30 random reachable poses."""
        rng = np.random.default_rng(2)
        found = 0
        for _ in range(30):
            _, (position, turn) = self.random_pose(rng)
            q = ik.solve(self.sim, position, turn, self.home, rng, Budget(2000))
            if q is None:
                continue
            found += 1
            there, facing = self.sim.gripper_pose(q)
            self.assertLess(np.linalg.norm(there - position), 0.001)
            self.assertLess(np.linalg.norm(ik.rotation_vector(facing @ turn.T)), 0.005)
        self.assertGreaterEqual(found, 27)

    def test_solution_respects_the_joint_limits_and_keeps_the_other_joints(self):
        """R8"""
        rng = np.random.default_rng(3)
        _, (position, turn) = self.random_pose(rng)
        q = ik.solve(self.sim, position, turn, self.home, rng, Budget(2000))
        self.assertIsNotNone(q)
        self.assertTrue(np.all(q >= self.robot.lower - 1e-9) and np.all(q <= self.robot.upper + 1e-9))
        others = [k for k in range(15) if k not in self.robot.plan_joints]
        np.testing.assert_array_equal(q[others], self.home[others])
        turning = [k for k in self.robot.plan_joints if self.robot.wraps[k]]
        self.assertTrue(np.all(np.abs(q[turning] - self.home[turning]) <= np.pi + 1e-9))

    def test_unreachable_pose_returns_none(self):
        """R8: a pose 3 m above the base."""
        rng = np.random.default_rng(4)
        position = np.array([self.robot.base_origin[0], 0.0, 3.0])
        self.assertIsNone(ik.solve(self.sim, position, np.eye(3), self.home, rng, Budget(2000)))

    def test_work_budget_is_respected(self):
        """R26"""
        rng = np.random.default_rng(5)
        self.assertIsNone(ik.solve(self.sim, [0, 0, 3.0], np.eye(3), self.home, rng, Budget(0)))

    def test_same_seed_gives_the_same_solution(self):
        """R24"""
        _, (position, turn) = self.random_pose(np.random.default_rng(6))
        a = ik.solve(self.sim, position, turn, self.home, np.random.default_rng(7), Budget(2000))
        b = ik.solve(self.sim, position, turn, self.home, np.random.default_rng(7), Budget(2000))
        np.testing.assert_array_equal(a, b)


if __name__ == "__main__":
    unittest.main()
