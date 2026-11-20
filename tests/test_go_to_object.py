"""Segment 1 in the simulator: from the start of a trial to a parked base."""
import unittest

import numpy as np
import pybullet as p

from tests.support import live_state, make_env, sim_test
from tools.trial import close_env, quiet


@sim_test
class GoToObject(unittest.TestCase):
    """The foam brick: the solver is stepped until it enters GRASP."""

    @classmethod
    def setUpClass(cls):
        from solver import Config, Solver, State
        cls.env = make_env(object_idx=14)
        cls.solver = Solver(cls.env, Config(seed=300))
        client = cls.env.pb_physics_client
        cls.touched = set()                     # bodies the robot touched, other than the floor
        cls.rest = None
        positions = []
        for _ in range(400):
            with quiet():
                obs, _ = cls.env.step(cls.solver.get_action())
            positions.append(np.asarray(obs["object_pos"], dtype=float))
            if cls.solver.state is State.GO_TO_OBJECT and cls.rest is None:
                cls.rest = positions[-1]
            for c in p.getContactPoints(cls.env.robot_id, physicsClientId=client):
                if c[2] != cls.env.robot_id and p.getAABB(c[2], physicsClientId=client)[1][2] > 0.05:
                    cls.touched.add(c[2])
            if cls.solver.state not in (State.SETTLE, State.GO_TO_OBJECT):
                break
        cls.obs, cls.State = obs, State

    @classmethod
    def tearDownClass(cls):
        cls.solver.close()
        close_env(cls.env)

    def test_parks_within_the_budget(self):
        """R19: GRASP is entered inside the settle and go-to-object budgets."""
        config = self.solver.config
        self.assertIs(self.solver.state, self.State.GRASP)
        self.assertLessEqual(self.solver.tick, config.settle_ticks + config.go_to_object_ticks)
        self.assertEqual(self.solver.report.errors, [])

    def test_robot_touches_neither_table_nor_walls_nor_object(self):
        self.assertEqual(self.touched, set())

    def test_object_is_not_disturbed(self):
        """The object is where it came to rest."""
        self.assertLess(np.linalg.norm(np.asarray(self.obs["object_pos"]) - self.rest), 0.002)

    def test_a_candidate_is_within_reach_from_the_parking_pose(self):
        attempt = self.solver.attempt
        self.assertTrue(attempt.parking.reachable)
        base = self.solver.scene.robot.base_pose(self.obs["qpos"])
        self.assertLess(np.linalg.norm(base[:2] - attempt.parking.pose[:2]), 0.02)
        toward = np.asarray(self.obs["object_pos"])[:2] - base[:2]
        self.assertLess(abs((np.arctan2(toward[1], toward[0]) - base[2] + np.pi) % (2 * np.pi) - np.pi), 0.1)

    def test_scene_is_read_not_assumed(self):
        """R4: the base origin, the tabletop and the goal equal what the env was built with."""
        scene = self.solver.scene
        np.testing.assert_allclose(scene.robot.base_origin, [-2.0, 0.0, 0.0], atol=1e-9)
        self.assertEqual(len(scene.world.obstacles), 1)
        self.assertAlmostEqual(float(scene.world.obstacles[0].upper[2]), 0.626, places=2)
        self.assertAlmostEqual(scene.world.wall_size, 1.0, places=6)
        np.testing.assert_allclose(scene.world.goal, self.env.goal_pos)

    def test_pick_mode_is_table(self):
        self.assertEqual(self.solver.attempt.pick_mode, "table")


@sim_test
class PlanningLeavesTheEnvAlone(unittest.TestCase):
    def test_a_tick_that_plans_changes_nothing_in_the_env(self):
        """R30: the tick that builds the scene and plans the first arm move
        leaves joints, object and contacts of the env's simulator as they were."""
        from solver import Config, Solver
        env = make_env(object_idx=14)
        try:
            solver = Solver(env, Config(seed=300))
            before = live_state(env)
            solver.get_action()                 # builds the planning simulator, solves the ready pose
            self.assertIsNotNone(solver.scene)
            self.assertEqual(live_state(env), before)
            solver.close()
        finally:
            close_env(env)


if __name__ == "__main__":
    unittest.main()
