"""The state machine's spine: action shape, the exception guard, tick budgets,
fresh attempts, latched terminal pose. No simulator. Segment 4 in docs/plan.md.
"""
import unittest

import numpy as np

from solver import Config, Solver, State
from tests.support import FakeEnv


def run(solver, env, ticks):
    for _ in range(ticks):
        env.step(solver.get_action())


def stay(action):
    """A state handler that commands `action` and never leaves its state."""
    return lambda obs: (np.asarray(action, dtype=float), None)


class ActionContract(unittest.TestCase):
    def test_action_is_15_finite_joint_targets(self):
        """R1"""
        env = FakeEnv()
        action = Solver(env).get_action()
        self.assertIsInstance(action, np.ndarray)
        self.assertEqual(action.shape, (15,))
        self.assertTrue(np.all(np.isfinite(action)))

    def test_env_may_be_passed_to_get_action(self):
        """R1: the old call form, `solver.get_action(env)`, still works."""
        env = FakeEnv()
        self.assertEqual(Solver(env).get_action(env).shape, (15,))

    def test_returned_action_is_not_the_latched_array(self):
        env = FakeEnv()
        solver = Solver(env)
        solver.get_action()[:] = 99.0
        self.assertFalse(np.any(solver.get_action() == 99.0))


class ExceptionGuard(unittest.TestCase):
    def test_exception_in_a_state_becomes_a_retry(self):
        """R18, R20"""
        env = FakeEnv()
        solver = Solver(env)

        def broken(obs):
            raise ValueError("boom")

        solver._handlers[State.SETTLE] = broken
        action = solver.get_action()
        self.assertEqual(action.shape, (15,))
        self.assertIs(solver.state, State.RETRY)
        self.assertEqual(solver.attempt.failure, "exception")
        self.assertEqual(len(solver.report.errors), 1)
        self.assertIn("ValueError: boom", solver.report.errors[0][2])

    def test_exception_in_retry_stops_the_solver(self):
        """R18: a failure of the retry itself cannot loop."""
        env = FakeEnv()
        solver = Solver(env, start=State.RETRY)

        def broken(obs):
            raise ValueError("boom")

        solver._handlers[State.RETRY] = broken
        run(solver, env, 3)
        self.assertIs(solver.state, State.STOPPED)
        self.assertEqual(len(solver.report.errors), 1)

    def test_failed_observation_does_not_raise(self):
        """R18"""
        env = FakeEnv()
        solver = Solver(env)
        env.fail_obs = True
        self.assertEqual(solver.get_action().shape, (15,))

    def test_strict_lets_the_exception_out(self):
        env = FakeEnv()
        solver = Solver(env, Config(strict=True))
        with self.assertRaises(NotImplementedError):
            solver.get_action()

    def test_stub_solver_stops_cleanly(self):
        """With every state unimplemented the solver still returns actions and
        ends STOPPED, holding the pose it started in."""
        env = FakeEnv()
        env.qpos = np.linspace(0.0, 1.4, 15)
        solver = Solver(env)
        start = env.qpos.copy()
        run(solver, env, 10)
        self.assertIs(solver.state, State.STOPPED)
        np.testing.assert_array_equal(env.qpos, start)


class Budgets(unittest.TestCase):
    def test_state_that_overruns_its_budget_fails(self):
        """R19"""
        env = FakeEnv()
        solver = Solver(env, Config(settle_ticks=5))
        solver._handlers[State.SETTLE] = stay(np.zeros(15))
        run(solver, env, 5)
        self.assertIs(solver.state, State.SETTLE)
        run(solver, env, 1)
        self.assertIs(solver.state, State.RETRY)
        self.assertEqual(solver.attempt.failure, "settle budget spent")

    def test_solver_stops_at_the_episode_budget(self):
        """R16, R19"""
        env = FakeEnv()
        solver = Solver(env, Config(episode_ticks=20, settle_ticks=10_000))
        solver._handlers[State.SETTLE] = stay(np.zeros(15))
        run(solver, env, 20)
        self.assertIs(solver.state, State.SETTLE)
        run(solver, env, 1)
        self.assertIs(solver.state, State.STOPPED)

    def test_every_working_state_has_a_budget_below_the_episode(self):
        """R19"""
        config = Config()
        for state in (State.SETTLE, State.GO_TO_OBJECT, State.GRASP, State.NAVIGATE, State.REACH):
            self.assertLess(config.budget(state), config.episode_ticks, state)

    def test_first_attempt_can_lift_by_the_deadline(self):
        """R9: the budgets up to the lift fit inside the lift deadline."""
        config = Config()
        to_lift = config.settle_ticks + config.go_to_object_ticks + config.grasp_ticks
        self.assertLessEqual(to_lift, config.lift_deadline_tick)


class TerminalPose(unittest.TestCase):
    def test_done_holds_the_last_commanded_action(self):
        """R23: the latched command, not the measured pose, which sags."""
        env = FakeEnv()
        solver = Solver(env, start=State.REACH)
        target = np.full(15, 0.25)
        solver._handlers[State.REACH] = lambda obs: (target, State.DONE)
        solver.get_action()
        self.assertIs(solver.state, State.DONE)
        env.qpos = target - 0.01            # the robot sags below the command
        for _ in range(5):
            np.testing.assert_array_equal(solver.get_action(), target)


class Attempts(unittest.TestCase):
    def test_new_attempt_shares_nothing_with_the_last(self):
        """R21"""
        solver = Solver(FakeEnv())
        first = solver.attempt
        first.pick_mode = "floor"
        first.candidates.append("stale")
        first.failure = "no plan"
        solver.tick = 123
        solver._new_attempt()
        second = solver.attempt
        self.assertIsNot(second, first)
        self.assertEqual((second.index, second.started_tick), (1, 123))
        self.assertEqual(second.candidates, [])
        self.assertIsNone(second.pick_mode)
        self.assertIsNone(second.failure)
        self.assertEqual(solver.report.attempts,
                         [dict(index=0, started_tick=0, pick_mode="floor", lifted_tick=None, failure="no plan")])

    def test_config_cannot_be_changed(self):
        """R25"""
        with self.assertRaises(Exception):
            Config().episode_ticks = 1


class Report(unittest.TestCase):
    def test_transitions_are_recorded_with_their_tick(self):
        """R27"""
        env = FakeEnv()
        solver = Solver(env)
        solver._handlers[State.SETTLE] = lambda obs: (np.zeros(15), State.GO_TO_OBJECT)
        solver._handlers[State.GO_TO_OBJECT] = stay(np.zeros(15))
        run(solver, env, 3)
        self.assertEqual(solver.report.transitions, [(0, "settle"), (0, "go_to_object")])
        self.assertEqual(solver.report.settings["episode_ticks"], 3000)


if __name__ == "__main__":
    unittest.main()
