"""Shared helpers for the tests."""
from __future__ import annotations

import os
import unittest

import numpy as np

SIM = os.environ.get("PNR_SIM_TESTS") == "1"
sim_test = unittest.skipUnless(SIM, "simulator test: set PNR_SIM_TESTS=1 and run one process at a time")


class FakeEnv:
    """Stands in for the env where no physics is needed: a robot that reaches
    every commanded target within the tick, an object that never moves."""

    action_size = 15

    def __init__(self):
        self.qpos = np.zeros(15)
        self.fail_obs = False       # make the next observations raise

    def _get_obs(self) -> dict:
        if self.fail_obs:
            raise RuntimeError("observation failed")
        return dict(
            qpos=self.qpos.copy(),
            qvel=np.zeros(15),
            object_pos=np.array([0.0, 0.0, 0.7]),
            object_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            goal_pos=np.array([17.5, 0.5, 0.9]),
            object_pc=np.zeros((8, 3)),
            object_normals=np.tile([0.0, 0.0, 1.0], (8, 1)),
            cube_positions=np.zeros((0, 2)),
        )

    def step(self, action):
        self.qpos = np.asarray(action, dtype=float).copy()
        return self._get_obs(), dict(dist_to_goal=1.0, success=False)


def make_env(object_idx: int, maze_seed: int, cloud_seed: int):
    """A headless simulator env for one trial, with the point cloud seeded
    from outside the env so the course files stay untouched (R2, R28)."""
    raise NotImplementedError("segment 0")


def place_in_gripper(env, object_pose_in_gripper):
    """Test fixture: put the object between the pads and close them, so the
    segments after the grasp can be tested without it."""
    raise NotImplementedError("segment 0")


def park_at(env, x: float, y: float, yaw: float):
    """Test fixture: put the base at a world pose, so the grasp can be tested
    without the drive to the object."""
    raise NotImplementedError("segment 0")
