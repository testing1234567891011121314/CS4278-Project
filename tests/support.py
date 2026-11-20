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


def make_env(object_idx: int, maze_seed: int = 42, cloud_seed: int = 300):
    """A headless simulator env for one trial, with the point cloud seeded
    from outside the env so the course files stay untouched (R2, R28)."""
    from tools.trial import make_env as build
    return build(object_idx, maze_seed, cloud_seed)


def live_state(env) -> tuple:
    """Everything a planning call could disturb in the env's simulator."""
    import pybullet as p
    client = env.pb_physics_client
    joints = p.getJointStates(env.robot_id, list(range(p.getNumJoints(env.robot_id, physicsClientId=client))),
                              physicsClientId=client)
    return (tuple((j[0], j[1]) for j in joints),
            p.getBasePositionAndOrientation(env.object_id, physicsClientId=client),
            p.getBaseVelocity(env.object_id, physicsClientId=client),
            tuple(c[:9] for c in p.getContactPoints(physicsClientId=client)))
