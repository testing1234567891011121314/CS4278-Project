"""Numerical inverse kinematics for the torso and arm (R3, R8)."""
from __future__ import annotations

from typing import Optional

import numpy as np

from custom_libraries.common import Deadline
from custom_libraries.robot import RobotModel


def jacobian(robot: RobotModel, qpos) -> np.ndarray:
    """(6, n) Jacobian of the gripper pose with respect to the torso and arm joints."""
    raise NotImplementedError("segment 2")


def solve(robot: RobotModel, position, orientation, seed_qpos, rng: np.random.Generator,
          deadline: Deadline) -> Optional[np.ndarray]:
    """Joint values, within limits, that put the gripper at the world pose
    (`position`, xyzw `orientation`), starting from `seed_qpos` and restarting
    from random configurations until the deadline. The base joints are kept.
    None if no solution is found."""
    raise NotImplementedError("segment 2")
