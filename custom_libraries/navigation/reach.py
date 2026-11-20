"""Bringing the held object to the goal (R15)."""
from __future__ import annotations

from typing import Optional

import numpy as np

from custom_libraries.collision import CollisionChecker
from custom_libraries.common import Budget, Status
from custom_libraries.robot import RobotModel
from custom_libraries.world import World


def plan_reach(qpos, obs, robot: RobotModel, world: World, collision: CollisionChecker,
               rng: np.random.Generator, budget: Budget) -> Optional[np.ndarray]:
    """A (K, 15) joint path that puts the object's centre, not the gripper, on
    the goal, keeping the gripper's orientation so the object is not tipped
    out. None if the goal is out of reach from where the base stands."""
    raise NotImplementedError("segment 3")


class ReachFollower:
    def __init__(self, path: np.ndarray, robot: RobotModel, goal_tolerance: float):
        raise NotImplementedError("segment 3")

    @property
    def failure(self) -> Optional[str]:
        raise NotImplementedError("segment 3")

    def step(self, obs) -> tuple[np.ndarray, Status]:
        """The next (15,) action. DONE once the object is within the goal
        tolerance. FAILED if the object is lost or the path ends short."""
        raise NotImplementedError("segment 3")
