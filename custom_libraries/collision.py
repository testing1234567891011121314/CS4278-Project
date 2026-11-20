"""Clearance queries for the arm and base planners.

Answered in the planning simulator, never in the env's (R30, R31).
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from custom_libraries.planning_sim import PlanningSim
from custom_libraries.robot import RobotModel
from custom_libraries.world import ObjectState, World


class CollisionChecker:
    def __init__(self, robot: RobotModel, world: World, sim: PlanningSim):
        raise NotImplementedError("segment 2")

    def clearance(self, qpos, obj: Optional[ObjectState] = None, held: bool = False) -> float:
        """Smallest distance in metres between the robot at `qpos` and the
        scene, itself, and the object. With `held`, the object moves with the
        gripper and is checked against the scene instead of the fingers."""
        raise NotImplementedError("segment 2")

    def is_free(self, qpos, margin: float, obj: Optional[ObjectState] = None, held: bool = False) -> bool:
        raise NotImplementedError("segment 2")

    def path_is_free(self, path: np.ndarray, margin: float, obj: Optional[ObjectState] = None,
                     held: bool = False) -> bool:
        """True if every configuration along `path`, densely interpolated, is free."""
        raise NotImplementedError("segment 2")
