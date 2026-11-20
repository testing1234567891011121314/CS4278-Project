"""Carrying the held object from the table to the maze exit (R14, R16, R17)."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from custom_libraries.collision import CollisionChecker
from custom_libraries.common import Deadline, Status
from custom_libraries.robot import RobotModel
from custom_libraries.world import World


@dataclass(frozen=True)
class CarryPlan:
    retreat: np.ndarray             # (K, 2) world path that backs the base away from the table
    route: np.ndarray               # (M, 2) world path from there to the standing point for the reach
    yaw: np.ndarray                 # (M,) base heading along the route
    arm: np.ndarray                 # (15,) arm, torso and finger targets held during the drive
    max_step: float                 # metres per tick, set from the object's mass and the grip


def plan_carry(qpos, robot: RobotModel, world: World, collision: CollisionChecker, object_mass: float,
               ticks_left: int, rng: np.random.Generator, deadline: Deadline) -> Optional[CarryPlan]:
    """A route for the base with the object held, checked for the robot and
    the object together, that fits in `ticks_left`. None if there is none."""
    raise NotImplementedError("segment 3")


class CarryFollower:
    """Executes a carry plan tick by tick and watches the object."""

    def __init__(self, plan: CarryPlan, robot: RobotModel):
        raise NotImplementedError("segment 3")

    @property
    def failure(self) -> Optional[str]:
        raise NotImplementedError("segment 3")

    def step(self, obs) -> tuple[np.ndarray, Status]:
        """The next (15,) action. FAILED when the object is no longer held or
        the base cannot follow the route. DONE at the end of the route."""
        raise NotImplementedError("segment 3")
