"""Follows a grasp plan tick by tick and checks the hold (R9 to R12, R22)."""
from __future__ import annotations

from typing import Optional

import numpy as np

from custom_libraries.common import Status
from custom_libraries.grasping.arm_planner import GraspPlan
from custom_libraries.robot import RobotModel


class GraspExecutor:
    """One per attempt. `step` is called once per tick until it stops running."""

    def __init__(self, plan: GraspPlan, robot: RobotModel, lift_height: float, hold_ticks: int):
        raise NotImplementedError("segment 2")

    @property
    def phase(self) -> str:
        """The phase being executed, one of `arm_planner.PHASES`."""
        raise NotImplementedError("segment 2")

    @property
    def failure(self) -> Optional[str]:
        """Why the grasp failed, once `step` has returned FAILED."""
        raise NotImplementedError("segment 2")

    def step(self, obs) -> tuple[np.ndarray, Status]:
        """The next (15,) action. FAILED if the object moves before the close,
        if the hold is not verified after it, or if the object is lost in the
        lift or the hold. DONE after the object has been held `hold_ticks`
        at or above `lift_height`."""
        raise NotImplementedError("segment 2")


def is_held(robot: RobotModel, object_id: int) -> bool:
    """True while the object is in the hand: for a pinch, both pads touch it."""
    raise NotImplementedError("segment 2")


def grasp_error(robot: RobotModel, plan: GraspPlan, obs) -> tuple[float, float]:
    """Metres and radians between the planned and the actual gripper pose
    relative to the object (R12)."""
    raise NotImplementedError("segment 2")
