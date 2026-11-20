"""A collision-free arm motion that executes one grasp candidate (R8)."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from custom_libraries.collision import CollisionChecker
from custom_libraries.common import Deadline
from custom_libraries.grasping.candidates import GraspCandidate
from custom_libraries.robot import RobotModel
from custom_libraries.world import ObjectState, World

# The phases of a grasp, in order. Each waypoint of a plan belongs to one.
PHASES = ("open", "transit", "approach", "close", "verify", "lift", "hold")


@dataclass(frozen=True)
class GraspPlan:
    candidate: GraspCandidate
    waypoints: np.ndarray            # (K, 15) joint targets
    phases: tuple[str, ...]          # (K,) the phase of each waypoint
    object_position: np.ndarray      # where the object was when this was planned
    object_orientation: np.ndarray
    stats: dict                      # what was tried and why candidates were dropped


def plan_grasp(candidates: list[GraspCandidate], qpos, obj: ObjectState, robot: RobotModel, world: World,
               collision: CollisionChecker, rng: np.random.Generator,
               deadline: Deadline) -> tuple[Optional[GraspPlan], dict]:
    """The first candidate, in order, for which the whole motion plans: cheap
    rejection, pregrasp pose, transit, straight approach, straight lift with
    the object held. Returns the plan (or None) and the statistics."""
    raise NotImplementedError("segment 2")


def plan_transit(q_start, q_goal, robot: RobotModel, collision: CollisionChecker, rng: np.random.Generator,
                 deadline: Deadline) -> Optional[np.ndarray]:
    """A collision-free joint path from `q_start` to `q_goal` (bidirectional
    random tree), shortened. None if none is found before the deadline."""
    raise NotImplementedError("segment 2")


def plan_line(q_start, direction, distance: float, robot: RobotModel, collision: CollisionChecker,
              rng: np.random.Generator, deadline: Deadline, obj: Optional[ObjectState] = None,
              held: bool = False) -> Optional[np.ndarray]:
    """A joint path that moves the gripper `distance` metres along `direction`
    in a straight line, orientation fixed. Used for the approach and the lift."""
    raise NotImplementedError("segment 2")
