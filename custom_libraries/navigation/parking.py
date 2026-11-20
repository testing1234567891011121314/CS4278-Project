"""Where the base stands to pick the object."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from custom_libraries.grasping.candidates import GraspCandidate
from custom_libraries.navigation.grid import OccupancyGrid
from custom_libraries.robot import RobotModel
from custom_libraries.world import ObjectState, World


@dataclass(frozen=True)
class ParkingPose:
    x: float
    y: float
    yaw: float
    reachable: tuple[int, ...]      # indices of the candidates within reach from here


def choose_parking(candidates: list[GraspCandidate], obj: ObjectState, world: World, grid: OccupancyGrid,
                   robot: RobotModel, base_now, exclude: tuple[ParkingPose, ...] = ()) -> Optional[ParkingPose]:
    """A free pose, reachable by a route from `base_now`, from which the most
    and best candidates are within the arm's reach, and at which the base and
    torso clear the table while turning to face the object. `exclude` lists
    poses already tried in this trial. None if there is no such pose."""
    raise NotImplementedError("segment 1")
