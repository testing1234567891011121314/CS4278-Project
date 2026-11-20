"""Drives the base along a path, one target per tick."""
from __future__ import annotations

from typing import Optional

import numpy as np

from custom_libraries.common import Status
from custom_libraries.navigation.grid import OccupancyGrid


class BaseFollower:
    """The env moves the base by position control toward the commanded target,
    so the follower steps a reference along the path with a ramped speed and
    commands that. One per path."""

    def __init__(self, path: np.ndarray, final_yaw: Optional[float], grid: OccupancyGrid, max_step: float,
                 max_turn: float, ramp_ticks: int):
        """`max_step` metres and `max_turn` radians are the most the reference
        moves per tick; the speed ramps up and down over `ramp_ticks`."""
        raise NotImplementedError("segment 1")

    @property
    def failure(self) -> Optional[str]:
        """Why the drive failed, once `step` has returned FAILED."""
        raise NotImplementedError("segment 1")

    def step(self, base_pose) -> tuple[np.ndarray, Status]:
        """The next world (x, y, yaw) target for the base. FAILED if the base
        falls too far behind its reference or stops making progress. DONE when
        it rests at the path's end, facing `final_yaw`."""
        raise NotImplementedError("segment 1")
