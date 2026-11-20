"""Occupancy grid of the floor, built from the maze cubes and the table (R14)."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from custom_libraries.world import World


@dataclass(frozen=True)
class OccupancyGrid:
    occupied: np.ndarray            # (rows, cols) bool; True where the base centre may not be
    origin: np.ndarray              # (2,) world xy of the corner of cell (0, 0)
    resolution: float               # metres per cell

    @classmethod
    def from_world(cls, world: World, resolution: float, radius: float,
                   include_table: bool = True) -> "OccupancyGrid":
        """Walls (and the table, unless left out so the base can reach under
        it) grown by `radius` as a disc."""
        raise NotImplementedError("segment 1")

    def cell(self, xy) -> tuple[int, int]:
        """The (row, col) containing world point `xy`."""
        raise NotImplementedError("segment 1")

    def centre(self, cell) -> np.ndarray:
        """World xy of the centre of `cell`."""
        raise NotImplementedError("segment 1")

    def is_free(self, xy) -> bool:
        """False for occupied cells and for points outside the grid."""
        raise NotImplementedError("segment 1")

    def segment_is_free(self, a, b) -> bool:
        """True if the straight line from `a` to `b` crosses no occupied cell."""
        raise NotImplementedError("segment 1")
