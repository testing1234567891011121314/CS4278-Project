"""Occupancy grid of the floor, built from the maze cubes and the table (R14)."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from custom_libraries.world import World


@dataclass(frozen=True)
class OccupancyGrid:
    occupied: np.ndarray            # (rows, cols) bool; True where the base centre may not be. Row = y, col = x.
    origin: np.ndarray              # (2,) world xy of the lower corner of cell (0, 0)
    resolution: float               # metres per cell

    @classmethod
    def from_world(cls, world: World, resolution: float, radius: float,
                   obstacle_radius: Optional[float] = None) -> "OccupancyGrid":
        """A cell is occupied if its centre is within `radius` of a maze cube,
        or within `obstacle_radius` of another fixed body's outline (None
        leaves those out, so the base can reach under a table). Walls grow as
        discs, not squares."""
        squares = [(c - world.wall_size / 2.0, c + world.wall_size / 2.0, radius) for c in world.wall_centres]
        if obstacle_radius is not None:
            squares += [(box.lower[:2], box.upper[:2], obstacle_radius) for box in world.obstacles]
        corners = np.array([s[0] for s in squares] + [s[1] for s in squares] +
                           [box.lower[:2] for box in world.obstacles] + [box.upper[:2] for box in world.obstacles])
        origin = np.floor(corners.min(axis=0) / resolution) * resolution - resolution
        cols, rows = (np.ceil((corners.max(axis=0) - origin) / resolution).astype(int) + 1)
        x = origin[0] + (np.arange(cols) + 0.5) * resolution
        y = origin[1] + (np.arange(rows) + 0.5) * resolution
        occupied = np.zeros((rows, cols), dtype=bool)
        for lower, upper, grow in squares:
            dx = np.maximum(0.0, np.maximum(lower[0] - x, x - upper[0]))
            dy = np.maximum(0.0, np.maximum(lower[1] - y, y - upper[1]))
            occupied |= np.hypot(dx[None, :], dy[:, None]) < grow
        return cls(occupied=occupied, origin=origin, resolution=float(resolution))

    def cell(self, xy) -> tuple[int, int]:
        """The (row, col) containing world point `xy`. May lie outside the grid."""
        col, row = np.floor((np.asarray(xy, dtype=float)[:2] - self.origin) / self.resolution).astype(int)
        return int(row), int(col)

    def centre(self, cell) -> np.ndarray:
        """World xy of the centre of `cell`."""
        return self.origin + (np.array([cell[1], cell[0]]) + 0.5) * self.resolution

    def inside(self, cell) -> bool:
        return 0 <= cell[0] < self.occupied.shape[0] and 0 <= cell[1] < self.occupied.shape[1]

    def cell_is_free(self, cell) -> bool:
        return self.inside(cell) and not self.occupied[cell[0], cell[1]]

    def is_free(self, xy) -> bool:
        """False for occupied cells and for points outside the grid."""
        return self.cell_is_free(self.cell(xy))

    def segment_is_free(self, a, b) -> bool:
        """True if the straight line from `a` to `b` crosses no occupied cell."""
        a, b = np.asarray(a, dtype=float)[:2], np.asarray(b, dtype=float)[:2]
        pieces = max(1, int(np.ceil(np.linalg.norm(b - a) / (self.resolution / 4.0))))
        return all(self.is_free(a + t * (b - a)) for t in np.linspace(0.0, 1.0, pieces + 1))
