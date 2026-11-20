"""Shortest path on an occupancy grid (R3, R14)."""
from __future__ import annotations

from typing import Optional

import numpy as np

from custom_libraries.navigation.grid import OccupancyGrid


def find_path(grid: OccupancyGrid, start, goal) -> Optional[np.ndarray]:
    """(K, 2) world points from `start` to `goal` through free cells, or None
    if there is no route. Never raises for an unreachable goal. A start inside
    an occupied cell first leaves by the nearest free cell."""
    raise NotImplementedError("segment 1")


def shorten(path: np.ndarray, grid: OccupancyGrid) -> np.ndarray:
    """`path` with waypoints removed wherever the straight line between their
    neighbours is free on the same grid the path was found on."""
    raise NotImplementedError("segment 1")
