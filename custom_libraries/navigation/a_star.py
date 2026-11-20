"""Shortest path on an occupancy grid (R3, R14)."""
from __future__ import annotations

import heapq
from collections import deque
from typing import Optional

import numpy as np

from custom_libraries.navigation.grid import OccupancyGrid

STEPS = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]


def nearest_free_cell(grid: OccupancyGrid, cell, limit: int = 2000) -> Optional[tuple[int, int]]:
    """The free cell closest to `cell` in grid steps, or None within `limit` cells."""
    seen, queue = {cell}, deque([cell])
    while queue and len(seen) <= limit:
        here = queue.popleft()
        if grid.cell_is_free(here):
            return here
        for dr, dc in STEPS:
            nxt = (here[0] + dr, here[1] + dc)
            if nxt not in seen:
                seen.add(nxt)
                queue.append(nxt)
    return None


def find_path(grid: OccupancyGrid, start, goal) -> Optional[np.ndarray]:
    """(K, 2) world points from `start` to `goal` through free cells, or None
    if there is no route. Never raises for an unreachable goal. A start inside
    an occupied cell first leaves by the nearest free cell."""
    start, goal = np.asarray(start, dtype=float)[:2], np.asarray(goal, dtype=float)[:2]
    last = grid.cell(goal)
    if not grid.cell_is_free(last):
        return None
    first = nearest_free_cell(grid, grid.cell(start))
    if first is None:
        return None

    def remaining(cell) -> float:           # octile distance: exact on an empty grid
        dr, dc = abs(cell[0] - last[0]), abs(cell[1] - last[1])
        return max(dr, dc) + (np.sqrt(2.0) - 1.0) * min(dr, dc)

    cost = {first: 0.0}
    parent: dict = {}
    queue = [(remaining(first), 0.0, first)]
    while queue:
        _, spent, here = heapq.heappop(queue)
        if here == last:
            cells = [here]
            while cells[-1] in parent:
                cells.append(parent[cells[-1]])
            points = [grid.centre(c) for c in reversed(cells)]
            return np.array([start] + points[(1 if grid.cell(start) == first else 0):-1] + [goal]) \
                if len(points) > 1 else np.array([start, goal])
        if spent > cost[here]:
            continue
        for dr, dc in STEPS:
            nxt = (here[0] + dr, here[1] + dc)
            if not grid.cell_is_free(nxt):
                continue
            if dr and dc and not (grid.cell_is_free((here[0] + dr, here[1])) and
                                  grid.cell_is_free((here[0], here[1] + dc))):
                continue                    # no cutting across a wall's corner
            total = spent + (np.sqrt(2.0) if dr and dc else 1.0)
            if total < cost.get(nxt, np.inf):
                cost[nxt] = total
                parent[nxt] = here
                heapq.heappush(queue, (total + remaining(nxt), total, nxt))
    return None


def shorten(path: np.ndarray, grid: OccupancyGrid) -> np.ndarray:
    """`path` with waypoints removed wherever the straight line between their
    neighbours is free on the same grid the path was found on."""
    path = np.asarray(path, dtype=float)
    kept, here = [path[0]], 0
    while here < len(path) - 1:
        nxt = len(path) - 1
        while nxt > here + 1 and not grid.segment_is_free(path[here], path[nxt]):
            nxt -= 1
        kept.append(path[nxt])
        here = nxt
    return np.array(kept)


def length(path: np.ndarray) -> float:
    return float(np.linalg.norm(np.diff(np.asarray(path, dtype=float), axis=0), axis=1).sum())
