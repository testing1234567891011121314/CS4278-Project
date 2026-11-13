"""Own implementation of 8-connected grid A*, with no PyBullet calls.

Usage: path = a_star(start, goal, inflated_occupancy)
Cells are (row, column); zero means free. Returns [] if no path exists.
Costs are in grid-cell units: 1 straight and sqrt(2) diagonally.
"""
from heapq import heappop, heappush
from math import sqrt


def heuristic(start, goal):
    """Octile distance: admissible for the movement costs below."""
    dr = abs(start[0] - goal[0])
    dc = abs(start[1] - goal[1])
    return max(dr, dc) + (sqrt(2) - 1) * min(dr, dc)


def get_neighbors(cell, occupancy):
    row, col = cell
    rows, cols = occupancy.shape
    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1),
                   (-1, -1), (-1, 1), (1, -1), (1, 1)]:
        nr, nc = row + dr, col + dc
        if not (0 <= nr < rows and 0 <= nc < cols):
            continue
        if occupancy[nr, nc] != 0:
            continue
        diagonal = dr != 0 and dc != 0
        if diagonal and (occupancy[row + dr, col] != 0
                         or occupancy[row, col + dc] != 0):
            continue
        yield (nr, nc), sqrt(2) if diagonal else 1.0


def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return path[::-1]


def a_star(start, goal, occupancy):
    start, goal = tuple(start), tuple(goal)
    rows, cols = occupancy.shape
    for name, cell in [('start', start), ('goal', goal)]:
        row, col = cell
        if not (0 <= row < rows and 0 <= col < cols):
            print(f'{name} is outside the map: {cell}')
            return []
        if occupancy[row, col] != 0:
            print(f'{name} is blocked after inflation: {cell}')
            return []

    frontier = [(heuristic(start, goal), 0.0, start)]
    came_from = {}
    g_score = {start: 0.0}
    while frontier:
        _, queued_g, current = heappop(frontier)
        # Ignore old queue entries superseded by an improved route.
        if queued_g > g_score[current]:
            continue
        if current == goal:
            return reconstruct_path(came_from, current)
        for neighbor, cost in get_neighbors(current, occupancy):
            new_g = queued_g + cost
            if new_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = new_g
                heappush(frontier, (
                    new_g + heuristic(neighbor, goal), new_g, neighbor
                ))
    return []
