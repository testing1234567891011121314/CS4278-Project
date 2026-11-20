"""Occupancy grid and path search on small hand-made maps. No simulator.
Segments 1 and 3."""
import unittest

import numpy as np

from custom_libraries.navigation import a_star
from custom_libraries.navigation.grid import OccupancyGrid
from custom_libraries.world import Box, World


def world(cubes, size=1.0, boxes=()) -> World:
    return World(wall_centres=np.asarray(cubes, dtype=float).reshape(-1, 2), wall_size=size,
                 obstacles=tuple(boxes), floor=0.0, goal=np.zeros(3))


def ring(n=7):
    """Cube centres of a square wall around an (n-2) x (n-2) m room."""
    return [(x + 0.5, y + 0.5) for x in range(n) for y in range(n) if x in (0, n - 1) or y in (0, n - 1)]


class Grid(unittest.TestCase):
    def test_cubes_become_occupied_cells(self):
        """R14"""
        grid = OccupancyGrid.from_world(world([(0.5, 0.5)]), 0.1, 0.0001)
        self.assertFalse(grid.is_free([0.5, 0.5]))
        self.assertFalse(grid.is_free([0.05, 0.95]))
        self.assertTrue(grid.is_free([1.05, 0.5]))

    def test_inflation_is_a_disc(self):
        """Beside the cube the wall grows by the radius; off its corner the
        same offset in both directions is farther than the radius, so free."""
        grid = OccupancyGrid.from_world(world([(0.5, 0.5), (3.5, 3.5)]), 0.05, 0.30)
        self.assertFalse(grid.is_free([1.275, 0.5]))
        self.assertTrue(grid.is_free([1.325, 0.5]))
        self.assertTrue(grid.is_free([1.225, 1.225]))       # 0.318 m from the corner
        self.assertFalse(grid.is_free([1.175, 1.175]))      # 0.247 m from the corner

    def test_points_outside_the_grid_are_not_free(self):
        grid = OccupancyGrid.from_world(world(ring()), 0.1, 0.3)
        self.assertFalse(grid.is_free([-50.0, 0.0]))
        self.assertFalse(grid.is_free([3.5, 500.0]))

    def test_other_bodies_can_be_left_out(self):
        table = Box(np.array([3.0, 3.0, 0.57]), np.array([4.0, 4.0, 0.63]))
        without = OccupancyGrid.from_world(world(ring(), boxes=[table]), 0.1, 0.3)
        grown = OccupancyGrid.from_world(world(ring(), boxes=[table]), 0.1, 0.3, obstacle_radius=0.2)
        self.assertTrue(without.is_free([3.5, 3.5]))
        self.assertFalse(grown.is_free([3.5, 3.5]))
        self.assertFalse(grown.is_free([2.85, 3.5]))
        self.assertTrue(grown.is_free([2.75, 3.5]))

    def test_wall_size_is_measured_from_the_cubes(self):
        """R4: cubes on a 0.5 m pitch give 0.5 m walls."""
        from custom_libraries.world import _pitch
        self.assertAlmostEqual(_pitch(np.array([(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (0.0, 2.0)])), 0.5)

    def test_cell_and_centre_agree(self):
        grid = OccupancyGrid.from_world(world(ring()), 0.1, 0.3)
        for xy in ([3.21, 2.87], [1.0, 1.0], [5.99, 0.01]):
            centre = grid.centre(grid.cell(xy))
            self.assertTrue(np.all(np.abs(centre - xy) <= 0.05 + 1e-9))


class PathSearch(unittest.TestCase):
    def setUp(self):
        # A 5 x 5 m room with a wall across the middle and a gap at the top.
        cubes = ring() + [(3.5, y + 0.5) for y in range(1, 4)]
        self.grid = OccupancyGrid.from_world(world(cubes), 0.1, 0.3)

    def test_path_goes_round_the_wall(self):
        """R14"""
        path = a_star.find_path(self.grid, [2.0, 2.0], [5.0, 2.0])
        self.assertIsNotNone(path)
        np.testing.assert_allclose(path[0], [2.0, 2.0])
        np.testing.assert_allclose(path[-1], [5.0, 2.0])
        self.assertGreater(path[:, 1].max(), 4.3)           # through the gap above the wall
        for a, b in zip(path[:-1], path[1:]):
            self.assertTrue(self.grid.segment_is_free(a, b))

    def test_path_is_the_shortest_on_an_open_floor(self):
        grid = OccupancyGrid.from_world(world(ring(12)), 0.1, 0.3)
        path = a_star.shorten(a_star.find_path(grid, [2.05, 2.05], [9.05, 6.05]), grid)
        self.assertLess(a_star.length(path), np.hypot(7.0, 4.0) * 1.01)

    def test_no_route_returns_none(self):
        """R18: no exception for a walled-off goal."""
        cubes = ring() + [(3.5, y + 0.5) for y in range(1, 6)]
        grid = OccupancyGrid.from_world(world(cubes), 0.1, 0.3)
        self.assertIsNone(a_star.find_path(grid, [2.0, 2.0], [5.0, 2.0]))

    def test_goal_in_a_wall_returns_none(self):
        self.assertIsNone(a_star.find_path(self.grid, [2.0, 2.0], [3.5, 2.0]))

    def test_start_inside_an_inflated_cell_still_finds_a_path(self):
        path = a_star.find_path(self.grid, [1.1, 2.0], [5.0, 2.0])
        self.assertIsNotNone(path)
        np.testing.assert_allclose(path[0], [1.1, 2.0])
        self.assertTrue(self.grid.is_free(path[1]))

    def test_shortened_path_stays_free_on_the_same_grid(self):
        path = a_star.find_path(self.grid, [2.0, 2.0], [5.0, 2.0])
        short = a_star.shorten(path, self.grid)
        self.assertLess(len(short), len(path))
        self.assertLessEqual(a_star.length(short), a_star.length(path) + 1e-9)
        for a, b in zip(short[:-1], short[1:]):
            self.assertTrue(self.grid.segment_is_free(a, b))

    def test_route_exists_through_the_course_maze_for_several_seeds(self):
        """R14: mazes built as the env builds them, for ten entrance and exit choices."""
        from maze_utils import add_left_room_to_maze, generate_maze_map
        rows, cols, room = 12, 16, 6
        for seed in range(10):
            rng = np.random.default_rng(seed)
            row_in, row_out = (int(rng.integers(1, rows - 1)) for _ in range(2))
            maze = generate_maze_map(n_rows=rows, n_cols=cols, in_pos=(row_in, 0), out_pos=(row_out, cols - 1),
                                     grid_density=0.8, add_x_min_wall=True, add_x_max_wall=True,
                                     add_y_min_wall=True, add_y_max_wall=True, p_greedy=0.6, seed=5)
            maze = np.asarray(add_left_room_to_maze(maze, room_width=room))
            r, c = np.nonzero(maze)
            cubes = np.column_stack([c + 0.5 - room + 2.0 - 0.0, rows / 2.0 - r - 0.5])
            grid = OccupancyGrid.from_world(world(cubes), 0.1, 0.35)
            goal = [cols - 0.5 + 2.0, rows / 2.0 - row_out - 0.5]
            self.assertIsNotNone(a_star.find_path(grid, [-2.0, 0.0], goal), seed)


if __name__ == "__main__":
    unittest.main()
