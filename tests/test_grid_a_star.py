"""Occupancy grid and path search on small hand-made maps. No simulator.
Segments 1 and 3."""
import unittest


@unittest.skip("segment 1 not implemented")
class Grid(unittest.TestCase):
    def test_cubes_become_occupied_cells(self):
        """R14"""
        self.fail("test not written")

    def test_inflation_is_a_disc(self):
        """A single cube grown by a radius: the diagonal neighbour at more
        than the radius stays free."""
        self.fail("test not written")

    def test_points_outside_the_grid_are_not_free(self):
        self.fail("test not written")

    def test_table_can_be_left_out(self):
        self.fail("test not written")

    def test_wall_size_is_measured_from_the_cubes(self):
        """R4: cubes on a 0.5 m pitch give 0.5 m walls."""
        self.fail("test not written")


@unittest.skip("segment 1 not implemented")
class PathSearch(unittest.TestCase):
    def test_shortest_path_through_a_small_maze(self):
        """R14"""
        self.fail("test not written")

    def test_no_route_returns_none(self):
        """R18: no exception for a walled-off goal."""
        self.fail("test not written")

    def test_start_inside_an_inflated_cell_still_finds_a_path(self):
        self.fail("test not written")

    def test_shortened_path_stays_free_on_the_same_grid(self):
        self.fail("test not written")

    def test_route_exists_through_the_course_maze_for_several_seeds(self):
        """R14: mazes built with maze_utils for maze seeds 0 to 9."""
        self.fail("test not written")


if __name__ == "__main__":
    unittest.main()
