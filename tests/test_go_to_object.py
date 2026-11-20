"""Segment 1 in the simulator: from the start of a trial to a parked base."""
import unittest


@unittest.skip("segment 1 not implemented")
class GoToObject(unittest.TestCase):
    def test_parks_within_the_budget_for_a_table_object(self):
        """R19: foam brick; the solver reaches GRASP within the settle and
        go-to-object budgets."""
        self.fail("test not written")

    def test_parks_within_the_budget_for_a_floor_object(self):
        """The mug, which bounces off the table."""
        self.fail("test not written")

    def test_robot_touches_neither_table_nor_walls(self):
        self.fail("test not written")

    def test_object_is_not_disturbed(self):
        """The object's pose at GRASP is its pose at rest."""
        self.fail("test not written")

    def test_a_candidate_is_within_reach_from_the_parking_pose(self):
        self.fail("test not written")

    def test_scene_is_read_not_assumed(self):
        """R4: the robot's base origin, the tabletop height and the goal come
        out equal to the values the env was built with."""
        self.fail("test not written")


if __name__ == "__main__":
    unittest.main()
