"""Segment 3 in the simulator: from a held object to a delivered one."""
import unittest


@unittest.skip("segment 3 not implemented")
class NavigateToExit(unittest.TestCase):
    def test_empty_gripper_reaches_the_goal(self):
        """R14: no object in the hand; the gripper ends within 10 cm of the goal."""
        self.fail("test not written")

    def test_held_foam_brick_is_delivered_in_two_mazes(self):
        """R15, R16: the test places the brick in the gripper; delivered
        within the navigate and reach budgets for maze seeds 42 and 1."""
        self.fail("test not written")

    def test_object_is_not_dropped_on_the_way(self):
        """R17"""
        self.fail("test not written")

    def test_robot_and_object_touch_no_wall(self):
        self.fail("test not written")

    def test_done_keeps_the_object_at_the_goal(self):
        """R23: 100 ticks after DONE the object is still within 10 cm."""
        self.fail("test not written")


if __name__ == "__main__":
    unittest.main()
