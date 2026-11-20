"""Segment 4 in the simulator: what happens after a failure."""
import unittest


@unittest.skip("segment 4 not implemented")
class Retry(unittest.TestCase):
    def test_object_dropped_in_the_maze_is_picked_up_again(self):
        """R20: the test opens the gripper during the carry; the solver picks
        the object from the floor and delivers it, or is STOPPED by tick 3,000."""
        self.fail("test not written")

    def test_failed_grasp_tries_another_candidate(self):
        """R20"""
        self.fail("test not written")

    def test_second_attempt_does_not_reuse_the_first_plan(self):
        """R21"""
        self.fail("test not written")

    def test_no_retry_is_started_without_enough_ticks(self):
        """R19"""
        self.fail("test not written")

    def test_object_still_held_after_a_navigation_failure_is_not_put_down(self):
        self.fail("test not written")


if __name__ == "__main__":
    unittest.main()
