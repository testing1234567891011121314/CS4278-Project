"""Segment 2 in the simulator: from a parked base to a lifted, held object."""
import unittest


@unittest.skip("segment 2 not implemented")
class AttemptGrasp(unittest.TestCase):
    def test_foam_brick_is_lifted_by_the_deadline(self):
        """R9: from the start of the trial, lifted by tick 600."""
        self.fail("test not written")

    def test_lift_is_at_least_10_cm_through_the_hold(self):
        """R10"""
        self.fail("test not written")

    def test_object_is_held_for_50_ticks(self):
        """R11"""
        self.fail("test not written")

    def test_grasp_error_is_reported_and_small(self):
        """R12"""
        self.fail("test not written")

    def test_planning_does_not_move_the_object(self):
        """The object's pose is unchanged across a planning call."""
        self.fail("test not written")

    def test_object_moved_before_the_close_fails_the_attempt(self):
        """R22: the test nudges the object during the approach."""
        self.fail("test not written")

    def test_empty_close_fails_before_the_lift(self):
        """R22: the test removes the object as the fingers close."""
        self.fail("test not written")


if __name__ == "__main__":
    unittest.main()
