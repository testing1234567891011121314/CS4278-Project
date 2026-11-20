"""Segment 2 and the back-off in the simulator: from a parked base to a lifted,
held object carried away from the table. The outcome is measured from the
simulator by tools/pick_all.py, not taken from the solver."""
import sys
import unittest

from tests.support import sim_test
from tools.trial import ROOT

sys.path.insert(0, str(ROOT / "tools"))


@sim_test
class FoamBrick(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import pick_all
        cls.row = pick_all.run(object_idx=14, maze_seed=42, cloud_seed=300, ticks=700, verbose=False)

    def test_lifted_by_the_deadline(self):
        """R9: at least 10 cm up, with both pads on it, by tick 600."""
        self.assertIsNotNone(self.row["lift_tick"])
        self.assertLessEqual(self.row["lift_tick"], 600)

    def test_held_at_least_10_cm_up_for_50_ticks(self):
        """R10, R11"""
        self.assertTrue(self.row["hold_50"])

    def test_grasp_is_where_it_was_planned(self):
        """R12: the hand is within 10 mm of its planned place on the object."""
        self.assertLess(self.row["grasp_error_mm"], 10.0)

    def test_backed_off_with_the_object_in_the_hand(self):
        self.assertEqual(self.row["state"], "navigate")
        self.assertTrue(self.row["held_at_end"])
        self.assertGreaterEqual(self.row["backed_off_m"], 0.3)

    def test_first_attempt_succeeds_without_an_error(self):
        self.assertEqual(self.row["attempts"], 1)
        self.assertEqual(self.row["errors"], 0)
        self.assertIsNone(self.row["failure"])


@unittest.skip("not written yet")
class GraspFailures(unittest.TestCase):
    def test_object_moved_before_the_close_fails_the_attempt(self):
        """R22: the test nudges the object during the approach."""
        self.fail("test not written")

    def test_empty_close_fails_before_the_lift(self):
        """R22: the test removes the object as the fingers close."""
        self.fail("test not written")


if __name__ == "__main__":
    unittest.main()
