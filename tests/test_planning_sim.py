"""The planning simulator against the live one. Segment 2."""
import unittest


@unittest.skip("segment 2 not implemented")
class PlanningSimulator(unittest.TestCase):
    def test_copy_has_the_same_bodies_as_the_live_scene(self):
        """R31: robot, table, maze and object, at the live poses, with the
        same collision shapes."""
        self.fail("test not written")

    def test_sync_reproduces_the_observed_state(self):
        """R31: after `sync`, link poses and the object pose match the live ones."""
        self.fail("test not written")

    def test_planning_leaves_the_live_simulator_untouched(self):
        """R30: joint states, object pose and velocities, and contact points
        of the live client are identical before and after a planning call."""
        self.fail("test not written")

    def test_clearances_agree_with_the_live_simulator(self):
        """For the current pose, distances measured in the copy equal those
        measured in the live client."""
        self.fail("test not written")

    def test_memory_stays_inside_the_cap(self):
        """R29: live env plus planning simulator peak below 2 GB."""
        self.fail("test not written")


if __name__ == "__main__":
    unittest.main()
