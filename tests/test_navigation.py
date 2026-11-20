"""Parking choice and the base follower against a simple base model. No
simulator. Segments 1 and 3."""
import unittest


@unittest.skip("segment 1 not implemented")
class Parking(unittest.TestCase):
    def test_pose_is_free_and_within_reach_of_a_candidate(self):
        self.fail("test not written")

    def test_no_reachable_pose_returns_none(self):
        self.fail("test not written")

    def test_excluded_poses_are_not_chosen_again(self):
        """R20: a retry parks somewhere else."""
        self.fail("test not written")


@unittest.skip("segment 1 not implemented")
class BaseFollowing(unittest.TestCase):
    def test_step_per_tick_never_exceeds_the_limit(self):
        """R17: the reference moves at most `max_step` and `max_turn` per tick."""
        self.fail("test not written")

    def test_speed_ramps_up_and_down(self):
        self.fail("test not written")

    def test_done_at_the_end_of_the_path_facing_the_final_yaw(self):
        self.fail("test not written")

    def test_base_that_does_not_follow_fails_within_a_bound(self):
        """R19: a base stuck against a wall is reported, not waited for."""
        self.fail("test not written")


if __name__ == "__main__":
    unittest.main()
