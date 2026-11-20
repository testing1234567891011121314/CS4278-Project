"""The base follower against a simple model of the env's base. No simulator.
Segments 1 and 3."""
import unittest

import numpy as np

from custom_libraries.common import Status
from custom_libraries.navigation.base_follower import BaseFollower
from custom_libraries.navigation.parking import travel_order


def drive(follower, start, gain=0.3, substeps=5, blocked_after=None, limit=2000):
    """Run the follower with a base that, like the env's, closes a fixed share
    of the gap to its target in each physics step. Returns the targets, the
    base poses and the final status."""
    pose = np.array(start, dtype=float)
    targets, poses, status = [], [], Status.RUNNING
    for tick in range(limit):
        target, status = follower.step(pose)
        targets.append(target)
        if status is not Status.RUNNING:
            break
        if blocked_after is None or tick < blocked_after:
            for _ in range(substeps):
                pose = pose + gain * (target - pose)
        poses.append(pose.copy())
    return np.array(targets), np.array(poses), status


class BaseFollowing(unittest.TestCase):
    def setUp(self):
        self.path = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])

    def test_step_per_tick_never_exceeds_the_limit(self):
        """R17: the reference moves at most `max_step` and turns at most `max_turn` per tick."""
        follower = BaseFollower(self.path, 0.0, 1.2, max_step=0.04, max_turn=0.06, ramp=0.004)
        targets, _, status = drive(follower, [0.0, 0.0, 0.0])
        self.assertIs(status, Status.DONE)
        moved = np.linalg.norm(np.diff(targets[:, :2], axis=0), axis=1)
        self.assertLessEqual(moved.max(), 0.04 + 1e-9)
        self.assertLessEqual(np.abs(np.diff(targets[:, 2])).max(), 0.06 + 1e-9)

    def test_speed_ramps_up_and_down(self):
        follower = BaseFollower(self.path, 0.0, None, max_step=0.04, max_turn=0.06, ramp=0.004)
        targets, _, _ = drive(follower, [0.0, 0.0, 0.0])
        moved = np.linalg.norm(np.diff(targets[:, :2], axis=0), axis=1)
        moved = moved[moved > 1e-12]
        self.assertLessEqual(np.abs(np.diff(moved)).max(), 0.004 + 0.012)   # a corner shortens one step's chord
        self.assertLess(moved[0], 0.01)
        self.assertLess(moved[-1], 0.01)
        self.assertAlmostEqual(moved.max(), 0.04, places=6)

    def test_done_at_the_end_of_the_path_facing_the_final_yaw(self):
        follower = BaseFollower(self.path, 0.0, 1.2, max_step=0.04, max_turn=0.06, ramp=0.004)
        _, poses, status = drive(follower, [0.0, 0.0, 0.0])
        self.assertIs(status, Status.DONE)
        np.testing.assert_allclose(poses[-1], [1.0, 1.0, 1.2], atol=0.02)

    def test_turn_on_the_spot_takes_its_time(self):
        """A path with no length still turns no faster than `max_turn`."""
        follower = BaseFollower(np.array([[2.0, 3.0], [2.0, 3.0]]), 0.0, np.pi / 2, 0.04, 0.06, 0.004)
        targets, poses, status = drive(follower, [2.0, 3.0, 0.0])
        self.assertIs(status, Status.DONE)
        self.assertLessEqual(np.abs(np.diff(targets[:, 2])).max(), 0.06 + 1e-9)
        self.assertGreater(len(targets), (np.pi / 2) / 0.06)
        np.testing.assert_allclose(poses[-1], [2.0, 3.0, np.pi / 2], atol=0.02)

    def test_turn_goes_the_short_way_round(self):
        follower = BaseFollower(self.path, 3.0, -3.0, 0.04, 0.06, 0.004)
        targets, _, _ = drive(follower, [0.0, 0.0, 3.0])
        self.assertAlmostEqual(targets[-1, 2], 2 * np.pi - 3.0, places=6)

    def test_poses_cover_the_whole_drive(self):
        """The poses handed to the collision check start and end where the drive does."""
        follower = BaseFollower(self.path, 0.5, 1.2, 0.04, 0.06, 0.004)
        poses = follower.poses(0.05)
        np.testing.assert_allclose(poses[0], [0.0, 0.0, 0.5])
        np.testing.assert_allclose(poses[-1], [1.0, 1.0, 1.2])
        self.assertLessEqual(np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1).max(), 0.05 + 1e-9)

    def test_base_that_does_not_follow_fails_within_a_bound(self):
        """R19: a base stuck against a wall is reported, not waited for."""
        follower = BaseFollower(self.path, 0.0, None, 0.04, 0.06, 0.004)
        targets, _, status = drive(follower, [0.0, 0.0, 0.0], blocked_after=10)
        self.assertIs(status, Status.FAILED)
        self.assertIn("fell behind", follower.failure)
        self.assertLess(len(targets), 30)


class TravelOrder(unittest.TestCase):
    def test_grasps_closing_along_the_travel_line_come_first(self):
        from custom_libraries.grasping.candidates import GraspCandidate

        def candidate(closing, score):
            return GraspCandidate("antipodal", np.zeros(3), np.array(closing, dtype=float), np.array([0.0, 0, -1]),
                                  0.05, 0.07, 0.0, score)

        found = [candidate([0, 1, 0], 0.80), candidate([1, 0, 0], 0.75), candidate([0, 1, 0], 0.30)]
        self.assertEqual(travel_order(found, [1.0, 0.0], 0.2), [1, 0, 2])
        self.assertEqual(travel_order(found, [0.0, 1.0], 0.2), [0, 1, 2])
        self.assertEqual(travel_order(found, [1.0, 0.0], 0.0), [0, 1, 2])


@unittest.skip("segment 3 not implemented")
class Parking(unittest.TestCase):
    def test_excluded_poses_are_not_chosen_again(self):
        """R20: a retry parks somewhere else."""
        self.fail("test not written")


if __name__ == "__main__":
    unittest.main()
