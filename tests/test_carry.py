"""The carry's geometry and rate limits against a simple model of the env's
base. No simulator. Segment 3."""
import unittest

import numpy as np

from custom_libraries.common import Status
from custom_libraries.navigation import carry
from custom_libraries.robot import RobotModel


def robot() -> RobotModel:
    """The Fetch joint layout, without a simulator behind it."""
    return RobotModel(body_id=0, client_id=0, joint_indices=tuple(range(15)), joint_names=("",) * 15,
                      lower=np.full(15, -400.0), upper=np.full(15, 400.0), wraps=np.zeros(15, dtype=bool),
                      base=(0, 1, 2), torso=3, head=(4, 5), arm=tuple(range(6, 13)), fingers=(13, 14),
                      gripper_link=20, finger_links=(21, 22), base_origin=np.zeros(3))


def plan(route, yaw=0.0, arm=0.7, closing=0.3, roll_sign=-1, step=0.03) -> carry.CarryPlan:
    base = np.array([route[0][0], route[0][1], yaw])
    obj = base[:2] + arm * np.array([np.cos(yaw), np.sin(yaw)])
    return carry.make_plan(np.asarray(route, dtype=float), base, obj, closing, 0.0, roll_sign, step)


L_ROUTE = carry.round_corners(np.array([[0.0, 0.0], [5.0, 0.0], [5.0, 5.0]]), 0.5)


class Geometry(unittest.TestCase):
    def test_rounded_corner_stays_near_the_corner_and_keeps_the_ends(self):
        sharp = np.array([[0.0, 0.0], [5.0, 0.0], [5.0, 5.0]])
        np.testing.assert_allclose(L_ROUTE[0], sharp[0])
        np.testing.assert_allclose(L_ROUTE[-1], sharp[-1])
        gaps = np.linalg.norm(L_ROUTE - [5.0, 0.0], axis=1)
        self.assertLess(gaps.min(), 0.26)                   # the curve cuts the corner by at most a quarter metre
        turns = np.diff(np.unwrap(np.arctan2(*np.diff(L_ROUTE, axis=0).T[::-1])))
        self.assertLess(np.abs(turns).max(), np.radians(25))    # no sharp bend is left

    def test_base_trails_one_arm_behind_and_heads_for_the_object(self):
        p = plan(L_ROUTE)
        for along in np.linspace(p.arm, p.length, 60):
            pose, _ = p.pose(along)
            lead = p.point(along)
            self.assertAlmostEqual(np.linalg.norm(lead - pose[:2]), p.arm, places=3)
            toward = np.arctan2(*(lead - pose[:2])[::-1])
            self.assertAlmostEqual(float(carry.wrap_angle(toward - pose[2] - p.bearing)), 0.0, places=6)

    def test_object_starts_where_it_is(self):
        p = plan(L_ROUTE)
        pose, heading = p.pose(p.arm)
        np.testing.assert_allclose(pose, [0.0, 0.0, 0.0], atol=1e-6)
        self.assertAlmostEqual(heading, 0.0, places=6)

    def test_arm_stays_on_the_inside_of_the_corner(self):
        """The base cuts no farther outside the route than the route itself."""
        p = plan(L_ROUTE)
        for along in np.linspace(p.arm, p.length, 80):
            pose, _ = p.pose(along)
            self.assertLessEqual(pose[0], 5.0 + 1e-6)
            self.assertGreaterEqual(pose[1], -1e-6)

    def test_turn_on_the_spot_points_the_arm_down_the_route(self):
        p = plan(np.array([[0.0, 0.0], [0.0, 4.0]]), yaw=0.2)
        self.assertAlmostEqual(p.turn, np.pi / 2 - 0.2, places=6)

    def test_roll_lays_the_closing_line_along_the_travel(self):
        """closing heading = yaw + closing + sign * (roll - roll_now), up to half a turn."""
        p = plan(L_ROUTE, closing=0.3, roll_sign=-1)
        for yaw, heading in ((0.0, 0.0), (0.4, 1.2), (-1.0, 2.9)):
            roll = p.roll(yaw, heading, near=0.0)
            line = yaw + p.closing + p.roll_sign * (roll - p.roll_now)
            self.assertAlmostEqual(float(carry.half_turn(line - heading)), 0.0, places=6)
            self.assertLessEqual(abs(roll), np.pi / 2 + 1e-9)

    def test_roll_stays_put_when_the_hand_cannot_steer(self):
        p = carry.make_plan(L_ROUTE, [0, 0, 0], [0.7, 0.0], None, 0.5, -1, 0.03)
        self.assertEqual(p.roll_sign, 0)
        self.assertEqual(p.roll(1.0, 2.0, near=0.5), 0.5)

    def test_poses_to_check_cover_the_turn_and_the_drive(self):
        p = plan(L_ROUTE, yaw=1.0)
        poses = carry.poses_to_check(p, p.turn)
        self.assertAlmostEqual(poses[0][0][2], 1.0)
        np.testing.assert_allclose(poses[-1][0][:2], p.pose(p.length)[0][:2], atol=0.06)
        steps = np.linalg.norm(np.diff([q[0][:2] for q in poses], axis=0), axis=1)
        self.assertLess(steps.max(), 0.12)


class Following(unittest.TestCase):
    def run_carry(self, p, rise=0.05, blocked_after=None):
        hold = np.zeros(15)
        follower = carry.CarryFollower(p, robot(), hold, rise, p.turn)
        base = np.array([p.path[0][0], p.path[0][1], p.yaw_now])
        actions, bases, status = [], [], Status.RUNNING
        for tick in range(4000):
            action, status = follower.step(base)
            actions.append(action)
            if status is not Status.RUNNING:
                break
            if blocked_after is None or tick < blocked_after:
                for _ in range(5):                       # the env's base: 30 % of the gap per physics step
                    base = base + 0.3 * (action[:3] - base)
            bases.append(base.copy())
        return follower, np.array(actions), np.array(bases), status

    def test_object_ends_on_the_goal(self):
        p = plan(L_ROUTE)
        follower, actions, bases, status = self.run_carry(p)
        self.assertIs(status, Status.DONE)
        end = bases[-1]
        obj = end[:2] + p.arm * np.array([np.cos(end[2] + p.bearing), np.sin(end[2] + p.bearing)])
        np.testing.assert_allclose(obj, [5.0, 5.0], atol=0.03)

    def test_every_rate_is_within_its_limit(self):
        """R17: the base's step, its turn, the wrist's roll and the torso's rise per tick."""
        p = plan(L_ROUTE, yaw=2.0, step=0.03)
        s = carry.CarrySettings()
        _, actions, _, status = self.run_carry(p, rise=0.08)
        self.assertIs(status, Status.DONE)
        steps = np.abs(np.diff(actions, axis=0))
        self.assertLessEqual(np.linalg.norm(np.diff(actions[:, :2], axis=0), axis=1).max(), 0.03 * 1.35)
        self.assertLessEqual(steps[:, 2].max(), s.max_yaw + 1e-9)
        self.assertLessEqual(steps[:, 12].max(), s.max_roll + 1e-9)
        self.assertLessEqual(steps[:, 3].max(), s.torso_step + 1e-9)
        self.assertAlmostEqual(actions[-1, 3], 0.08, places=6)

    def test_object_speed_ramps_and_respects_the_plan(self):
        p = plan(np.array([[0.0, 0.0], [10.0, 0.0]]), step=0.012)
        _, actions, _, _ = self.run_carry(p, rise=0.0)
        moved = np.linalg.norm(np.diff(actions[:, :2], axis=0), axis=1)
        self.assertAlmostEqual(moved.max(), 0.012, places=6)
        # The last step into the goal is cut short, so the final change may be a little over one ramp.
        self.assertLessEqual(np.abs(np.diff(moved)).max(), 1.6 * carry.CarrySettings().ramp)

    def test_base_that_does_not_follow_fails(self):
        """R19"""
        p = plan(np.array([[0.0, 0.0], [10.0, 0.0]]))
        follower, _, _, status = self.run_carry(p, rise=0.0, blocked_after=30)
        self.assertIs(status, Status.FAILED)
        self.assertIn("fell behind", follower.failure)


if __name__ == "__main__":
    unittest.main()
