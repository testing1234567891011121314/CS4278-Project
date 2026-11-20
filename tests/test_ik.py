"""Inverse kinematics against forward kinematics. No simulator. Segment 2."""
import unittest


@unittest.skip("segment 2 not implemented")
class InverseKinematics(unittest.TestCase):
    def test_solution_reproduces_a_pose_made_by_forward_kinematics(self):
        """R8: for random joint values inside the limits, solving for the pose
        they give returns joints whose pose matches within 1 mm and 0.5 degrees."""
        self.fail("test not written")

    def test_solution_respects_the_joint_limits(self):
        """R8"""
        self.fail("test not written")

    def test_unreachable_pose_returns_none(self):
        """R8: a pose 3 m from the base."""
        self.fail("test not written")

    def test_base_joints_are_not_moved(self):
        self.fail("test not written")

    def test_deadline_is_respected(self):
        """R26"""
        self.fail("test not written")

    def test_same_seed_gives_the_same_solution(self):
        """R24"""
        self.fail("test not written")


if __name__ == "__main__":
    unittest.main()
