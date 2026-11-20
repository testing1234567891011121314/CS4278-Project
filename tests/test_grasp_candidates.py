"""Grasp candidates on synthetic point clouds. No simulator. Segments 1 and 2."""
import unittest


@unittest.skip("segment 1 not implemented")
class AntipodalCandidates(unittest.TestCase):
    def test_box_gives_pairs_across_opposite_faces(self):
        """R7: on a 40 x 60 x 80 mm box every candidate's contacts lie on
        opposite faces and its width is one of the box's dimensions."""
        self.fail("test not written")

    def test_cylinder_gives_pairs_across_the_diameter(self):
        """R7: closing axes pass within a few millimetres of the cylinder's axis."""
        self.fail("test not written")

    def test_sphere_gives_pairs_through_the_centre(self):
        """R7"""
        self.fail("test not written")

    def test_nothing_wider_than_the_open_gripper(self):
        """R7: a 120 mm cube gives no antipodal candidate."""
        self.fail("test not written")

    def test_same_seed_gives_the_same_candidates(self):
        """R24"""
        self.fail("test not written")

    def test_candidates_are_ordered_best_first(self):
        self.fail("test not written")


@unittest.skip("segment 2 not implemented")
class CheapRejection(unittest.TestCase):
    def test_palm_inside_the_object_is_rejected(self):
        """A candidate on a tall box whose depth puts the palm inside it."""
        self.fail("test not written")

    def test_fingertips_below_the_support_are_rejected(self):
        self.fail("test not written")

    def test_a_clear_candidate_is_kept(self):
        self.fail("test not written")


@unittest.skip("segment 2 not implemented")
class FallbackCandidates(unittest.TestCase):
    def test_thin_rim_gets_a_candidate(self):
        """R13: a bowl-like shell with an 8 mm wall."""
        self.fail("test not written")

    def test_object_wider_than_the_gripper_gets_a_candidate(self):
        """R13: a 103 mm cylinder, 140 mm tall."""
        self.fail("test not written")

    def test_choice_does_not_depend_on_the_object_id(self):
        """R4"""
        self.fail("test not written")


if __name__ == "__main__":
    unittest.main()
