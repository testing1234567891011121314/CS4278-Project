"""Grasp candidates on synthetic point clouds. No simulator. Segments 1 and 2."""
import unittest

import numpy as np

from custom_libraries.grasping.candidates import CandidateSettings, antipodal_candidates, antipodal_pairs
from custom_libraries.planning_sim import Gripper
from custom_libraries.world import ObjectState

# The Fetch hand, as PlanningSim measures it.
GRIPPER = Gripper(max_opening=0.10285, min_opening=0.00285, finger_travel=0.05, pad_back=-0.0315, pad_tip=0.0315,
                  pad_width=0.024, pad_thickness=0.012, palm_front=-0.032, palm_half=(0.063, 0.0413))
SUPPORT = 0.626


def state(points, normals) -> ObjectState:
    points = np.asarray(points, dtype=float)
    return ObjectState(position=points.mean(axis=0), orientation=np.array([0.0, 0, 0, 1]), cloud=points,
                       normals=np.asarray(normals, dtype=float))


def box(size, count=1024, seed=0, bottom=SUPPORT) -> ObjectState:
    """Points on the faces of an axis-aligned box resting on the support."""
    rng = np.random.default_rng(seed)
    half = np.asarray(size, dtype=float) / 2.0
    areas = np.array([half[1] * half[2], half[0] * half[2], half[0] * half[1]]).repeat(2)
    faces = rng.choice(6, size=count, p=areas / areas.sum())
    points = rng.uniform(-1.0, 1.0, (count, 3)) * half
    normals = np.zeros((count, 3))
    axis, sign = faces // 2, 1.0 - 2.0 * (faces % 2)
    points[np.arange(count), axis] = sign * half[axis]
    normals[np.arange(count), axis] = sign
    return state(points + [0.0, 0.0, bottom + half[2]], normals)


def cylinder(radius, height, count=1024, seed=0, bottom=SUPPORT) -> ObjectState:
    """Points on an upright cylinder, side and both caps."""
    rng = np.random.default_rng(seed)
    side = rng.random(count) < (2 * height) / (2 * height + 2 * radius)
    angle = rng.uniform(0, 2 * np.pi, count)
    ring = np.column_stack([np.cos(angle), np.sin(angle)])
    r = np.where(side, radius, radius * np.sqrt(rng.random(count)))
    top = rng.random(count) < 0.5
    z = np.where(side, rng.uniform(0, height, count), np.where(top, height, 0.0))
    points = np.column_stack([ring * r[:, None], z + bottom])
    normals = np.where(side[:, None], np.column_stack([ring, np.zeros(count)]),
                       np.column_stack([np.zeros((count, 2)), np.where(top, 1.0, -1.0)]))
    return state(points, normals)


def sphere(radius, count=1024, seed=0, bottom=SUPPORT) -> ObjectState:
    rng = np.random.default_rng(seed)
    normals = rng.normal(size=(count, 3))
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
    return state(normals * radius + [0.0, 0.0, bottom + radius], normals)


def lowest_finger_point(candidate) -> float:
    """Height of the lowest corner of the two fingers, before they close."""
    position, turn = candidate.gripper_pose()
    corners = [[x, side * y, z]
               for x in (GRIPPER.pad_back, GRIPPER.pad_tip)
               for side in (-1, 1) for y in (candidate.opening / 2, candidate.opening / 2 + GRIPPER.pad_thickness)
               for z in (-GRIPPER.pad_width / 2, GRIPPER.pad_width / 2)]
    return float((position + np.array(corners) @ turn.T)[:, 2].min())


class AntipodalCandidates(unittest.TestCase):
    def test_box_gives_pairs_across_opposite_faces(self):
        """R7: every candidate closes along a box axis, across that dimension."""
        size = np.array([0.04, 0.06, 0.08])
        found = antipodal_candidates(box(size), GRIPPER, SUPPORT)
        self.assertGreater(len(found), 10)
        for c in found:
            axis = int(np.argmax(np.abs(c.closing_axis)))
            self.assertGreater(abs(c.closing_axis[axis]), np.cos(np.radians(15.0)) - 1e-9)
            self.assertLess(abs(c.width - size[axis]), 0.02)

    def test_cylinder_gives_pairs_across_the_diameter(self):
        """R7: on the curved side, closing lines pass near the cylinder's axis."""
        found = antipodal_candidates(cylinder(0.03, 0.10), GRIPPER, SUPPORT)
        sideways = [c for c in found if abs(c.closing_axis[2]) < 0.3]
        self.assertGreater(len(sideways), 10)
        for c in sideways:
            self.assertLess(np.linalg.norm(c.centre[:2]), 0.012)
            self.assertLess(abs(c.width - 0.06), 0.008)

    def test_sphere_gives_pairs_through_the_centre(self):
        """R7"""
        obj = sphere(0.035)
        found = antipodal_candidates(obj, GRIPPER, SUPPORT)
        self.assertGreater(len(found), 10)
        for c in found:
            self.assertLess(np.linalg.norm(c.centre - [0.0, 0.0, SUPPORT + 0.035]), 0.012)

    def test_nothing_wider_than_the_open_gripper(self):
        """R7: a 120 mm cube gives no candidate."""
        self.assertEqual(antipodal_candidates(box([0.12, 0.12, 0.12]), GRIPPER, SUPPORT), [])

    def test_width_leaves_room_for_the_fingers(self):
        settings = CandidateSettings()
        found = antipodal_candidates(box([0.04, 0.09, 0.08]), GRIPPER, SUPPORT)
        self.assertEqual({c.kind for c in found}, {"antipodal"})
        for c in found:
            self.assertLessEqual(c.width, GRIPPER.max_opening - 2 * settings.clearances[-1] + 1e-9)
            self.assertGreaterEqual(c.opening, c.width + 2 * settings.clearances[-1] - 1e-9)
            self.assertLessEqual(c.opening, GRIPPER.max_opening + 1e-12)

    def test_object_as_wide_as_the_open_hand_gets_tight_candidates(self):
        """R13: a 102.5 mm cylinder, 140 mm tall, fits the 102.85 mm opening
        with no room to spare: the fingers open fully and go round it, from
        above or from the side, never from below."""
        found = antipodal_candidates(cylinder(0.05125, 0.14, count=2048), GRIPPER, SUPPORT)
        self.assertGreater(len(found), 5)
        for c in found:
            self.assertEqual(c.kind, "tight")
            self.assertEqual(c.opening, GRIPPER.max_opening)
            self.assertLess(c.approach_axis[2], 0.1)
        self.assertTrue(any(c.approach_axis[2] < -0.9 for c in found))

    def test_closing_line_through_the_centre_of_mass_ranks_first(self):
        """The object turns freely about the line between the pads, so the
        best grasps on a long box pass through its centre."""
        obj = box([0.04, 0.20, 0.04], count=2048)
        found = antipodal_candidates(obj, GRIPPER, SUPPORT)
        from_above = [c for c in found if c.approach_axis[2] < -0.95]
        self.assertLess(abs(from_above[0].centre[1] - obj.position[1]), 0.03)
        self.assertGreater(max(abs(c.centre[1] - obj.position[1]) for c in from_above), 0.03)

    def test_tight_candidates_rank_below_roomy_ones(self):
        found = antipodal_candidates(box([0.06, 0.10, 0.08], count=2048), GRIPPER, SUPPORT, limit=2000)
        kinds = [c.kind for c in found]
        self.assertIn("tight", kinds)
        self.assertIn("antipodal", kinds)
        self.assertLess(kinds.index("antipodal"), kinds.index("tight"))

    def test_same_cloud_gives_the_same_candidates(self):
        """R24"""
        a = antipodal_candidates(box([0.04, 0.06, 0.08]), GRIPPER, SUPPORT)
        b = antipodal_candidates(box([0.04, 0.06, 0.08]), GRIPPER, SUPPORT)
        self.assertEqual(len(a), len(b))
        for x, y in zip(a, b):
            np.testing.assert_array_equal(x.centre, y.centre)
            np.testing.assert_array_equal(x.approach_axis, y.approach_axis)
            self.assertEqual(x.score, y.score)

    def test_candidates_are_ordered_best_first(self):
        scores = [c.score for c in antipodal_candidates(sphere(0.035), GRIPPER, SUPPORT)]
        self.assertEqual(scores, sorted(scores, reverse=True))

    def test_axes_are_unit_and_at_right_angles(self):
        for c in antipodal_candidates(cylinder(0.03, 0.10), GRIPPER, SUPPORT):
            self.assertAlmostEqual(np.linalg.norm(c.closing_axis), 1.0, places=9)
            self.assertAlmostEqual(np.linalg.norm(c.approach_axis), 1.0, places=9)
            self.assertAlmostEqual(float(np.dot(c.closing_axis, c.approach_axis)), 0.0, places=9)
            _, turn = c.gripper_pose()
            np.testing.assert_allclose(turn.T @ turn, np.eye(3), atol=1e-9)
            self.assertAlmostEqual(np.linalg.norm(c.gripper_pose()[0] + c.approach_axis * c.offset - c.centre), 0.0)

    def test_thin_wall_is_pinched_across_its_thickness(self):
        """R13: an 8 mm wall standing on the support (a rim) gets candidates."""
        found = antipodal_candidates(box([0.008, 0.12, 0.06], count=2048), GRIPPER, SUPPORT)
        thin = [c for c in found if abs(c.width - 0.008) < 0.003]
        self.assertGreater(len(thin), 5)

    def test_shell_one_millimetre_thick_is_pinched_from_its_free_edge(self):
        """R13: a sparse cloud of a 1 mm wall has no two points opposite each
        other, yet the wall gets rim candidates, approached from its top edge."""
        found = antipodal_candidates(box([0.001, 0.12, 0.06], count=600), GRIPPER, SUPPORT)
        rims = [c for c in found if c.kind == "rim"]
        self.assertGreater(len(rims), 5)
        for c in rims:
            self.assertGreater(abs(c.closing_axis[0]), 0.99)
            self.assertLess(c.width, 0.002)
        self.assertLess(rims[0].approach_axis[2], -0.8)

    def test_solid_objects_get_no_rim_candidates(self):
        kinds = {c.kind for c in antipodal_candidates(box([0.04, 0.06, 0.08]), GRIPPER, SUPPORT)}
        self.assertEqual(kinds, {"antipodal"})


class CheapRejection(unittest.TestCase):
    def test_palm_stays_outside_a_tall_object(self):
        """From above, the contacts on a 200 mm tall box are near its top:
        lower down the palm would be inside the box."""
        obj = box([0.06, 0.06, 0.20], count=2048)
        top = SUPPORT + 0.20
        reach = GRIPPER.pad_tip - GRIPPER.palm_front
        from_above = [c for c in antipodal_candidates(obj, GRIPPER, SUPPORT) if c.approach_axis[2] < -0.95]
        self.assertGreater(len(from_above), 3)
        for c in from_above:
            self.assertLess(top - c.centre[2], reach)

    def test_fingers_stay_above_the_support(self):
        """A 20 mm tall slab: every candidate keeps the fingers off the table."""
        found = antipodal_candidates(box([0.05, 0.08, 0.02]), GRIPPER, SUPPORT)
        self.assertGreater(len(found), 3)
        for c in found:
            self.assertGreaterEqual(lowest_finger_point(c), SUPPORT + CandidateSettings().tip_margin - 1e-9)

    def test_low_object_is_held_near_the_fingertips(self):
        """The fingers cannot go deeper than the table allows."""
        found = antipodal_candidates(box([0.05, 0.08, 0.02]), GRIPPER, SUPPORT)
        from_above = [c for c in found if c.approach_axis[2] < -0.95]
        self.assertTrue(from_above)
        for c in from_above:
            self.assertGreater(c.offset, 0.008)

    def test_pairs_respect_the_cone(self):
        obj = sphere(0.035)
        settings = CandidateSettings()
        i, j, alignment = antipodal_pairs(obj, 0.09, settings)
        self.assertTrue(np.all(alignment >= np.cos(settings.cone)))
        line = obj.cloud[j] - obj.cloud[i]
        line /= np.linalg.norm(line, axis=1, keepdims=True)
        self.assertTrue(np.all(np.einsum("ij,ij->i", obj.normals[j], line) >= np.cos(settings.cone) - 1e-12))


if __name__ == "__main__":
    unittest.main()
