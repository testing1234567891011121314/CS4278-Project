"""Tests for GraspPlanner.candidate_features and the exploration order on a
synthetic box cloud; no PyBullet required.

Run: python -m unittest tests/test_grasp_features.py -v
"""
from pathlib import Path
import sys
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from custom_libraries.grasp import GraspPlanner


def box_cloud(size=(0.06, 0.10, 0.08), base_z=0.60, step=0.004):
    """Points and outward normals on the surface of an axis-aligned box whose
    bottom face rests at base_z, centred on x = y = 0."""
    half = np.asarray(size) / 2
    centre = np.array([0., 0., base_z + half[2]])
    pts, ns = [], []
    for axis in range(3):
        u, v = [a for a in range(3) if a != axis]
        # Offset by half a step so that no point lies on an edge twice.
        gu = np.arange(-half[u] + step / 2, half[u], step)
        gv = np.arange(-half[v] + step / 2, half[v], step)
        uu, vv = np.meshgrid(gu, gv)
        for sign in (1., -1.):
            p = np.zeros((uu.size, 3))
            p[:, u], p[:, v], p[:, axis] = uu.ravel(), vv.ravel(), sign * half[axis]
            n = np.zeros_like(p)
            n[:, axis] = sign
            pts.append(p + centre)
            ns.append(n)
    return np.vstack(pts), np.vstack(ns), centre


def top_down_candidate(centre, half_x, score=1.0):
    """Pinch across x (closing axis +y of the gripper = world +x), approach
    straight down, as the sampler builds it for roll 0."""
    p1, p2 = centre + [-half_x, 0., 0.], centre + [half_x, 0., 0.]
    approach, closing = np.array([0., 0., -1.]), np.array([1., 0., 0.])
    rot = np.column_stack((approach, closing, np.cross(approach, closing)))
    return (p1, p2, centre.copy(), Rotation.from_matrix(rot).as_quat(), score)


class FeatureTests(unittest.TestCase):
    def setUp(self):
        self.planner = GraspPlanner()
        self.pts, self.ns, self.com = box_cloud()

    def test_top_down_pinch(self):
        # 10 mm below the centre of mass, so the pads stay clear of the top face.
        centre = self.com - [0., 0., 0.01]
        f = self.planner.candidate_features(top_down_candidate(centre, 0.03), self.pts, self.ns, self.com)
        self.assertAlmostEqual(f['tilt_deg'], 0.0, places=4)
        self.assertAlmostEqual(f['closing_deg'], 0.0, places=4)
        self.assertAlmostEqual(f['roll_deg'], 0.0, places=4)
        self.assertAlmostEqual(f['width_mm'], 60.0, places=6)
        self.assertAlmostEqual(f['margin_mm'], (0.00285 + 0.1 - 0.06) * 1000, places=6)
        self.assertAlmostEqual(f['depth_mm'], 10.0)
        self.assertAlmostEqual(f['centre_h_mm'], 30.0, places=6)
        self.assertAlmostEqual(f['centre_h_frac'], 0.375, places=6)
        self.assertAlmostEqual(f['com_dist_mm'], 10.0, places=6)
        self.assertAlmostEqual(f['com_dz_mm'], 10.0, places=6)
        self.assertAlmostEqual(f['com_dxy_mm'], 0.0, places=6)
        self.assertAlmostEqual(f['torque_arm_mm'], 0.0, places=6)
        self.assertAlmostEqual(f['quality'], 1.0, places=6)
        self.assertEqual(f['score'], 1.0)
        # Both pads lie on the flat 100 x 80 mm faces: well covered, no normal spread.
        self.assertGreater(f['pad_cov_min'], 0.9)
        self.assertAlmostEqual(f['pad_spread_max_deg'], 0.0, places=4)
        # Finger tips 31.5 mm below the pad centre, which is 10 mm above the contact centre.
        self.assertAlmostEqual(f['finger_low_mm'], 30.0 + 10.0 - 31.5, places=6)
        # The top face, 40 mm above the pad centre, is inside the palm box
        # (which starts 32 mm behind it) plus its 6 mm margin.
        self.assertEqual((f['flag_tight'], f['flag_palm'], f['flag_support']), (0., 1., 0.))
        np.testing.assert_allclose([f['ext1_mm'], f['ext2_mm'], f['ext3_mm']], [100., 80., 60.], atol=1e-6)
        self.assertAlmostEqual(f['height_mm'], 80.0, places=6)
        self.assertTrue(all(isinstance(v, float) for v in f.values()))

    def test_off_centre_and_low(self):
        # Grasp 25 mm above and 20 mm along y from the centre of mass, near
        # the bottom: the com offset is all vertical + horizontal, the torque arm
        # (rotation about the horizontal closing axis x) is the y offset, and
        # the fingertips reach below the table.
        centre = self.com + [0., 0.02, -0.025]
        f = self.planner.candidate_features(top_down_candidate(centre, 0.03), self.pts, self.ns, self.com)
        self.assertAlmostEqual(f['com_dist_mm'], np.hypot(20., 25.), places=6)
        self.assertAlmostEqual(f['com_dz_mm'], 25.0, places=6)
        self.assertAlmostEqual(f['com_dxy_mm'], 20.0, places=6)
        self.assertAlmostEqual(f['torque_arm_mm'], 20.0, places=6)
        self.assertLess(f['finger_low_mm'], 0.0)
        self.assertEqual(f['flag_support'], 1.0)

    def test_edge_patch_has_spread(self):
        # Pads centred on the box's top edges: the patch holds points of the
        # side face and the top face, so normals spread.
        centre = self.com + [0., 0., 0.04]
        f = self.planner.candidate_features(top_down_candidate(centre, 0.03), self.pts, self.ns, self.com)
        self.assertGreater(f['pad_spread_min_deg'], 10.0)

    def test_sampler_candidates_and_roll(self):
        cands = self.planner.sample_antipodal_grasps(points=self.pts, normals=self.ns, com=self.com)
        self.assertTrue(cands)
        rolls = set()
        for c in cands[:200]:
            f = self.planner.candidate_features(c, self.pts, self.ns, self.com)
            self.assertAlmostEqual(f['score'], c[4])
            self.assertGreaterEqual(f['quality'], 1 / np.sqrt(1.25) - 1e-9)
            rolls.add(round(f['roll_deg']))
        self.assertTrue(rolls <= {0, 30, -30, 90, -90}, rolls)

    def test_explore_order(self):
        cands = self.planner.sample_antipodal_grasps(points=self.pts, normals=self.ns, com=self.com)
        k = 40
        a = self.planner._explore_order(cands, k, np.random.default_rng(3))
        b = self.planner._explore_order(cands, k, np.random.default_rng(3))
        self.assertEqual([id(c) for c in a], [id(c) for c in b])          # seeded
        self.assertEqual({id(c) for c in a[:k]}, {id(c) for c in cands[:k]})
        self.assertEqual([id(c) for c in a[k:]], [id(c) for c in cands[k:]])
        # Round robin over families: no family repeats before every family has had a turn.
        keys = [self.planner._family_key(c) for c in a[:k]]
        n_fam = len(set(keys))
        self.assertEqual(len(set(keys[:n_fam])), n_fam)


if __name__ == '__main__':
    unittest.main()
