"""Geometry tests for custom_libraries/yaw_carry.py; no PyBullet required.

Run: python -m unittest tests/test_yaw_carry.py -v
"""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from custom_libraries.yaw_carry import (ClearanceField, CarryTrajectory, fillet, route_cells,
                                        shortcut, spin_clear, outline)


def corridor_l():
    """10 m x 10 m map, walls everywhere except an L-shaped 1 m corridor:
    along y in [4.5, 5.5] from x = 0.5 to 5.5, then up x in [4.5, 5.5]."""
    occ = np.ones((100, 100), np.uint8)       # row 0 is y = 10 (y_max), col 0 is x = 0
    occ[45:55, 5:55] = 0                      # y 4.5..5.5, x 0.5..5.5
    occ[5:55, 45:55] = 0                      # x 4.5..5.5, y 4.5..9.5
    return occ


class FieldTests(unittest.TestCase):
    def test_lower_bound(self):
        field = ClearanceField(corridor_l(), 0.0, 10.0, 0.1)
        # Corridor centre line: true clearance 0.5, estimate a little below.
        c = field(np.array([[2.0, 5.0]]))[0]
        self.assertLessEqual(c, 0.5)
        self.assertGreater(c, 0.45)
        self.assertEqual(field(np.array([[2.0, 7.0]]))[0], 0.0)    # inside a wall
        self.assertEqual(field(np.array([[-1.0, 5.0]]))[0], 0.0)   # off the map


class RouteTests(unittest.TestCase):
    def setUp(self):
        self.occ = corridor_l()
        self.field = ClearanceField(self.occ, 0.0, 10.0, 0.1)

    def test_route_stays_centred(self):
        route = route_cells(self.field, (1.0, 5.0), (5.0, 9.0), 0.1, 0.0, 10.0, self.occ.shape, 0.34)
        self.assertIsNotNone(route)
        straight = route[(route[:, 0] > 1.5) & (route[:, 0] < 4.0)]
        self.assertTrue(np.all(np.abs(straight[:, 1] - 5.0) < 0.11))

    def test_fillet_is_smooth_and_clear(self):
        route = route_cells(self.field, (1.0, 5.0), (5.0, 9.0), 0.1, 0.0, 10.0, self.occ.shape, 0.34)
        path = fillet(shortcut(route, self.field, 0.40), self.field, 0.40, 0.5)
        steps = np.linalg.norm(np.diff(path, axis=0), axis=1)
        self.assertLess(steps.max(), 0.0101)
        self.assertTrue(np.all(self.field(path) >= 0.40))
        heading = np.unwrap(np.arctan2(*np.diff(path, axis=0).T[::-1]))
        self.assertLess(np.abs(np.diff(heading)).max(), 0.05)       # no sharp corner
        self.assertAlmostEqual(heading[-1] - heading[0], np.pi / 2, places=2)


class TrajectoryTests(unittest.TestCase):
    def test_object_on_path_base_trails(self):
        occ = corridor_l()
        field = ClearanceField(occ, 0.0, 10.0, 0.1)
        route = route_cells(field, (1.0, 5.0), (5.0, 9.0), 0.1, 0.0, 10.0, occ.shape, 0.34)
        path = fillet(shortcut(route, field, 0.40), field, 0.40, 0.5)
        tr = CarryTrajectory(path, (0.7, 0.0))
        chord = np.linalg.norm(tr.obj - tr.base, axis=1)
        self.assertLess(np.abs(chord - 0.7).max(), 0.005)
        self.assertTrue(np.all(np.diff(tr.sb) >= 0))
        # Yaw points from base to object.
        heading = np.arctan2(*(tr.obj - tr.base).T[::-1])
        self.assertLess(np.abs(np.angle(np.exp(1j * (heading - tr.yaw)))).max(), 1e-6)
        # A 0.7 m arm (a thin line plus a 0.29 m base circle) fits the corner.
        ang = np.linspace(0, 2 * np.pi, 60, endpoint=False)
        fp = np.vstack([0.29 * np.c_[np.cos(ang), np.sin(ang)], np.c_[np.linspace(0, 0.75, 40), np.zeros(40)]])
        self.assertEqual(tr.collisions(field, fp, 0.04), [])
        # ...but cannot turn on the spot at the corner.
        self.assertFalse(spin_clear(field, fp, np.array([5.0, 5.0]), 0.0, np.pi / 2, 0.04))
        v = tr.speed_profile(0.03, 0.036, 0.05, 0.06, 0.002)
        self.assertEqual(v[-1], 0.0)
        self.assertLessEqual(v.max(), 0.03)

    def test_outline_spacing(self):
        pts = outline([np.array([[0, 0], [1, 0], [1, 1], [0, 1]], float)], 0.01)
        self.assertEqual(len(pts), 400)


if __name__ == "__main__":
    unittest.main()
