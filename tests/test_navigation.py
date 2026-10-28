"""Run from your project root: python -m unittest discover -s navigation_tests -v

Dependencies: numpy. Tests import your production modules (not copies).
Optional: NAV_SOLVER_FILE selects the solver source for action-contract tests.
"""
import ast
import contextlib
import heapq
import importlib
import io
import math
import os
from pathlib import Path
import unittest
import numpy as np


def module(primary, fallback):
    try:
        return importlib.import_module(primary)
    except ModuleNotFoundError as exc:
        if exc.name not in {primary, primary.split('.')[0]}:
            raise
        return importlib.import_module(fallback)


astar_module = module('custom_libraries.a_star', 'a_star')
curve_module = module('custom_libraries.curved_navigator', 'curved_navigation')
a_star = astar_module.a_star
CurvedNavigator = curve_module.CurvedNavigator


def cost(path):
    return sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a,b in zip(path,path[1:]))


def reference_cost(grid, start, goal):
    # Independent Dijkstra oracle, deliberately not using A* helpers.
    if grid[start] or grid[goal]:
        return math.inf
    queue = [(0., start)]
    distances = {start: 0.}
    while queue:
        dist, (r,c) = heapq.heappop(queue)
        if dist != distances[(r,c)]:
            continue
        if (r,c) == goal:
            return dist
        for nr in range(r-1, r+2):
            for nc in range(c-1, c+2):
                if (nr,nc)==(r,c) or not (0<=nr<grid.shape[0] and 0<=nc<grid.shape[1]):
                    continue
                if grid[nr,nc] or (nr!=r and nc!=c and (grid[nr,c] or grid[r,nc])):
                    continue
                nd = dist + math.hypot(nr-r,nc-c)
                if nd < distances.get((nr,nc),math.inf):
                    distances[(nr,nc)] = nd
                    heapq.heappush(queue,(nd,(nr,nc)))
    return math.inf


class GridMap:
    cell_size = .1
    x_min = 0.
    y_max = 5.
    def __init__(self):
        self.inflated_occupancy = np.zeros((50,50),dtype=np.uint8)
    def convert_to_world(self, cell):
        r,c = cell
        return np.array([(c+.5)*self.cell_size,self.y_max-(r+.5)*self.cell_size])


class AStarTests(unittest.TestCase):
    def test_straight(self):
        grid=np.zeros((10,10))
        self.assertAlmostEqual(cost(a_star((1,1),(1,8),grid)),7)
    def test_diagonal_cost(self):
        grid=np.zeros((10,10))
        path=a_star((1,1),(8,8),grid)
        self.assertEqual(len(path),8)
        self.assertAlmostEqual(cost(path),7*math.sqrt(2))
    def test_corner_cutting_rejected(self):
        grid=np.zeros((3,3)); grid[0,1]=grid[1,0]=1
        self.assertEqual(a_star((0,0),(1,1),grid),[])
    def test_one_blocked_side_prevents_diagonal(self):
        grid=np.zeros((3,3)); grid[0,1]=1
        self.assertAlmostEqual(cost(a_star((0,0),(1,1),grid)),2)
    def test_unreachable(self):
        grid=np.zeros((5,5)); grid[2,:]=1
        self.assertEqual(a_star((0,0),(4,4),grid),[])
    def test_blocked_endpoints_and_bounds(self):
        grid=np.zeros((5,5)); grid[1,1]=1
        with contextlib.redirect_stdout(io.StringIO()):
            for start,goal in [((1,1),(4,4)),((0,0),(1,1)),((-1,0),(4,4)),((0,0),(5,4))]:
                with self.subTest(start=start,goal=goal):
                    self.assertEqual(a_star(start,goal,grid),[])
    def test_same_cell(self):
        self.assertEqual(a_star((1,1),(1,1),np.zeros((3,3))),[(1,1)])
    def test_does_not_mutate_grid(self):
        grid=np.zeros((8,8)); grid[3:6,4]=1; before=grid.copy()
        a_star((1,1),(6,6),grid)
        np.testing.assert_array_equal(grid,before)
    def test_matches_dijkstra_on_40_seeded_maps(self):
        for seed in range(40):
            with self.subTest(seed=seed):
                grid=(np.random.default_rng(seed).random((14,14))<.25).astype(int)
                start,goal=(0,0),(13,13); grid[start]=grid[goal]=0
                path=a_star(start,goal,grid)
                expected=reference_cost(grid,start,goal)
                if math.isinf(expected):
                    self.assertEqual(path,[])
                    continue
                self.assertEqual(path[0],start); self.assertEqual(path[-1],goal)
                self.assertAlmostEqual(cost(path),expected,places=9)
                for a,b in zip(path,path[1:]):
                    self.assertFalse(grid[b])
                    dr,dc=b[0]-a[0],b[1]-a[1]
                    self.assertLessEqual(max(abs(dr),abs(dc)),1)
                    if dr and dc:
                        self.assertFalse(grid[a[0]+dr,a[1]])
                        self.assertFalse(grid[a[0],a[1]+dc])


class CurveTests(unittest.TestCase):
    def setUp(self):
        self.map=GridMap(); self.nav=CurvedNavigator(self.map)
        self.path=[(35,c) for c in range(10,26)]+[(r,25) for r in range(34,19,-1)]
        self.start=self.map.convert_to_world(self.path[0])
    def assert_free(self):
        self.assertGreater(len(self.nav.points),0)
        for a,b in zip(self.nav.points,self.nav.points[1:]):
            self.assertTrue(self.nav.segment_free(a,b))
    def test_open_corner_rounded(self):
        self.nav.build(self.start,self.path)
        self.assertEqual(self.nav.rounded_corners,1)
        self.assertGreater(len(self.nav.points),3)
        np.testing.assert_allclose(self.nav.points[0],self.start)
        np.testing.assert_allclose(self.nav.points[-1],self.map.convert_to_world(self.path[-1]))
        self.assert_free()
    def test_obstructed_corner_falls_back_or_shrinks(self):
        self.map.inflated_occupancy[34,24]=1
        self.nav.build(self.start,self.path)
        self.assert_free()
    def test_dense_analytic_bezier_samples_are_free(self):
        a,b,c=np.array([1.,1.]),np.array([2.,1.]),np.array([2.,2.])
        self.assertIsNotNone(self.nav._curve(a,b,c))
        for t in np.linspace(0,1,501):
            point=(1-t)**2*a+2*t*(1-t)*b+t*t*c
            self.assertTrue(self.nav.segment_free(point,point))
        self.map.inflated_occupancy[37,17]=1
        self.assertIsNone(self.nav._curve(a,b,c))
    def test_corner_touch_and_tiny_crossing_rejected(self):
        self.map.inflated_occupancy[34,24]=1
        self.assertFalse(self.nav.segment_free([2.4,1.5],[2.5,1.6]))
        self.assertFalse(self.nav.segment_free([2.39999,1.55],[2.40001,1.55]))
    def test_outside_map_rejected(self):
        self.assertFalse(self.nav.segment_free([-.1,1],[1,1]))
    def test_invalid_diagonal_rejected(self):
        self.map.inflated_occupancy[34,24]=1
        self.nav.build(self.map.convert_to_world((35,24)),[(35,24),(34,25)])
        self.assertEqual(self.nav.status,'invalid_grid_path_or_start')
    def test_blocked_start_connection_rejected(self):
        self.map.inflated_occupancy[35,10]=1
        self.nav.build(self.start,self.path)
        self.assertEqual(self.nav.status,'invalid_grid_path_or_start')
    def test_empty_path(self):
        path=[]
        np.testing.assert_array_equal(self.nav.step(self.start,path),[0,0])
        self.assertEqual(self.nav.status,'no_path')
    def test_already_at_goal(self):
        path=[self.path[0]]
        np.testing.assert_array_equal(self.nav.step(self.start,path),[0,0])
        self.assertEqual(self.nav.status,'reached')
    def test_no_measured_motion_does_not_advance_progress(self):
        self.nav.step(self.start,self.path)
        progress=self.nav.progress
        for _ in range(20): self.nav.step(self.start,self.path)
        self.assertEqual(self.nav.progress,progress)
    def test_position_feedback_tracking(self):
        pos=self.start.copy(); previous=0.
        for _ in range(2000):
            d=self.nav.step(pos,self.path)
            self.assertLessEqual(np.linalg.norm(d),self.nav.max_lookahead+.021)
            self.assertTrue(self.nav.segment_free(pos,pos+d))
            self.assertGreaterEqual(self.nav.progress,previous)
            previous=self.nav.progress; pos+=.35*d
            if self.nav.status=='reached': break
        self.assertEqual(self.nav.status,'reached')
    def test_replan_rebuilds_curve(self):
        self.nav.step(self.start,self.path)
        new_path=[(35,c) for c in range(10,16)]
        self.nav.step(self.start,new_path)
        np.testing.assert_allclose(self.nav.points[-1],self.map.convert_to_world(new_path[-1]))
    def test_lateral_error_can_be_corrected_in_open_space(self):
        # Position feedback may choose a short safe chord back toward the path.
        path=[(35,c) for c in range(10,26)]
        self.nav.step(self.start,path)
        d=self.nav.step(self.start+[0,.1],path)
        self.assertLess(d[1],0)
        self.assertTrue(self.nav.segment_free(self.start+[0,.1],self.start+[0,.1]+d))
        self.assertEqual(self.nav.status,'following')
    def test_astar_to_curve_integration(self):
        self.map.inflated_occupancy[20:40,20]=1
        path=a_star((30,10),(30,30),self.map.inflated_occupancy)
        self.assertTrue(path)
        pos=self.map.convert_to_world(path[0])
        for _ in range(2000):
            pos+=.35*self.nav.step(pos,path)
            if self.nav.status=='reached': break
        self.assertEqual(self.nav.status,'reached')


class SolverActionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        filename=Path(os.environ.get('NAV_SOLVER_FILE','solver.py'))
        if not filename.exists():
            raise unittest.SkipTest('solver.py not found; set NAV_SOLVER_FILE to test get_action')
        # Execute the actual action method only, avoiding GUI/asset initialization.
        tree=ast.parse(filename.read_text(encoding='utf-8-sig'))
        solver=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Solver')
        method=next(n for n in solver.body if isinstance(n,ast.FunctionDef) and n.name=='get_action')
        globals_={'np':np, 'a_star':a_star}
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(filename),'exec'),globals_)
        cls.method=staticmethod(globals_['get_action'])
    def test_action_shape_limits_and_nonbase_preservation(self):
        from types import SimpleNamespace
        qpos=np.zeros(15); initial=np.arange(15)*.01
        env=SimpleNamespace(_get_obs=lambda:{'qpos':qpos.copy(),'goal_pos':np.array([2.,2.,.9])})
        nav=SimpleNamespace(step=lambda base,path:np.array([.2,.1]),grid_index=1,status='following')
        solver=SimpleNamespace(action=initial.copy(),path=[(1,1),(1,2)],curved_nav=nav,
            convert_to_grid=lambda x:(1,1),action_scale=np.full(15,.01),
            action_lower_bound=np.full(15,-2.),action_upper_bound=np.full(15,2.))
        for key in ['torso_lift','head_pan','head_tilt','shoulder_pan','shoulder_lift',
                    'upperarm_roll','elbow_flex','forearm_roll','wrist_flex','wrist_roll','gripper_open']:
            setattr(solver,key,0)
        with contextlib.redirect_stdout(io.StringIO()): result=self.method(solver,env)
        self.assertEqual(result.shape,(15,))
        np.testing.assert_allclose(result[:2],[.2,.1])
        np.testing.assert_allclose(result[2:],initial[2:])
        self.assertFalse(np.shares_memory(result,solver.action))
    def test_astar_binding_is_callable(self):
        # Check the real solver import in a separate test, using a harmless
        # stand-in only if PyBullet is unavailable. Never initialize a world.
        import importlib.util
        import sys
        import types
        from unittest.mock import patch
        filename=Path(os.environ.get('NAV_SOLVER_FILE','solver.py'))
        spec=importlib.util.spec_from_file_location('_solver_under_test',filename)
        loaded=importlib.util.module_from_spec(spec)
        try:
            import pybullet
            replacements={}
        except ImportError:
            replacements={'pybullet':types.ModuleType('pybullet')}
        with patch.dict(sys.modules,replacements): spec.loader.exec_module(loaded)
        self.assertTrue(callable(loaded.a_star),
            'Import the function: from custom_libraries.a_star import a_star')


if __name__=='__main__': unittest.main()
