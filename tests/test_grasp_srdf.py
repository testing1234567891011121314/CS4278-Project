"""SRDF and start-state regression tests; no PyBullet installation required.

Place this file at project-root/tests/test_grasp_srdf.py and run:
    python -m pytest tests/test_grasp_srdf.py -v

The planner is loaded from custom_libraries/grasp.py; robot descriptions are
read directly from assets/fetch/fetch.urdf and assets/fetch/fetch.srdf.

These use real URDF/SRDF metadata and a small fake Bullet API. Collision contacts
are injected: these tests verify planner policy/control flow, not mesh geometry,
IK performance, or physical grasp success.
"""
from __future__ import annotations

import types

import copy
import importlib.util
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import warnings
import xml.etree.ElementTree as ET

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'grasp_under_test', PROJECT_ROOT / 'custom_libraries' / 'grasp.py')
grasp = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = grasp
SPEC.loader.exec_module(grasp)


def contact(a, b, distance=-0.001):
    row = [0] * 10
    row[3], row[4], row[8] = a, b, distance
    return tuple(row)


class FakeBullet:
    JOINT_REVOLUTE, JOINT_PRISMATIC, JOINT_FIXED = 0, 1, 4

    def __init__(self, urdf):
        root = ET.fromstring(urdf)
        joints = root.findall('joint')
        self.link_ids = {j.find('child').get('link'): i for i, j in enumerate(joints)}
        roots = {e.get('name') for e in root.findall('link')} - self.link_ids.keys()
        assert len(roots) == 1
        self.root_link = next(iter(roots))
        self.link_ids[self.root_link] = -1
        self.info = []
        self.q = {}
        self.vel = {}
        for i, joint in enumerate(joints):
            kind = {'fixed': self.JOINT_FIXED, 'prismatic': self.JOINT_PRISMATIC,
                    'revolute': self.JOINT_REVOLUTE}[joint.get('type')]
            limit = joint.find('limit')
            row = [0] * 17
            row[0], row[1], row[2] = i, joint.get('name').encode(), kind
            row[8] = float(limit.get('lower', '0')) if limit is not None else 0.
            row[9] = float(limit.get('upper', '0')) if limit is not None else 0.
            row[12] = joint.find('child').get('link').encode()
            row[16] = self.link_ids[joint.find('parent').get('link')]
            self.info.append(tuple(row))
            self.q[i] = 0.02 if joint.get('name') in grasp.FINGER_NAMES else 0.
            self.vel[i] = 0.01 + i * 0.001
        self.self_contacts = []
        self.obstacle_contacts = []
        self.saved = {}
        self.restores = 0
        self.fail_query = False

    def getNumJoints(self, robot, **kwargs):
        return len(self.info)

    def getJointInfo(self, robot, joint, **kwargs):
        return self.info[joint]

    def getBodyInfo(self, robot, **kwargs):
        return self.root_link.encode(), b'fetch'

    def getNumBodies(self, **kwargs):
        return 2

    def getBodyUniqueId(self, index, **kwargs):
        return index

    def getJointStates(self, robot, ids, **kwargs):
        return [(self.q[j], self.vel[j], (), 0.) for j in ids]

    def resetJointState(self, robot, joint, value, **kwargs):
        self.q[joint] = value
        self.vel[joint] = 0.

    def getClosestPoints(self, body_a, body_b, distance, **kwargs):
        if self.fail_query:
            raise RuntimeError('injected collision-query failure')
        rows = self.self_contacts if body_a == body_b else self.obstacle_contacts
        # The planner queries self-collision pair by pair (linkIndexA/B).
        pair = {kwargs.get('linkIndexA'), kwargs.get('linkIndexB')}
        if body_a == body_b and None not in pair:
            rows = [row for row in rows if {row[3], row[4]} == pair]
        return [row for row in rows if row[8] <= distance]

    def getAABB(self, body, link=-1, **kwargs):
        # Every box overlaps every other, so no pair is skipped by the box test.
        return (0., 0., 0.), (1., 1., 1.)

    def saveState(self, **kwargs):
        key = len(self.saved) + 1
        self.saved[key] = copy.deepcopy((self.q, self.vel))
        return key

    def restoreState(self, stateId, **kwargs):
        self.q, self.vel = copy.deepcopy(self.saved[stateId])
        self.restores += 1

    def removeState(self, stateUniqueId, **kwargs):
        del self.saved[stateUniqueId]


class SRDFTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        model_dir = PROJECT_ROOT / 'assets' / 'fetch'
        cls.urdf_text = (model_dir / 'fetch.urdf').read_text(encoding='utf-8')
        cls.srdf_text = (model_dir / 'fetch.srdf').read_text(encoding='utf-8')

    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.urdf = Path(self.folder.name) / 'fetch.urdf'
        self.srdf = self.urdf.with_suffix('.srdf')
        self.urdf.write_text(self.urdf_text)
        self.srdf.write_text(self.srdf_text)
        self.bullet = FakeBullet(self.urdf_text)
        p_patch = patch.object(grasp, 'p', self.bullet)
        p_patch.start()
        self.addCleanup(p_patch.stop)
        ids = [i for i, row in enumerate(self.bullet.info)
               if row[2] != self.bullet.JOINT_FIXED]
        self.env = SimpleNamespace(
            robot_id=0, object_id=1, pb_physics_client=0, joint_indices=ids,
            joint_lower=[self.bullet.info[j][8] for j in ids],
            joint_upper=[self.bullet.info[j][9] for j in ids])

    def planner(self, **kwargs):
        return grasp.GraspPlanner(self.env, robot_urdf_path=self.urdf, **kwargs)

    def obs(self):
        return {'qpos': np.array([self.bullet.q[j] for j in self.env.joint_indices]),
                'object_pc': np.empty((0, 3)), 'object_normals': np.empty((0, 3))}

    def write_srdf(self, entries, name='fetch'):
        self.srdf.write_text(f'<robot name="{name}">{entries}</robot>')

    def assert_restored(self, before):
        self.assertEqual((self.bullet.q, self.bullet.vel), before)
        self.assertEqual(self.bullet.saved, {})
        self.assertEqual(self.bullet.restores, 1)

    def test_all_ten_supplied_exclusions_are_loaded(self):
        planner = self.planner()
        self.assertEqual(planner.collision_policy['srdf_pair_count'], 10)
        for entry in ET.fromstring(self.srdf_text).findall('disable_collisions'):
            pair = frozenset(self.bullet.link_ids[entry.get(key)] for key in ('link1', 'link2'))
            self.assertIn(pair, planner.disabled_pairs)
            self.assertNotIn(pair, planner.self_pairs)

    def test_excluded_pair_no_longer_blocks_start(self):
        planner = self.planner()
        ids = self.bullet.link_ids
        self.bullet.self_contacts = [contact(ids['shoulder_lift_link'], ids['torso_lift_link'])]
        before = copy.deepcopy((self.bullet.q, self.bullet.vel))
        self.assertIsNone(planner.plan(self.obs()))  # Empty cloud, not an invalid start.
        self.assertTrue(planner.last_stats['start_valid'])
        self.assertEqual(planner.last_stats['reason'], 'no_antipodal_pairs')
        self.assert_restored(before)

    def test_unrelated_collision_still_blocks_before_sampling_or_ik(self):
        planner = self.planner()
        ids = self.bullet.link_ids
        self.bullet.self_contacts = [contact(ids['wrist_roll_link'], ids['base_link'])]
        before = copy.deepcopy((self.bullet.q, self.bullet.vel))
        with patch.object(planner, 'sample_antipodal_grasps') as sample, \
                patch.object(grasp, '_ManipulatorIK') as ik:
            self.assertIsNone(planner.plan(self.obs()))
            sample.assert_not_called()
            ik.assert_not_called()
        stats = planner.last_stats
        self.assertEqual(stats['reason'], 'invalid_start_state')
        self.assertEqual(stats['tried'], 0)
        self.assertEqual(stats['opening_fail'], 0)
        self.assertEqual(stats['start_state_error']['link_names'], ('wrist_roll_link', 'base_link'))
        self.assertEqual(stats['invalid_examples'][0]['stage'], 'start')
        self.assert_restored(before)

    def test_obstacle_collision_is_not_disabled_by_srdf(self):
        planner = self.planner()
        ids = self.bullet.link_ids
        self.bullet.obstacle_contacts = [contact(ids['r_gripper_finger_link'], -1)]
        self.assertIsNone(planner.plan(self.obs()))
        error = planner.last_stats['start_state_error']
        self.assertEqual(error['reason'], 'obstacle_collision')
        self.assertEqual(error['robot_link_name'], 'r_gripper_finger_link')

    def test_torso_motion_and_follower_mapping_are_preserved(self):
        planner = self.planner()
        self.assertEqual(len(planner.motion_indices), 8)
        self.assertEqual(planner.motion_indices[0], planner.torso_index)
        self.assertFalse(planner.wrap[0])
        parked = self.obs()['qpos']
        motion = np.r_[0.12, np.full(7, 0.1)]
        target = planner._full(parked, motion, np.array([0.03, 0.03]))
        np.testing.assert_allclose(target[planner.motion_indices], motion)
        plan = types.SimpleNamespace(waypoints=np.array([parked, target]), phases=['transit', 'transit'])
        follower = grasp.GraspFollower(planner, plan)
        self.assertNotIn(planner.torso_index, follower.pinned)
        head_index = planner.action_names.index('head_pan_joint')
        self.assertIn(head_index, follower.pinned)

    def test_head_obstacle_collision_is_checked_with_moving_torso(self):
        planner = self.planner()
        head_link = self.bullet.link_ids['head_tilt_link']
        self.assertIn(head_link, planner.moving_links)
        self.bullet.obstacle_contacts = [contact(head_link, -1)]
        self.assertIsNone(planner.plan(self.obs()))
        error = planner.last_stats['start_state_error']
        self.assertEqual(error['reason'], 'obstacle_collision')
        self.assertEqual(error['robot_link_name'], 'head_tilt_link')

    def test_positive_distance_inside_margin_is_still_rejected(self):
        planner = self.planner()
        ids = self.bullet.link_ids
        self.bullet.self_contacts = [contact(ids['wrist_roll_link'], ids['base_link'], 0.001)]
        self.assertIsNone(planner.plan(self.obs()))
        self.assertEqual(planner.last_stats['start_state_error']['distance'], 0.001)

    def test_query_exception_restores_positions_and_velocities(self):
        planner = self.planner()
        self.bullet.fail_query = True
        before = copy.deepcopy((self.bullet.q, self.bullet.vel))
        with self.assertRaisesRegex(RuntimeError, 'injected'):
            planner.plan(self.obs())
        self.assert_restored(before)

    def test_start_deadline_reports_time_budget(self):
        planner = self.planner()
        with patch.object(grasp.time, 'monotonic', side_effect=[0., 31.]):
            self.assertIsNone(planner.plan(self.obs()))
        self.assertEqual(planner.last_stats['reason'], 'time_budget')
        self.assertFalse(planner.last_stats['start_valid'])

    def test_explicit_srdf_location(self):
        renamed = self.srdf.with_name('configured_robot.srdf')
        self.srdf.rename(renamed)
        self.assertEqual(self.planner(robot_srdf_path=renamed).srdf_path, renamed)

    def test_missing_sibling_is_an_error_by_default(self):
        self.srdf.unlink()
        with self.assertRaisesRegex(FileNotFoundError, 'robot_srdf_path'):
            self.planner()

    def test_missing_explicit_path_is_always_an_error(self):
        with self.assertRaises(FileNotFoundError):
            self.planner(robot_srdf_path=self.srdf.with_name('missing.srdf'), require_srdf=False)

    def test_intentional_opt_out_warns_and_keeps_other_checks(self):
        self.srdf.unlink()
        with warnings.catch_warnings(record=True) as messages:
            warnings.simplefilter('always')
            planner = self.planner(require_srdf=False)
        self.assertEqual(len(messages), 1)
        self.assertIn('No SRDF loaded', str(messages[0].message))
        ids = self.bullet.link_ids
        self.assertIn(frozenset((ids['shoulder_lift_link'], ids['torso_lift_link'])), planner.self_pairs)
        self.assertIsNone(planner.collision_policy['srdf_path'])

    def test_wrong_robot_name(self):
        self.write_srdf('', name='different_robot')
        with self.assertRaisesRegex(ValueError, 'matching the URDF'):
            self.planner()

    def test_wrong_root_element(self):
        self.srdf.write_text('<not_robot name="fetch"/>')
        with self.assertRaisesRegex(ValueError, 'matching the URDF'):
            self.planner()

    def test_malformed_xml(self):
        self.srdf.write_text('<robot')
        with self.assertRaisesRegex(ValueError, 'Malformed SRDF'):
            self.planner()

    def test_unknown_urdf_link(self):
        self.write_srdf('<disable_collisions link1="typo_link" link2="base_link"/>')
        with self.assertRaisesRegex(ValueError, 'unknown robot link.*typo_link'):
            self.planner()

    def test_missing_live_link(self):
        index = self.bullet.link_ids['upperarm_roll_link']
        row = list(self.bullet.info[index])
        row[12] = b'renamed_upperarm_link'
        self.bullet.info[index] = tuple(row)
        with self.assertRaisesRegex(ValueError, 'live robot.*unknown robot link.*upperarm_roll_link'):
            self.planner()

    def test_missing_link_attribute(self):
        self.write_srdf('<disable_collisions link1="base_link"/>')
        with self.assertRaisesRegex(ValueError, 'two nonempty link names'):
            self.planner()

    def test_same_link_pair(self):
        self.write_srdf('<disable_collisions link1="base_link" link2="base_link"/>')
        with self.assertRaisesRegex(ValueError, 'against itself'):
            self.planner()

    def test_extra_pairs_merge_and_deduplicate(self):
        pairs = [('torso_lift_link', 'shoulder_lift_link'),
                 ('wrist_roll_link', 'base_link'), ('base_link', 'wrist_roll_link')]
        planner = self.planner(disabled_self_pairs=pairs)
        self.assertEqual(planner.collision_policy['srdf_pair_count'], 10)
        self.assertEqual(planner.collision_policy['explicit_pair_count'], 2)
        self.assertEqual(planner.collision_policy['disabled_pair_count'], 11)

    def test_invalid_explicit_pairs(self):
        for pairs in ([('typo', 'base_link')], ['ab'], [('base_link',)], [None]):
            with self.subTest(pairs=pairs), self.assertRaises(ValueError):
                self.planner(disabled_self_pairs=pairs)

    def test_duplicate_srdf_pairs_are_canonicalised(self):
        self.write_srdf(
            '<disable_collisions link1="base_link" link2="wrist_roll_link"/>'
            '<disable_collisions link1="wrist_roll_link" link2="base_link"/>')
        self.assertEqual(self.planner().collision_policy['srdf_pair_count'], 1)

    def test_empty_valid_srdf_does_not_disable_unlisted_pairs(self):
        self.write_srdf('')
        planner = self.planner()
        self.assertEqual(planner.disabled_pairs, set())
        self.assertGreater(len(planner.self_pairs), 0)

    def test_geometric_only_sampler_needs_no_robot_files(self):
        planner = grasp.GraspPlanner(robot_urdf_path='/nonexistent/fetch.urdf')
        self.assertEqual(planner.sample_antipodal_grasps(
            points=np.empty((0, 3)), normals=np.empty((0, 3))), [])

