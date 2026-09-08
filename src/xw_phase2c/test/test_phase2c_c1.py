#!/usr/bin/env python3
"""Phase2C-C1 offline / unit tests (no production Reloc, no robot.launch change).

Run inside ROS env or with PYTHONPATH to src:
  python3 -m pytest ... OR
  python3 ros2_ws/src/xw_phase2c/test/test_phase2c_c1.py
"""

from __future__ import annotations

import json
import os
import shutil
import sys
import tempfile
import time
import unittest
from pathlib import Path

# Allow running without install.
_SRC = Path(__file__).resolve().parents[1]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from xw_phase2c.last_good_pose import (  # noqa: E402
    LastGoodPose,
    compute_map_hash,
    read_last_good_pose,
    validate_as_proposal,
    write_last_good_pose,
)
from xw_phase2c.charger_prior import (  # noqa: E402
    evaluate_charger_soft_prior,
    load_charger_waypoint,
    verify_charger_with_laser,
)
from xw_phase2c.recovery_state import Phase2CRecoveryState, TaskSnapshot  # noqa: E402

_PERC = Path('/ros2_ws/src/xw_perception')
if not _PERC.is_dir():
    _PERC = Path('/home/radxa/ros2_ws/src/xw_perception')
if str(_PERC) not in sys.path:
    sys.path.insert(0, str(_PERC))
from xw_perception.perception_mode_manager_node import _PROFILES  # noqa: E402


class TestLastGoodPose(unittest.TestCase):
    def setUp(self) -> None:
        self.td = Path(tempfile.mkdtemp(prefix='lgp_'))
        # Minimal map files for hash.
        (self.td / 'vp.yaml').write_text('image: vp.pgm\nresolution: 0.05\n', encoding='utf-8')
        (self.td / 'vp.pgm').write_bytes(b'P5\n1 1\n255\n\x00')

    def tearDown(self) -> None:
        shutil.rmtree(self.td, ignore_errors=True)

    def test_write_and_read(self) -> None:
        h = compute_map_hash(self.td, 'vp')
        self.assertTrue(h)
        pose = LastGoodPose(
            map_name='vp',
            map_hash=h,
            timestamp=time.time(),
            x=1.0,
            y=2.0,
            yaw=0.5,
            covariance=[0.1, 0.1, 0.05],
            source='amcl',
            quality=0.8,
        )
        path = write_last_good_pose(self.td, pose)
        self.assertTrue(path.is_file())
        # Atomic: no leftover tmp
        self.assertFalse(path.with_suffix('.yaml.tmp').exists())
        got = read_last_good_pose(self.td, 'vp')
        self.assertIsNotNone(got)
        assert got is not None
        self.assertAlmostEqual(got.x, 1.0)
        self.assertEqual(got.map_hash, h)
        self.assertIn('PROPOSAL_ONLY', pose.as_dict()['note'])

    def test_hash_mismatch_reject(self) -> None:
        h = compute_map_hash(self.td, 'vp')
        write_last_good_pose(
            self.td,
            LastGoodPose(
                map_name='vp',
                map_hash=h,
                timestamp=time.time(),
                x=0.0,
                y=0.0,
                yaw=0.0,
                covariance=[0.1, 0.1, 0.05],
                quality=0.9,
            ),
        )
        # Change map → hash mismatch
        (self.td / 'vp.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
        res = validate_as_proposal(self.td, 'vp', min_quality=0.3)
        self.assertFalse(res.ok)
        self.assertEqual(res.reason, 'map_hash_mismatch')

    def test_age_and_quality_reject(self) -> None:
        h = compute_map_hash(self.td, 'vp')
        write_last_good_pose(
            self.td,
            LastGoodPose(
                map_name='vp',
                map_hash=h,
                timestamp=time.time() - 999999,
                x=0.0,
                y=0.0,
                yaw=0.0,
                covariance=[0.1, 0.1, 0.05],
                quality=0.9,
            ),
        )
        res = validate_as_proposal(self.td, 'vp', max_age_sec=60.0)
        self.assertFalse(res.ok)
        self.assertEqual(res.reason, 'age_exceeded')

        write_last_good_pose(
            self.td,
            LastGoodPose(
                map_name='vp',
                map_hash=h,
                timestamp=time.time(),
                x=0.0,
                y=0.0,
                yaw=0.0,
                covariance=[0.1, 0.1, 0.05],
                quality=0.1,
            ),
        )
        res2 = validate_as_proposal(self.td, 'vp', min_quality=0.35)
        self.assertFalse(res2.ok)
        self.assertEqual(res2.reason, 'quality_too_low')


class TestChargerPrior(unittest.TestCase):
    def setUp(self) -> None:
        self.td = Path(tempfile.mkdtemp(prefix='chg_'))
        wp = self.td / 'waypoints'
        wp.mkdir()
        (wp / 'vp_pointList.yaml').write_text(
            'waypoints:\n- name: charger\n  x: 1.87\n  y: -0.06\n  yaw: -3.13\n',
            encoding='utf-8',
        )

    def tearDown(self) -> None:
        shutil.rmtree(self.td, ignore_errors=True)

    def test_soft_prior_available(self) -> None:
        r = evaluate_charger_soft_prior(
            charging=True,
            docked=True,
            battery_charging=False,
            maps_dir=self.td,
            map_name='vp',
        )
        self.assertTrue(r.charger_prior_available)
        self.assertTrue(r.soft_prior_only)
        self.assertIsNotNone(r.charger_waypoint)

    def test_no_charge_evidence(self) -> None:
        r = evaluate_charger_soft_prior(
            charging=False,
            docked=False,
            battery_charging=False,
            maps_dir=self.td,
            map_name='vp',
        )
        self.assertFalse(r.charger_prior_available)
        self.assertEqual(r.reason, 'no_charging_evidence')

    def test_laser_stub_no_initialpose(self) -> None:
        out = verify_charger_with_laser((1.87, -0.06, -3.13))
        self.assertFalse(out['ok'])
        # Without scan/map → missing_inputs (C2 implemented laser path)
        self.assertIn(out.get('status'), ('missing_inputs', 'not_implemented_c1'))
        src = (_SRC / 'xw_phase2c' / 'charger_prior.py').read_text(encoding='utf-8')
        self.assertNotIn('create_publisher', src)
        node_src = (_SRC / 'xw_phase2c' / 'charger_prior_node.py').read_text(encoding='utf-8')
        self.assertNotIn("'/initialpose'", node_src)
        self.assertNotIn('"/initialpose"', node_src)
        self.assertIn('SOFT PRIOR', node_src)

    def test_real_maps_charger(self) -> None:
        maps = Path(os.environ.get('XW_MAPS', '/home/radxa/ros2_ws/maps'))
        if not (maps / 'waypoints' / 'vp_pointList.yaml').is_file():
            self.skipTest('vp waypoints not present')
        wp = load_charger_waypoint(maps, 'vp')
        self.assertIsNotNone(wp)


class TestRecoveryProfile(unittest.TestCase):
    def test_profile_dict(self) -> None:
        p = _PROFILES['LOCALIZATION_RECOVERY']
        self.assertTrue(p['rgb_up'])
        self.assertFalse(p['rgb_down'])
        self.assertFalse(p['depth_up'])
        self.assertFalse(p['depth_down'])
        self.assertFalse(p['points_nav'])
        self.assertEqual(p['fall_infer_fps'], 0.0)
        self.assertEqual(p['follow_infer_fps'], 0.0)

    def test_recovery_state_independent(self) -> None:
        st = Phase2CRecoveryState()
        st.enter_lost(TaskSnapshot(task_type='navigate', reason='test'))
        self.assertTrue(st.active)
        self.assertTrue(st.goals_blocked)
        st.clear()
        self.assertFalse(st.active)
        self.assertFalse(st.goals_blocked)


class TestSupervisorFlagDefault(unittest.TestCase):
    def test_flag_default_false_in_source(self) -> None:
        path = Path('/ros2_ws/src/xw_supervisor/xw_supervisor/supervisor_node.py')
        if not path.is_file():
            path = Path('/home/radxa/ros2_ws/src/xw_supervisor/xw_supervisor/supervisor_node.py')
        text = path.read_text(encoding='utf-8')
        self.assertIn("declare_parameter('phase2c_lost_cancel_enabled', False)", text)
        self.assertNotIn('/xw/relocalize', text)


class TestWriterFollowBlock(unittest.TestCase):
    def test_writer_blocks_follow(self) -> None:
        path = _SRC / 'xw_phase2c' / 'last_good_pose_writer_node.py'
        text = path.read_text(encoding='utf-8')
        self.assertIn("block_write_during_follow", text)
        self.assertIn('follow_freeze_block', text)


if __name__ == '__main__':
    unittest.main(verbosity=2)
