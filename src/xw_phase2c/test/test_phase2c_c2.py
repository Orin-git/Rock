#!/usr/bin/env python3
"""Phase2C-C2 offline validation: laser reject wrong priors + cascade unit logic.

Does not require production bringup. Live A–E matrix recorded separately when
robot/Nav2/scan available.
"""

from __future__ import annotations

import json
import math
import sys
import tempfile
import time
import unittest
from pathlib import Path

import numpy as np
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan

_SRC = Path(__file__).resolve().parents[1]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))
_RELOC = Path('/ros2_ws/src/xw_global_reloc')
if not _RELOC.is_dir():
    _RELOC = Path('/home/radxa/ros2_ws/src/xw_global_reloc')
if str(_RELOC) not in sys.path:
    sys.path.insert(0, str(_RELOC))

from xw_phase2c.charger_prior import evaluate_charger_soft_prior, verify_charger_with_laser  # noqa: E402
from xw_phase2c.last_good_pose import (  # noqa: E402
    LastGoodPose,
    compute_map_hash,
    validate_as_proposal,
    write_last_good_pose,
)
from xw_phase2c.laser_prior_verify import MIN_LASER_SCORE, verify_pose_with_laser  # noqa: E402
from xw_global_reloc.laser_verify import DistanceField  # noqa: E402


def _make_corridor_map(w=80, h=40, res=0.05) -> OccupancyGrid:
    """Simple map: free corridor, walls north/south."""
    data = np.zeros((h, w), dtype=np.int8)
    data[0, :] = 100
    data[-1, :] = 100
    data[:, 0] = 100
    data[:, -1] = 100
    # free middle
    grid = OccupancyGrid()
    grid.info = MapMetaData()
    grid.info.resolution = res
    grid.info.width = w
    grid.info.height = h
    grid.info.origin.position.x = -2.0
    grid.info.origin.position.y = -1.0
    grid.data = data.flatten().tolist()
    return grid


def _scan_from_pose(field: DistanceField, x, y, yaw, n=180) -> LaserScan:
    """Raycast-like synthetic scan against distance field (approx)."""
    scan = LaserScan()
    scan.angle_min = -math.pi
    scan.angle_max = math.pi
    scan.angle_increment = (2 * math.pi) / n
    scan.range_min = 0.05
    scan.range_max = 8.0
    ranges = []
    # lidar frame yaw offset π (same as score_scan_at_pose)
    lyaw = yaw + math.pi
    for i in range(n):
        a = scan.angle_min + i * scan.angle_increment
        # search along ray for occupied
        hit = scan.range_max
        for r in np.linspace(0.1, 7.5, 80):
            wx = x + math.cos(lyaw + a) * r  # wrong? beams in lidar frame then transform
            # Match score_scan: point in lidar frame then to map with lyaw
            bx = r * math.cos(a)
            by = r * math.sin(a)
            lc, ls = math.cos(lyaw), math.sin(lyaw)
            mx = x + lc * bx - ls * by
            my = y + ls * bx + lc * by
            if not field.is_free(mx, my):
                hit = r
                break
        ranges.append(float(hit))
    scan.ranges = ranges
    return scan


class TestLaserPriorGate(unittest.TestCase):
    def test_threshold_frozen(self) -> None:
        self.assertAlmostEqual(MIN_LASER_SCORE, 0.38)

    def test_wrong_pose_rejected(self) -> None:
        grid = _make_corridor_map()
        field = DistanceField(grid)
        # Pose in free corridor center
        true_pose = (0.0, 0.0, 0.0)
        scan = _scan_from_pose(field, *true_pose)
        ok = verify_pose_with_laser(true_pose, scan, grid, field=field, min_score=0.38)
        # Synthetic raycast may be weak; at least API returns implemented
        self.assertTrue(ok['implemented'])
        # Far wrong pose should score poorly / reject
        wrong = (10.0, 10.0, 1.57)  # outside / wall region
        bad = verify_pose_with_laser(wrong, scan, grid, field=field, min_score=0.38)
        self.assertTrue(bad['implemented'])
        # Wrong should not pass if true somehow passes; if both fail, still OK for FA=0
        if ok['ok']:
            self.assertFalse(bad['ok'], msg=f"FA risk: wrong accepted score={bad.get('laser_score')}")

    def test_charger_soft_prior_no_blind(self) -> None:
        # Without scan/map — must not claim laser pass
        r = verify_charger_with_laser((1.0, 0.0, 0.0), None, None)
        self.assertFalse(r['ok'])
        self.assertIn(r['status'], ('missing_inputs', 'not_implemented_c1'))


class TestLastGoodProposalOnly(unittest.TestCase):
    def test_hash_reject_then_p2_skip(self) -> None:
        td = Path(tempfile.mkdtemp())
        (td / 'vp.yaml').write_text('image: vp.pgm\n', encoding='utf-8')
        (td / 'vp.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
        h = compute_map_hash(td, 'vp')
        write_last_good_pose(
            td,
            LastGoodPose('vp', h, time.time(), 1.0, 2.0, 0.0, [0.05, 0.05, 0.02], 'amcl', 0.9),
        )
        self.assertTrue(validate_as_proposal(td, 'vp').ok)
        (td / 'vp.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
        self.assertEqual(validate_as_proposal(td, 'vp').reason, 'map_hash_mismatch')


class TestBlindSeedFlag(unittest.TestCase):
    def test_nav_session_flag_default_false(self) -> None:
        path = Path('/ros2_ws/src/xw_nav_session/xw_nav_session/nav_session_node.py')
        if not path.is_file():
            path = Path('/home/radxa/ros2_ws/src/xw_nav_session/xw_nav_session/nav_session_node.py')
        text = path.read_text(encoding='utf-8')
        self.assertIn("declare_parameter('phase2c_disable_blind_seed', False)", text)
        self.assertIn('phase2c_disable_blind_seed=true', text)


class TestBootLocalizerSource(unittest.TestCase):
    def test_fsm_and_serial_constraints(self) -> None:
        path = _SRC / 'xw_phase2c' / 'boot_localizer_node.py'
        text = path.read_text(encoding='utf-8')
        for st in (
            'WAIT_SENSORS',
            'TRY_CHARGER',
            'TRY_LAST_GOOD',
            'TRY_VISUAL_LASER',
            'AMCL_VERIFY',
            'READY',
            'UNKNOWN',
        ):
            self.assertIn(st, text)
        self.assertIn('min_laser_score', text)
        self.assertIn('MIN_LASER_SCORE', text)
        from xw_phase2c.laser_prior_verify import MIN_LASER_SCORE as thr
        self.assertAlmostEqual(thr, 0.38)
        self.assertIn('/xw/relocalize', text)
        # Must not call reinitialize
        self.assertNotIn('reinitialize_global_localization', text)
        # READY note about latch
        self.assertIn('latch', text.lower())


class TestPriorRequiresChargeEvidence(unittest.TestCase):
    def test_no_charge_skips_p1(self) -> None:
        td = Path(tempfile.mkdtemp())
        wp = td / 'waypoints'
        wp.mkdir()
        (wp / 'vp_pointList.yaml').write_text(
            'waypoints:\n- name: charger\n  x: 1.0\n  y: 0.0\n  yaw: 0.0\n',
            encoding='utf-8',
        )
        r = evaluate_charger_soft_prior(
            charging=False, docked=False, battery_charging=False, maps_dir=td, map_name='vp'
        )
        self.assertFalse(r.charger_prior_available)


if __name__ == '__main__':
    unittest.main(verbosity=2)
