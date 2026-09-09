#!/usr/bin/env python3
"""Phase2C-C3 offline validation: LOST STOP+Reloc owner, flags, policies.

Does not require live bringup. Live A–E scenarios recorded in the C3 report.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

_SRC = Path(__file__).resolve().parents[1]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from xw_phase2c.recovery_state import Phase2CLocState, TaskSnapshot  # noqa: E402


def _read(rel: str) -> str:
    for root in (Path('/ros2_ws/src'), Path('/home/radxa/ros2_ws/src')):
        p = root / rel
        if p.is_file():
            return p.read_text(encoding='utf-8')
    p = _SRC / rel.replace('xw_phase2c/', '')
    if p.is_file():
        return p.read_text(encoding='utf-8')
    raise FileNotFoundError(rel)


class TestC3FlagDefaults(unittest.TestCase):
    def test_lost_recovery_flag_default_false(self) -> None:
        text = (_SRC / 'xw_phase2c' / 'lost_recovery_node.py').read_text(encoding='utf-8')
        self.assertIn("declare_parameter('phase2c_lost_recovery_enabled', False)", text)
        self.assertIn("declare_parameter('phase2c_localization_enabled', False)", text)

    def test_supervisor_c3_flag_default_false(self) -> None:
        text = _read('xw_supervisor/xw_supervisor/supervisor_node.py')
        self.assertIn("declare_parameter('phase2c_lost_recovery_enabled', False)", text)
        self.assertIn("declare_parameter('phase2c_localization_enabled', False)", text)

    def test_robot_launch_wires_phase2c(self) -> None:
        # C4B: production bringup includes lost_recovery behind master switch.
        text = _read('xw_bringup/launch/robot.launch.py')
        self.assertIn('phase2c_localization_enabled', text)
        self.assertIn('lost_recovery', text)
        self.assertIn('xw_boot_localizer', text)


class TestLostDetectionAndStop(unittest.TestCase):
    def test_unified_states(self) -> None:
        for s in Phase2CLocState:
            self.assertIn(
                s.value,
                (
                    'READY',
                    'BOOT_LOCALIZING',
                    'DEGRADED',
                    'LOST',
                    'RECOVERING',
                    'UNKNOWN',
                    'NEED_OPERATOR',
                ),
            )

    def test_stop_order_and_no_reinit(self) -> None:
        text = (_SRC / 'xw_phase2c' / 'lost_recovery_node.py').read_text(encoding='utf-8')
        # STOP sequence markers
        for needle in (
            '/xw/nav/cancel',
            'goals_blocked',
            'phase2c_recovery',
            '/xw/relocalize',
            'NEED_OPERATOR',
        ):
            self.assertIn(needle, text)
        self.assertIn("declare_parameter('auto_resume_follow', False)", text)
        self.assertNotIn('reinitialize_global_localization', text)
        # Debounce sources
        self.assertIn('status_3', text)
        self.assertIn('status_2_sustained', text)
        self.assertIn('tf_dead', text)
        self.assertIn('pose_jump', text)

    def test_stop_before_reloc(self) -> None:
        text = (_SRC / 'xw_phase2c' / 'lost_recovery_node.py').read_text(encoding='utf-8')
        stop_idx = text.find('_stop_motion_first')
        reloc_idx = text.find('_call_reloc')
        self.assertGreater(stop_idx, 0)
        self.assertGreater(reloc_idx, stop_idx)
        worker = text[text.find('def _recovery_worker') :]
        self.assertLess(worker.find('_stop_motion_first'), worker.find('_try_r1'))
        self.assertLess(worker.find('_try_r1'), worker.find('_try_r2'))
        self.assertLess(worker.find('_try_r2'), worker.find('_try_r3'))
        r3 = text[text.find('def _try_r3') :]
        self.assertIn('_call_reloc', r3)
        for code in (
            'R1_CURRENT_ACCEPT',
            'R1_CURRENT_REJECT',
            'R1_CURRENT_SKIP',
            'R2_LAST_GOOD_ACCEPT',
            'R2_LAST_GOOD_REJECT',
            'R2_LAST_GOOD_SKIP',
            'R3_VISUAL_ACCEPT',
            'R3_VISUAL_UNKNOWN',
        ):
            self.assertIn(code, text)


class TestSnapshotAndResumePolicy(unittest.TestCase):
    def test_snapshot_fields(self) -> None:
        snap = TaskSnapshot(
            task_type='navigate',
            nav_goal={'x': 1.0, 'y': 2.0, 'yaw': 0.0},
            follow_was_on=False,
            recharge_was_on=True,
            patrol_was_on=False,
            map_name='vp',
            reason='status_3',
        )
        d = __import__('json').loads(snap.to_json())
        for k in (
            'task_type',
            'nav_goal',
            'follow_was_on',
            'recharge_was_on',
            'patrol_was_on',
            'map_name',
            'reason',
            'stamp',
        ):
            self.assertIn(k, d)

    def test_resume_policies_in_source(self) -> None:
        text = (_SRC / 'xw_phase2c' / 'lost_recovery_node.py').read_text(encoding='utf-8')
        self.assertIn("auto_resume_follow', False)", text)
        self.assertIn('no_auto', text)
        self.assertIn('NEED_OPERATOR', text)
        self.assertIn('forbidden', text)
        # Nav replan allowed
        self.assertIn('replan_published', text)
        # Recharge re-check
        self.assertIn('already_charging_skip', text)


class TestSingleRecoveryOwner(unittest.TestCase):
    def test_health_blocks_heal_on_phase2c(self) -> None:
        text = _read('xw_localization_health/xw_localization_health/localization_health_node.py')
        self.assertIn('phase2c_recovery', text)
        self.assertIn('_phase2c_recovery', text)
        # heal forbidden when phase2c active
        self.assertIn('if self._phase2c_recovery:', text)
        self.assertIn('return False', text)

    def test_anti_reentry_mutex(self) -> None:
        text = (_SRC / 'xw_phase2c' / 'lost_recovery_node.py').read_text(encoding='utf-8')
        self.assertIn('self._busy', text)
        self.assertIn('unknown_cooldown', text)
        self.assertIn('post_ready_guard', text)
        self.assertIn('p3_max_attempts', text)

    def test_ready_not_latch_only(self) -> None:
        text = (_SRC / 'xw_phase2c' / 'lost_recovery_node.py').read_text(encoding='utf-8')
        self.assertIn('_amcl_ready_stable', text)
        self.assertIn('ready_stable_sec', text)
        self.assertIn('cov_xy', text)


class TestSupervisorC3Integration(unittest.TestCase):
    def test_suppress_heal_and_idle_guard(self) -> None:
        text = _read('xw_supervisor/xw_supervisor/supervisor_node.py')
        self.assertIn('Reloc path (heal suppressed)', text)
        self.assertIn('keeping recovery ownership', text)
        self.assertIn('_on_phase2c_snapshot', text)


class TestLaunchNotProduction(unittest.TestCase):
    def test_c3_launch_exists(self) -> None:
        p = _SRC / 'launch' / 'phase2c_c3_lost.launch.py'
        self.assertTrue(p.is_file())
        text = p.read_text(encoding='utf-8')
        self.assertIn('phase2c_lost_recovery_enabled', text)
        # Dev launch retained; production wiring lives in robot.launch.py (C4B).
        self.assertIn('xw_lost_recovery', text)


if __name__ == '__main__':
    unittest.main(verbosity=2)
