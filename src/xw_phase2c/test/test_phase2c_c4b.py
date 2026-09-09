#!/usr/bin/env python3
"""Phase2C-C4B offline validation: master switch, ownership, bringup wiring."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

_SRC = Path(__file__).resolve().parents[1]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from xw_phase2c.ownership import (  # noqa: E402
    InitialPoseOwner,
    OwnershipGuard,
    auto_owner_active,
    owner_payload,
    parse_owner,
)
from xw_phase2c.recovery_state import Phase2CLocState  # noqa: E402


def _read(rel: str) -> str:
    for root in (Path('/ros2_ws/src'), Path('/home/radxa/ros2_ws/src')):
        p = root / rel
        if p.is_file():
            return p.read_text(encoding='utf-8')
    raise FileNotFoundError(rel)


class TestOwnershipMutex(unittest.TestCase):
    def test_payload_roundtrip(self) -> None:
        raw = owner_payload(InitialPoseOwner.BOOT, 7, note='nav_enable')
        info = parse_owner(raw)
        self.assertEqual(info['owner'], 'boot')
        self.assertEqual(info['session_id'], 7)
        self.assertTrue(auto_owner_active('boot'))
        self.assertFalse(auto_owner_active('none'))

    def test_guard_blocks_remote_auto(self) -> None:
        g = OwnershipGuard(InitialPoseOwner.LOST)
        g.on_remote(owner_payload(InitialPoseOwner.BOOT, 1))
        self.assertTrue(g.remote_blocks())
        self.assertFalse(g.begin(2))
        g.on_remote(owner_payload(InitialPoseOwner.NONE, 1))
        self.assertTrue(g.begin(3))
        self.assertTrue(g.holding())
        g.end()
        self.assertFalse(g.holding())


class TestMasterSwitchWiring(unittest.TestCase):
    def test_robot_launch_production_nodes(self) -> None:
        text = _read('xw_bringup/launch/robot.launch.py')
        for needle in (
            'phase2c_localization_enabled',
            'xw_global_reloc_poc',
            'xw_last_good_pose_writer',
            'xw_charger_prior',
            'xw_boot_localizer',
            'xw_lost_recovery',
            'allow_amcl_handoff',
            'accept_external_prior_inject',
            "default_value='true'",
        ):
            self.assertIn(needle, text)

    def test_nav_session_master_gates_seed(self) -> None:
        text = _read('xw_nav_session/xw_nav_session/nav_session_node.py')
        self.assertIn('phase2c_localization_enabled', text)
        self.assertIn('_phase2c_blind_seed_disabled', text)
        self.assertIn('refusing blind seed while phase2c_localization_enabled', text)

    def test_supervisor_master(self) -> None:
        text = _read('xw_supervisor/xw_supervisor/supervisor_node.py')
        self.assertIn('phase2c_localization_enabled', text)
        self.assertIn('BOOT_LOCALIZING', text)
        self.assertIn('NEED_OPERATOR', text)

    def test_boot_session_and_goals(self) -> None:
        text = (_SRC / 'xw_phase2c' / 'boot_localizer_node.py').read_text(encoding='utf-8')
        self.assertIn('phase2c_localization_enabled', text)
        self.assertIn('goals_blocked', text)
        self.assertIn('BOOT_LOCALIZING', text)
        self.assertIn('accept_external_prior_inject', text)
        self.assertIn('_request_session_boot', text)
        self.assertIn('OWNER_TOPIC', text)
        self.assertIn('PRE_LOCALIZATION_READY', text)
        self.assertIn('POST_SEED_AMCL_READY', text)
        self.assertIn('VERIFYING_OPERATOR_POSE', text)
        self.assertIn('InitialPoseOwner.OPERATOR', text)
        self.assertIn('operator_post_seed', text)
        # Frozen threshold — no retune in C4B
        self.assertIn('min_laser_score', text)
        self.assertNotIn('reinitialize_global_localization', text)

    def test_lost_ownership(self) -> None:
        text = (_SRC / 'xw_phase2c' / 'lost_recovery_node.py').read_text(encoding='utf-8')
        self.assertIn('phase2c_localization_enabled', text)
        self.assertIn('_boot_busy', text)
        self.assertIn('NEED_OPERATOR', text)
        self.assertIn('OWNER_TOPIC', text)
        self.assertNotIn('reinitialize_global_localization', text)


class TestLocStates(unittest.TestCase):
    def test_c4b_states(self) -> None:
        for need in (
            Phase2CLocState.BOOT_LOCALIZING,
            Phase2CLocState.NEED_OPERATOR,
            Phase2CLocState.READY,
            Phase2CLocState.LOST,
            Phase2CLocState.RECOVERING,
        ):
            self.assertTrue(need.value)


if __name__ == '__main__':
    unittest.main(verbosity=2)
