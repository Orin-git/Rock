#!/usr/bin/env python3
"""Phase2D-C1 / B2 fix unit tests (offline)."""

from __future__ import annotations

import sys
import threading
import unittest
from pathlib import Path
from unittest.mock import MagicMock

_SRC = Path(__file__).resolve().parents[1]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from xw_global_reloc.phase2d.patrol_planner import plan_patrol_goals  # noqa: E402


class TestPlannerStopFlag(unittest.TestCase):
    def test_should_stop_returns_early(self) -> None:
        model = MagicMock()
        model.cell_size_m = 1.0
        model.yaw_bins = 8
        model.cell_summaries.return_value = []
        model.slots = {}
        stop = threading.Event()
        stop.set()
        # map may be missing on bare host — skip if so
        maps = Path('/ros2_ws/maps/vp.yaml')
        if not maps.is_file():
            maps = Path('/home/radxa/ros2_ws/maps/vp.yaml')
        if not maps.is_file():
            self.skipTest('vp.yaml missing')
        cfg = {'patrol': {'micro_max_goals': 6}, 'coverage': {'cell_size_m': 1.0, 'yaw_bins': 8}}
        goals = plan_patrol_goals(
            model, map_yaml=maps, cfg=cfg, mode='micro', should_stop=stop.is_set
        )
        self.assertEqual(goals, [])


class TestCompleteStopProtect(unittest.TestCase):
    def test_stop_does_not_abort_complete(self) -> None:
        # Lightweight: exercise state guard logic without spinning ROS
        from xw_global_reloc.phase2d import build_orchestrator_node as bon

        class Fake:
            def __init__(self):
                self._stop = threading.Event()
                self._nav_cancel_pub = MagicMock()
                self._nav_goal_handle = None
                self._lock = threading.Lock()
                self._session = bon.BuildSession(build_session_id='t', mode='micro', state='COMPLETE')
                self.states = []

            def _set_state(self, st, msg=''):
                self.states.append((st, msg))
                self._session.state = st
                self._session.message = msg

            def _persist_session(self):
                pass

            def _disarm_motion_subs(self):
                pass

            def status_dict(self):
                return {'state': self._session.state}

        f = Fake()
        out = bon.VisualDbBuildOrchestrator.stop_build(f, abort=True)
        self.assertEqual(f._session.state, 'COMPLETE')
        self.assertEqual(f.states, [])
        self.assertTrue(f._stop.is_set())
        self.assertEqual(out['state'], 'COMPLETE')


if __name__ == '__main__':
    unittest.main(verbosity=2)
