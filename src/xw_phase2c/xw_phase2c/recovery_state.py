"""Independent Phase2C recovery ownership — must NOT rely solely on recovery_enable.

Supervisor IDLE clears /xw/localization/recovery_enable. Phase2C controller keeps
its own latched state on /xw/localization/phase2c_recovery (+ task snapshot JSON).
"""

from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any, Dict, Optional


class Phase2CLocState(str, Enum):
    READY = 'READY'
    DEGRADED = 'DEGRADED'
    LOST = 'LOST'
    RECOVERING = 'RECOVERING'
    UNKNOWN = 'UNKNOWN'


@dataclass
class TaskSnapshot:
    task_type: str = 'none'  # none|navigate|follow|recharge|patrol
    nav_goal: Optional[Dict[str, float]] = None
    follow_was_on: bool = False
    recharge_was_on: bool = False
    patrol_was_on: bool = False
    map_name: str = ''
    reason: str = ''
    stamp: float = field(default_factory=time.time)

    def to_json(self) -> str:
        return json.dumps(asdict(self), separators=(',', ':'))

    @classmethod
    def from_json(cls, raw: str) -> 'TaskSnapshot':
        try:
            d = json.loads(raw or '{}')
        except json.JSONDecodeError:
            d = {}
        return cls(
            task_type=str(d.get('task_type') or 'none'),
            nav_goal=d.get('nav_goal'),
            follow_was_on=bool(d.get('follow_was_on')),
            recharge_was_on=bool(d.get('recharge_was_on')),
            patrol_was_on=bool(d.get('patrol_was_on')),
            map_name=str(d.get('map_name') or ''),
            reason=str(d.get('reason') or ''),
            stamp=float(d.get('stamp') or time.time()),
        )


@dataclass
class Phase2CRecoveryState:
    """Owned by Phase2C controller (Supervisor C1 path or future boot_lost_localizer)."""

    active: bool = False
    state: Phase2CLocState = Phase2CLocState.READY
    snapshot: Optional[TaskSnapshot] = None
    goals_blocked: bool = False
    # Explicit: do not treat health latch-clear as READY.
    note: str = 'READY requires post-seed AMCL+cov+TF+stable_window; never latch-cleared-only'

    def enter_lost(self, snapshot: TaskSnapshot) -> None:
        self.active = True
        self.state = Phase2CLocState.RECOVERING
        self.snapshot = snapshot
        self.goals_blocked = True

    def clear(self, to_state: Phase2CLocState = Phase2CLocState.READY) -> None:
        self.active = False
        self.state = to_state
        self.goals_blocked = False
        # Keep last snapshot for audit until overwritten.
