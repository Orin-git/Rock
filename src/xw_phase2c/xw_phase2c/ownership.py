"""Phase2C production /initialpose single-owner mutex.

Automatic writers (BOOT cascade, LOST recovery, Reloc handoff) must claim
ownership before publishing /initialpose. Operator /initialpose stays allowed
as recovery, but auto flows must not race each other.

Topic (latched String JSON):
  /xw/localization/initialpose_owner
  {"owner":"none|boot|lost|reloc|operator|legacy","session_id":N,"stamp":...}
"""

from __future__ import annotations

import json
import threading
import time
from enum import Enum
from typing import Any, Dict, Optional


OWNER_TOPIC = '/xw/localization/initialpose_owner'


class InitialPoseOwner(str, Enum):
    NONE = 'none'
    BOOT = 'boot'
    LOST = 'lost'
    RELOC = 'reloc'
    OPERATOR = 'operator'
    LEGACY = 'legacy'


# Owners that block another automatic cascade from starting.
_AUTO_ACTIVE = frozenset(
    {
        InitialPoseOwner.BOOT.value,
        InitialPoseOwner.LOST.value,
        InitialPoseOwner.RELOC.value,
    }
)


def owner_payload(
    owner: str | InitialPoseOwner,
    session_id: int,
    *,
    note: str = '',
    stamp: Optional[float] = None,
) -> str:
    return json.dumps(
        {
            'owner': str(owner.value if isinstance(owner, InitialPoseOwner) else owner),
            'session_id': int(session_id),
            'stamp': float(stamp if stamp is not None else time.time()),
            'note': note or '',
        },
        separators=(',', ':'),
    )


def parse_owner(raw: str) -> Dict[str, Any]:
    try:
        d = json.loads(raw or '{}')
    except json.JSONDecodeError:
        d = {}
    return {
        'owner': str(d.get('owner') or InitialPoseOwner.NONE.value),
        'session_id': int(d.get('session_id') or 0),
        'stamp': float(d.get('stamp') or 0.0),
        'note': str(d.get('note') or ''),
    }


def auto_owner_active(owner: str) -> bool:
    return str(owner or '') in _AUTO_ACTIVE


class OwnershipGuard:
    """Thread-safe local claim helper used by BOOT / LOST controllers."""

    def __init__(self, self_owner: InitialPoseOwner) -> None:
        self.self_owner = self_owner
        self._lock = threading.Lock()
        self._remote_owner = InitialPoseOwner.NONE.value
        self._remote_session = 0
        self._holding = False
        self._session_id = 0

    def on_remote(self, raw: str) -> None:
        info = parse_owner(raw)
        with self._lock:
            self._remote_owner = info['owner']
            self._remote_session = info['session_id']

    def remote_blocks(self) -> bool:
        with self._lock:
            if self._holding:
                return False
            return auto_owner_active(self._remote_owner) and self._remote_owner != self.self_owner.value

    def begin(self, session_id: int) -> bool:
        with self._lock:
            if self._holding:
                return False
            if auto_owner_active(self._remote_owner) and self._remote_owner != self.self_owner.value:
                return False
            self._holding = True
            self._session_id = int(session_id)
            return True

    def end(self) -> None:
        with self._lock:
            self._holding = False

    def holding(self) -> bool:
        with self._lock:
            return self._holding

    def session_id(self) -> int:
        with self._lock:
            return self._session_id

    def claim_json(self, note: str = '') -> str:
        return owner_payload(self.self_owner, self.session_id(), note=note)

    def release_json(self, note: str = 'released') -> str:
        return owner_payload(InitialPoseOwner.NONE, self.session_id(), note=note)
