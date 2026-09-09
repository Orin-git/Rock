"""Single Phase2C incident stream.

Boot and lost submit events. Supervisor is the only publisher of the latched
system topics (`/xw/localization/phase2c_loc_state`, `/xw/nav/goals_blocked`).
A newer generation always wins, so an old READY latch cannot cover NEED_OPERATOR.
"""

from __future__ import annotations

import json
import time
from typing import Any, Dict, Optional

CANONICAL_TOPIC = '/xw/localization/phase2c_state'
# Boot/lost submit here. Supervisor is the only publisher of CANONICAL_TOPIC.
SUBMIT_TOPIC = '/xw/localization/phase2c_event'

_INCIDENT = {
    'LOST',
    'RECOVERING',
    'NEED_OPERATOR',
    'VERIFYING_OPERATOR_POSE',
    'BOOT_LOCALIZING',
}


def make_event(
    state: str,
    goals_blocked: bool,
    source: str,
    incident_id: int = 0,
    generation: Optional[int] = None,
) -> str:
    gen = int(generation if generation is not None else time.time_ns())
    return json.dumps(
        {
            'state': str(state or 'READY'),
            'goals_blocked': bool(goals_blocked),
            'source': str(source or ''),
            'incident_id': int(incident_id),
            'generation': gen,
        },
        separators=(',', ':'),
    )


def parse_event(raw: str) -> Dict[str, Any]:
    try:
        data = json.loads(raw or '{}')
    except json.JSONDecodeError:
        data = {}
    if not isinstance(data, dict):
        data = {}
    try:
        gen = int(data.get('generation') or 0)
    except (TypeError, ValueError):
        gen = 0
    return {
        'state': str(data.get('state') or ''),
        'goals_blocked': bool(data.get('goals_blocked')),
        'source': str(data.get('source') or ''),
        'incident_id': int(data.get('incident_id') or 0),
        'generation': gen,
    }


def is_newer(event: Dict[str, Any], current_generation: int) -> bool:
    try:
        gen = int(event.get('generation') or 0)
    except (TypeError, ValueError):
        return False
    return gen > int(current_generation)


def is_open_incident(state: str) -> bool:
    return str(state or '') in _INCIDENT


def latched_pose_may_satisfy_ready(latched_only: bool, near: bool) -> bool:
    """A far latched AMCL pose must never count as post-seed convergence."""
    if latched_only and not near:
        return False
    return True
