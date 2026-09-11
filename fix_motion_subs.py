#!/usr/bin/env python3
"""Make _disarm_motion_subs() a state toggle instead of a teardown.

`_disarm_motion_subs` destroys the amcl/odom subscriptions. It is called from
two threads that are NOT the ones taking from those subscriptions' wait set:

  - :500  stop_build(), which runs on the executor thread via _on_build_cmd
  - :982  the build worker's `finally`

rclpy does not synchronise entity destruction against a spinning executor, so
this is the same race that killed capture_candidate_node with
    InvalidHandle: cannot use Destroyable because destruction was requested
and lost_recovery_node twice. The fix is the one already proven on the capture
node: create the subscriptions once, never destroy them, and gate the callbacks
on a flag so a disarmed node retains no payload.

Behaviour is preserved: `_amcl`/`_odom` are still cleared on disarm, and the two
readers (:597 seed, :1270 twist) already guard with `is not None`.

Pure text surgery: every old string must appear exactly once, else nothing is
written. The target file is given as argv[1]; edited in place so the hardlink
from build/ to src/ is preserved (a fresh inode would leave the imported copy
stale).
"""
import sys
from pathlib import Path

OLD_INIT = """        self._amcl_sub = None
        self._odom_sub = None
"""

NEW_INIT = """        self._amcl_sub = None
        self._odom_sub = None
        # Motion subscriptions are created once and never destroyed. `_disarm`
        # only stops retaining payloads, because destroying from the executor
        # thread races the executor's own take (see _disarm_motion_subs).
        self._motion_armed = False
"""

OLD_CB = """    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self._amcl = msg

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg
"""

NEW_CB = """    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        if not self._motion_armed:
            return
        self._amcl = msg

    def _on_odom(self, msg: Odometry) -> None:
        if not self._motion_armed:
            return
        self._odom = msg
"""

OLD_ARM = """    def _arm_motion_subs(self) -> None:
        if self._amcl_sub is None:
"""

NEW_ARM = """    def _arm_motion_subs(self) -> None:
        # Accept payloads BEFORE checking the subscriptions exist: this runs on
        # the executor thread and the executor may deliver into them the moment
        # they are created. Setting this afterwards would drop the first message
        # of every arm.
        self._motion_armed = True
        if self._amcl_sub is None:
"""

OLD_DISARM = """    def _disarm_motion_subs(self) -> None:
        for attr in ('_amcl_sub', '_odom_sub'):
            sub = getattr(self, attr)
            if sub is not None:
                try:
                    self.destroy_subscription(sub)
                except Exception:  # noqa: BLE001
                    pass
                setattr(self, attr, None)
        self._amcl = None
        self._odom = None
"""

NEW_DISARM = """    def _disarm_motion_subs(self) -> None:
        # NO destroy_subscription here. This is called from stop_build() on the
        # executor thread (:500) and from the worker's `finally` (:982); tearing
        # an entity down while the executor is taking from it is what killed
        # capture_candidate_node (InvalidHandle) and lost_recovery_node twice.
        # Disarm is therefore a state toggle: stop retaining, keep the entity.
        self._motion_armed = False
        self._amcl = None
        self._odom = None
"""


def main() -> int:
    path = Path(sys.argv[1])
    src = path.read_text(encoding='utf-8')

    for name, needle in (('OLD_INIT', OLD_INIT), ('OLD_CB', OLD_CB),
                         ('OLD_ARM', OLD_ARM), ('OLD_DISARM', OLD_DISARM)):
        n = src.count(needle)
        if n != 1:
            print(f'ABORT: {name} occurs {n} times, expected exactly 1')
            return 1

    out = (src.replace(OLD_INIT, NEW_INIT)
              .replace(OLD_CB, NEW_CB)
              .replace(OLD_ARM, NEW_ARM)
              .replace(OLD_DISARM, NEW_DISARM))

    # Post-checks: the disarm body must no longer destroy or null the handles.
    # Match the CALL, not the bare word — the new comment mentions
    # destroy_subscription by name while explaining why it is gone.
    region = out[out.index('def _disarm_motion_subs'):out.index('def stop_build')]
    if 'self.destroy_subscription(' in region:
        print('ABORT: destroy_subscription call still present in _disarm_motion_subs')
        return 1
    if "_amcl_sub" in region or "_odom_sub" in region:
        print('ABORT: _disarm_motion_subs still touches the subscription handles')
        return 1
    if out.count('self._motion_armed = True') != 1:
        print('ABORT: _motion_armed is not set exactly once')
        return 1
    if out.count('if not self._motion_armed:') != 2:
        print('ABORT: expected exactly 2 callback guards')
        return 1

    path.write_text(out, encoding='utf-8')
    print(f'OK: patched {path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
