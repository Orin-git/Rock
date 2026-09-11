#!/usr/bin/env python3
"""Split status_dict() so the build_busy branch cannot self-deadlock.

`self._lock` is a plain threading.Lock (deliberately NOT RLock). start_build's
build_busy branch calls status_dict() while already holding it -> nested acquire
-> the node wedges forever and silently ignores every later command.

Pure text surgery: every old string must appear exactly once, else nothing is
written. Target file is given as argv[1]; operates in place after writing a
.bak_<date> copy.
"""
import sys
from pathlib import Path

OLD_HEAD = """    def status_dict(self) -> Dict[str, Any]:
        with self._lock:
            if self._session is None:
                return {'state': 'IDLE', 'build_session_id': None}
            d = self._session.to_dict()
"""

NEW_HEAD = '''    def status_dict(self) -> Dict[str, Any]:
        with self._lock:
            return self._status_dict_locked()

    def _status_dict_locked(self) -> Dict[str, Any]:
        """Build the status payload. The caller MUST already hold `self._lock`.

        `_lock` is a plain (non-reentrant) threading.Lock, so this must never
        acquire it. start_build's build_busy branch calls this while already
        holding the lock; when that branch called the public status_dict()
        instead, the nested acquire self-deadlocked the whole node permanently
        and every later command was silently ignored.
        """
        if self._session is None:
            return {'state': 'IDLE', 'build_session_id': None}
        d = self._session.to_dict()
'''

OLD_TAIL = """            return d

    def _on_status_svc"""

NEW_TAIL = """        return d

    def _on_status_svc"""

OLD_CALL = "                return {'ok': False, 'message': 'build_busy', **self.status_dict()}"
NEW_CALL = "                return {'ok': False, 'message': 'build_busy', **self._status_dict_locked()}"


def main() -> int:
    path = Path(sys.argv[1])
    src = path.read_text(encoding='utf-8')

    for name, needle in (('OLD_HEAD', OLD_HEAD), ('OLD_TAIL', OLD_TAIL), ('OLD_CALL', OLD_CALL)):
        n = src.count(needle)
        if n != 1:
            print(f'ABORT: {name} occurs {n} times, expected exactly 1')
            return 1

    # De-indent the remaining body (everything between NEW_HEAD's end and OLD_TAIL)
    # by 4 spaces: it moves out of the `with self._lock:` block.
    head_i = src.index(OLD_HEAD)
    body_start = head_i + len(OLD_HEAD)
    tail_i = src.index(OLD_TAIL, body_start)
    body = src[body_start:tail_i]

    dedented_lines = []
    for line in body.split('\n'):
        if line.startswith('    '):
            dedented_lines.append(line[4:])
        elif line.strip() == '':
            dedented_lines.append('')
        else:
            print(f'ABORT: body line not indented as expected: {line!r}')
            return 1
    body_new = '\n'.join(dedented_lines)

    out = src[:head_i] + NEW_HEAD + body_new + NEW_TAIL + src[tail_i + len(OLD_TAIL):]
    out = out.replace(OLD_CALL, NEW_CALL)

    if out.count(NEW_CALL) != 1 or out.count('def _status_dict_locked') != 1:
        print('ABORT: post-check failed')
        return 1
    if 'self.status_dict()' in out[out.index('def _status_dict_locked'):out.index('def _on_status_svc')]:
        print('ABORT: status_dict still referenced inside the split region')
        return 1

    path.write_text(out, encoding='utf-8')
    print(f'OK: patched {path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
