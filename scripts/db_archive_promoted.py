#!/usr/bin/env python3
"""Archive the candidates a version has already absorbed -- MOVE, never delete.

WHAT THIS IS
------------
The candidate tree only ever grows.  Every promotion copies a candidate into a
version tree (`c1_validate_promote.py:817 shutil.copytree(active_root, dest,
symlinks=False)`) and then leaves the candidate directory sitting there forever,
carrying `promoted_as` / `version_promoted_to` in its own `meta.yaml`.  On
2026-09-22 that was 69 of 143 candidate directories, 6.94 MiB of the 14.53 MiB
candidate tree -- already-loaded content paying rent twice.

This script moves those directories under
`<root>/archive/promoted_<stamp>/candidate/keyframes/<id>` and records, verbatim,
what it moved and what it removed from the candidate index, so `--restore` can
put it all back byte-for-byte.  Nothing is deleted; nothing is unlinked.

WHY DEFAULT IS DRY-RUN
----------------------
`--apply` is a separate, deliberate act.  Run it with no flags first, read the
list, and check it against db_maintenance_report.py's independently-produced
`absorbed n=...` line.  Two scripts, two code paths, one number.

THE PREDICATE (duplicated on purpose)
-------------------------------------
  absorbed  := (promoted_as or version_promoted_to)      <- clause 1
               AND promoted_as is in the ACTIVE version's descriptors/index.json
                                                         <- clause 2

Clause 1 is duplicated from db_maintenance_report.py:_absorbed, which says so in
its own comment.  That is a deliberate maintenance contract between two scripts
that move in opposite directions: the report only looks, this one moves files,
and a behaviour change must not flow from one to the other by import.  If you
change it here, change it there too, and re-check both against
c1_validate_promote.py:_candidate_already_decided.

Clause 2 makes the archive collapse-proof: it refuses to move anything that the
CURRENT pointer does not cover.  Measured 69/69 on 2026-09-22 -- vacuous while
versions only move forward, restrictive the moment the pointer goes backwards.

THE INDEX RECONCILIATION IS NOT OPTIONAL
----------------------------------------
`c1_validate_promote.py:128-146` (load_candidates) reads `candidate/index.json`
WHEN IT EXISTS and only globs `keyframes/cand_*` as a fallback.  A directory that
is gone while its index entry remains therefore does not vanish -- it becomes a
record:

    CandRecord(kid, kdir, {}, 'REJECTED_INVALID', ['missing_meta'])

Moving 69 directories without pruning the index would create 69 phantom records.
So this script removes the 69 matching entries in the same transaction and
records each one verbatim, with its position, in MANIFEST.json.

THE WRITER CANNOT BE LOCKED OUT FROM HERE
-----------------------------------------
`candidate_writer.py:141` reads the whole index, `:159` writes the whole index:

    141:  index = self._load_index()
    158:  self.assert_not_production(self.index_path)
    159:  self.index_path.write_text(json.dumps(index, indent=2) + '\\n', ...)

There is no temp-and-rename and no cross-process lock -- only an in-process
threading.Lock.  A candidate writer (`xw_visual_db_capture`) is normally alive.
So this script does not pretend to hold a lock.  Instead it measures:

  * QUIET CHECK   the index md5 before and after its own full scan must be equal.
                  The window is however long the scan takes -- a measured
                  interval, not a threshold this script invented.
  * CAS           just before writing, re-read the index md5.  If it moved since
                  the scan, abort and move everything back.  Nothing written.
  * CONTENT CHECK re-read and parse after writing.  If any archived id is still
                  in the index, a writer wrote stale content over the pruned
                  version; move everything back and abort.

An abort NEVER writes the index a second time.  The abort means a writer is
active, and a second write is the exact race the abort exists to avoid.  If the
check finds an archived id missing from the index afterwards, that is precisely
the case for stopping the capture node and running `--restore <stamp>`.

CONSEQUENCE OF A CRASH MID-MOVE (recorded, not hidden)
------------------------------------------------------
The order is move-then-index.  A crash between the two leaves 69 directories in
`archive/` with the index still listing them -- visible in db_maintenance_report.py
as `index.json entries != keyframe dirs`, and repaired by `--restore <stamp>`.
The reverse order would instead hide 69 candidates from the promote path with no
phantom record to show for it.

WHAT --apply REFUSES (all before it touches anything)
-----------------------------------------------------
  G1  candidate/index.json is a regular file, not a symlink, and is not the same
      file as either the root descriptors index or the active version's index
  G2  the newest state/build_*.json records an end_time (the last known session
      finished).  This is NOT proof that no session is running -- a live session's
      JSON is only written on exit -- so it is a barrier plus the three measured
      checks above, not a lock.
  G3  the archive folder for this stamp does not already exist
  G4  every selected directory exists, is a real directory, and has a readable
      meta.yaml
  G5  candidate/keyframes and the archive parent are on the same device, so
      os.rename is atomic and cheap

NOT DONE (by construction)
--------------------------
  nothing is deleted, unlinked or truncated; no version tree is touched; no
  manifest is rewritten; no threshold is invented; the active pointer is never
  read-modify-written.  db_maintenance_report.py is NOT edited -- its AREAS
  tuple does not include 'archive', so after an --apply it will not see the
  archived bytes.  That is a known, reported gap, not a bug to fix by editing a
  script whose md5 is already on record.

Exit codes: 0 ok / 2 precondition / 3 guard / 4 rolled back.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

DEFAULT_ROOT = '/ros2_ws/maps/vp/visual'
POINTER_NAME = 'current_active_version'
CANDIDATE_DIR = 'candidate'
KEYFRAMES_DIR = 'keyframes'
VERSIONS_DIR = 'versions'
STATE_DIR = 'state'
ARCHIVE_DIR = 'archive'
INDEX_NAME = 'index.json'
META_NAME = 'meta.yaml'
DESCRIPTORS_INDEX = ('descriptors', 'index.json')
MANIFEST_NAME = 'MANIFEST.json'


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------

def _sha(path: str) -> Optional[str]:
    """md5 of a file, or None if it cannot be read.  Chunked: never load a
    whole tree into memory to hash it."""
    try:
        h = hashlib.md5()
        with open(path, 'rb') as fh:
            for chunk in iter(lambda: fh.read(1 << 20), b''):
                h.update(chunk)
        return h.hexdigest()
    except OSError:
        return None


def _ident(path: str) -> Dict[str, Any]:
    """The identity triple.  mtime is deliberately absent -- shutil.copystat
    carries the ORIGINAL mtime forward, so mtime is not evidence of anything."""
    try:
        st = os.stat(path)
    except OSError:
        return {'exists': False}
    return {'exists': True, 'md5': _sha(path), 'size': st.st_size,
            'inode': st.st_ino, 'islink': os.path.islink(path)}


def _iso(ts: Optional[float]) -> str:
    if not ts:
        return '-'
    return time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime(ts)) + 'Z'


def _mib(n: int) -> str:
    return f'{n / 1048576.0:.2f}'


def _read_yaml(path: str) -> Optional[Dict[str, Any]]:
    """A report -- or an archiver -- that dies on one bad file reports nothing
    at all.  Returns None and never raises."""
    try:
        import yaml
    except ImportError:
        return None
    try:
        with open(path, encoding='utf-8') as fh:
            data = yaml.safe_load(fh)
    except (OSError, ValueError):
        return None
    return data if isinstance(data, dict) else None


def _read_json(path: str) -> Optional[Any]:
    try:
        with open(path, encoding='utf-8') as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return None


def _walk_stats(path: str) -> Tuple[int, int]:
    """(bytes, files).  followlinks=False: current_active_version is a symlink
    into versions/ and must not be counted twice."""
    total = files = 0
    for dirpath, _dirnames, filenames in os.walk(path, followlinks=False):
        for name in filenames:
            try:
                total += os.stat(os.path.join(dirpath, name)).st_size
            except OSError:
                continue
            files += 1
    return total, files


def _ids_of(index_data: Any) -> List[str]:
    """The id of an index entry, however that entry is shaped.  c1 and the
    report both accept a bare string or a dict with 'id'."""
    out: List[str] = []
    if isinstance(index_data, list):
        for it in index_data:
            if isinstance(it, dict):
                if it.get('id') is not None:
                    out.append(str(it['id']))
            elif it is not None:
                out.append(str(it))
    return out


def _write_index_atomic(path: str, entries: List[Any]) -> None:
    """Sibling temp name + copystat + os.replace.  os.replace across devices
    raises Errno 18, so the temp MUST be a sibling.  Byte-identical in shape to
    candidate_writer.py:159 so a naive diff of the file stays readable."""
    data = json.dumps(entries, indent=2) + '\n'
    tmp = path + '.d2ctmp'
    with open(tmp, 'w', encoding='utf-8') as fh:
        fh.write(data)
    try:
        shutil.copystat(path, tmp)
    except OSError:
        pass
    os.replace(tmp, path)


# --------------------------------------------------------------------------
# the promote path's own "already decided" test, reproduced (not invented)
# --------------------------------------------------------------------------
# NOTE: this tiny predicate is duplicated in db_maintenance_report.py. That is
# deliberate -- the archiver moves files and must not inherit a behaviour change
# from a future edit to a read-only report. If you change it here, change it
# there too, and re-check both against
# c1_validate_promote.py:_candidate_already_decided.

def _absorbed(meta: Dict[str, Any]) -> bool:
    """c1_validate_promote.py:_candidate_already_decided, first clause.
    Kept deliberately narrow: this is the 'copied into a version tree' test."""
    return bool(meta.get('promoted_as') or meta.get('version_promoted_to'))


def _active_version(root: str) -> Tuple[Optional[str], str]:
    """readlink gives a PATH; the name stored in meta is bare.  Comparing a
    path against a bare name is a mistake already made once in recon (it
    reported all 69 promoted candidates as 'other version')."""
    ptr = os.path.join(root, POINTER_NAME)
    if not os.path.islink(ptr):
        return None, 'no-pointer'
    try:
        return os.path.basename(os.readlink(ptr)), 'ok'
    except OSError:
        return None, 'unreadable-pointer'


def _active_index_path(root: str, active: Optional[str]) -> Optional[str]:
    if not active:
        return None
    return os.path.join(root, VERSIONS_DIR, active, *DESCRIPTORS_INDEX)


def _last_session(root: str) -> Dict[str, Any]:
    """Newest state/build_*.json, by mtime.  Reported, not trusted as a lock."""
    sdir = os.path.join(root, STATE_DIR)
    try:
        names = [n for n in os.listdir(sdir) if n.startswith('build_') and n.endswith('.json')]
    except OSError:
        return {'found': False, 'reason': 'no state dir'}
    if not names:
        return {'found': False, 'reason': 'no state/build_*.json'}
    newest = max(names, key=lambda n: os.stat(os.path.join(sdir, n)).st_mtime)
    path = os.path.join(sdir, newest)
    data = _read_json(path)
    if not isinstance(data, dict):
        return {'found': True, 'path': path, 'readable': False, 'end_time': None}
    end = data.get('end_time')
    return {'found': True, 'path': path, 'readable': True,
            'build_session_id': data.get('build_session_id'),
            'state': data.get('state'), 'stop_reason': data.get('stop_reason'),
            'start_time': data.get('start_time'), 'end_time': end,
            'finished': bool(end), 'mtime': os.stat(path).st_mtime}


# --------------------------------------------------------------------------
# the scan (read-only; this is also the thing the quiet check brackets)
# --------------------------------------------------------------------------

def _scan(root: str) -> Dict[str, Any]:
    a: List[str] = []
    add = a.append
    t0 = time.time()

    idx_path = os.path.join(root, CANDIDATE_DIR, INDEX_NAME)
    index_ident_start = _ident(idx_path)
    root_idx = os.path.join(root, *DESCRIPTORS_INDEX)
    active, how = _active_version(root)
    act_idx_path = _active_index_path(root, active)
    act_ident = _ident(act_idx_path) if act_idx_path else {'exists': False}
    active_ids = _ids_of(_read_json(act_idx_path)) if act_idx_path else []
    active_id_set = set(active_ids)

    kf_root = os.path.join(root, CANDIDATE_DIR, KEYFRAMES_DIR)
    try:
        dirs = sorted(d for d in os.listdir(kf_root) if os.path.isdir(os.path.join(kf_root, d)))
    except OSError:
        dirs = []

    table = _read_json(idx_path)
    index_entries = table if isinstance(table, list) else []
    index_entry_ids = _ids_of(index_entries)
    pos_of: Dict[str, int] = {}
    for i, eid in enumerate(index_entry_ids):
        pos_of.setdefault(eid, i)

    selected: List[Dict[str, Any]] = []
    clause1_only: List[str] = []
    clause2_only: List[str] = []
    unreadable: List[str] = []
    n_absorbed = 0
    for kid in dirs:
        kdir = os.path.join(kf_root, kid)
        meta = _read_yaml(os.path.join(kdir, META_NAME))
        if meta is None:
            unreadable.append(kid)
            continue
        c1 = _absorbed(meta)
        promoted_as = str(meta.get('promoted_as') or '')
        c2 = promoted_as in active_id_set
        if c1:
            n_absorbed += 1
        if c1 and not c2:
            clause1_only.append(kid)
        if c2 and not c1:
            clause2_only.append(kid)
        if c1 and c2:
            nbytes, nfiles = _walk_stats(kdir)
            j = pos_of.get(kid)
            selected.append({
                'id': kid,
                'src': kdir,
                'promoted_as': promoted_as,
                'version_promoted_to': meta.get('version_promoted_to'),
                'bytes': nbytes,
                'files': nfiles,
                'in_index': kid in pos_of,
                'index_pos': j,
                'index_entry': index_entries[j] if j is not None else None,
            })

    # index entries with no directory on disk.  On a clean tree this is 0; a
    # non-zero value means a previous run was interrupted, which changes how
    # --restore should be read.
    pre_phantoms = [eid for eid in index_entry_ids
                    if not os.path.isdir(os.path.join(kf_root, eid))]

    tot_bytes, tot_files = _walk_stats(root)
    index_ident_end = _ident(idx_path)

    add('active version          : %s (%s)' % (active, how))
    add('active index ids        : %d' % len(active_ids))
    add('candidate dirs on disk  : %d' % len(dirs))
    add('candidate index entries : %d' % len(index_entries))
    add('unreadable meta.yaml    : %d' % len(unreadable))
    add('absorbed (clause 1)     : %d' % n_absorbed)
    add('clause 1 only (not in active) : %d' % len(clause1_only))
    add('clause 2 only (no promote)    : %d' % len(clause2_only))
    add('SELECTED (1 and 2)      : %d' % len(selected))
    if clause2_only:
        add('  note: clause-2-only ids have promoted_as in the active index but no '
            'other promote mark; they are NOT moved (clause 1 is the '
            '"copied into a version tree" test)')

    return {
        'root': root,
        'active': active, 'active_how': how,
        'active_index_path': act_idx_path, 'active_index_ident': act_ident,
        'active_ids': active_ids,
        'index_path': idx_path,
        'index_ident_start': index_ident_start,
        'index_ident_end': index_ident_end,
        'index_entries': index_entries,
        'index_entry_ids': index_entry_ids,
        'index_stable': (index_ident_start.get('md5') == index_ident_end.get('md5')),
        'scan_secs': round(time.time() - t0, 3),
        'candidate_dirs': len(dirs),
        'unreadable': unreadable,
        'clause1_only': clause1_only,
        'clause2_only': clause2_only,
        'selected': selected,
        'selected_ids': [s['id'] for s in selected],
        'selected_bytes': sum(s['bytes'] for s in selected),
        'selected_files': sum(s['files'] for s in selected),
        'pre_phantoms': pre_phantoms,
        'root_index_ident': _ident(root_idx),
        'total_bytes': tot_bytes, 'total_files': tot_files,
        'versions_entries': _n_versions(root),
    }


def _n_versions(root: str) -> int:
    try:
        return len([n for n in os.listdir(os.path.join(root, VERSIONS_DIR))
                    if os.path.isdir(os.path.join(root, VERSIONS_DIR, n))])
    except OSError:
        return -1


# --------------------------------------------------------------------------
# guards
# --------------------------------------------------------------------------

def _guards(scan: Dict[str, Any]) -> List[str]:
    """Every reason --apply must not start.  All of them are checked before the
    first mkdir.  Returns [] when clear."""
    bad: List[str] = []
    root = scan['root']
    idx_path = scan['index_path']
    ident = scan['index_ident_start']

    if not scan['selected']:
        bad.append('G0 nothing selected -- nothing to archive')

    if scan['active_how'] != 'ok' or not scan['active']:
        bad.append('G1 current_active_version is not a readable symlink (%s)' % scan['active_how'])

    if not ident.get('exists'):
        bad.append('G1 candidate/%s does not exist' % INDEX_NAME)
    else:
        if ident.get('islink'):
            bad.append('G1 candidate/%s is a symlink; refusing to rewrite a link target'
                       % INDEX_NAME)
        for other, label in ((os.path.join(root, *DESCRIPTORS_INDEX), 'root descriptors index'),
                             (scan['active_index_path'] or '', "active version's index")):
            if not other:
                continue
            try:
                if os.path.exists(other) and os.path.samefile(idx_path, other):
                    bad.append('G1 candidate/%s IS the %s -- abort' % (INDEX_NAME, label))
            except OSError:
                pass

    sess = _last_session(root)
    if not sess.get('found'):
        bad.append('G2 cannot establish the last-session barrier: %s' % sess.get('reason'))
    elif not sess.get('readable'):
        bad.append('G2 newest session file is unreadable: %s' % sess.get('path'))
    elif not sess.get('finished'):
        bad.append('G2 newest session %s has NO end_time (state=%s) -- a session may be '
                   'in progress' % (sess.get('build_session_id'), sess.get('state')))

    if not scan['index_stable']:
        bad.append('G2 quiet check failed: candidate/%s changed during the %.3fs scan '
                   '(md5 %s -> %s) -- a writer is active'
                   % (INDEX_NAME, scan['scan_secs'],
                      str(scan['index_ident_start'].get('md5'))[:12],
                      str(scan['index_ident_end'].get('md5'))[:12]))

    if scan['unreadable']:
        bad.append('G4 %d candidate dir(s) have no readable %s: %s'
                   % (len(scan['unreadable']), META_NAME, ', '.join(scan['unreadable'][:5])))

    for s in scan['selected']:
        if os.path.islink(s['src']) or not os.path.isdir(s['src']):
            bad.append('G4 %s is not a real directory' % s['src'])

    kf_root = os.path.join(root, CANDIDATE_DIR, KEYFRAMES_DIR)
    try:
        dev_kf = os.stat(kf_root).st_dev
        dev_root = os.stat(root).st_dev
        if dev_kf != dev_root:
            bad.append('G5 candidate/keyframes (dev %d) and root (dev %d) are on different '
                       'devices -- os.rename would not be atomic' % (dev_kf, dev_root))
    except OSError as exc:
        bad.append('G5 cannot stat devices: %s' % exc)

    return bad


# --------------------------------------------------------------------------
# apply
# --------------------------------------------------------------------------

def _archive_dir(root: str, stamp: str) -> str:
    return os.path.join(root, ARCHIVE_DIR, stamp)


def _apply(root: str, scan: Dict[str, Any], stamp: str) -> int:
    L: List[str] = []
    a = L.append
    def bail(code: int, tail: str) -> int:
        # Every abort must SAY WHY.  The success path prints L at the end, but
        # the early returns used to build L and throw it away: an operator got
        # exit 3 with no output and no way to tell G3 from a failed rename.
        print('\n'.join(L))
        print()
        print(tail)
        return code

    rolled_back = '== ABORTED -- moved director(y/ies) rolled back, index untouched =='

    adir = _archive_dir(root, stamp)
    if os.path.exists(adir):
        a('ABORT: G3 archive folder already exists: %s' % adir)
        return bail(3, '== ABORTED -- nothing was touched ==')

    before = {
        'index': dict(scan['index_ident_start']),
        'active_index': dict(scan['active_index_ident']),
        'candidate_dirs': scan['candidate_dirs'],
        'versions_entries': scan['versions_entries'],
        'total_bytes': scan['total_bytes'],
    }

    dest_kf = os.path.join(adir, CANDIDATE_DIR, KEYFRAMES_DIR)
    os.makedirs(dest_kf, exist_ok=True)
    a('archive          : %s' % adir)

    # -- S2: move ---------------------------------------------------------
    moved: List[Dict[str, str]] = []
    for s in scan['selected']:
        dst = os.path.join(dest_kf, s['id'])
        try:
            os.rename(s['src'], dst)
        except OSError as exc:
            a('ABORT: rename failed for %s: %s' % (s['id'], exc))
            a('  rolling back %d already-moved director(y/ies)' % len(moved))
            _unmove(moved, a)
            return bail(3, rolled_back)
        moved.append({'id': s['id'], 'src': s['src'], 'dst': dst})
    a('moved            : %d director(y/ies), %s MiB, %d files'
      % (len(moved), _mib(sum(s['bytes'] for s in scan['selected'])),
         sum(s['files'] for s in scan['selected'])))

    # -- S3: CAS ----------------------------------------------------------
    now_ident = _ident(scan['index_path'])
    if now_ident.get('md5') != before['index'].get('md5'):
        a('ABORT (CAS): candidate/%s moved between the scan and the write' % INDEX_NAME)
        a('  scan=%s  now=%s' % (str(before['index'].get('md5'))[:12],
                                 str(now_ident.get('md5'))[:12]))
        a('  NOT writing the index. Moving everything back.')
        _unmove(moved, a)
        return bail(4, rolled_back)

    # -- S4: prune + write ------------------------------------------------
    sel = set(scan['selected_ids'])
    removed: List[Dict[str, Any]] = []
    kept: List[Any] = []
    for i, entry in enumerate(scan['index_entries']):
        eid = (str(entry['id']) if isinstance(entry, dict) and entry.get('id') is not None
               else (str(entry) if not isinstance(entry, dict) and entry is not None else None))
        if eid is not None and eid in sel:
            removed.append({'pos': i, 'id': eid, 'entry': entry})
        else:
            kept.append(entry)
    _write_index_atomic(scan['index_path'], kept)
    after_idx = _ident(scan['index_path'])
    a('index            : %d -> %d entries (removed %d), md5 %s -> %s'
      % (len(scan['index_entries']), len(kept), len(removed),
         str(before['index'].get('md5'))[:12], str(after_idx.get('md5'))[:12]))

    # -- S5: content check ------------------------------------------------
    reread = _read_json(scan['index_path'])
    reread_ids = set(_ids_of(reread))
    stale = sorted(sel & reread_ids)
    if stale:
        a('ABORT (content check): %d archived id(s) are STILL in the index after the write'
          % len(stale))
        a('  e.g. %s' % ', '.join(stale[:5]))
        a('  A writer wrote content it had read before this run. NOT writing again.')
        a('  Moving everything back; the index is the writer\'s and already lists them.')
        _unmove(moved, a)
        return bail(4, rolled_back)
    appended = sorted(reread_ids - set(scan['index_entry_ids']))
    if appended:
        a('writer appended during the window: %d new entr(y/ies) kept (%s)'
          % (len(appended), ', '.join(appended[:3])))

    # -- S6: manifest -----------------------------------------------------
    after = {
        'index': dict(after_idx),
        'active_index': _ident(scan['active_index_path']) if scan['active_index_path'] else {},
        'candidate_dirs': len([d for d in os.listdir(os.path.join(root, CANDIDATE_DIR,
                                                                 KEYFRAMES_DIR))
                               if os.path.isdir(os.path.join(root, CANDIDATE_DIR,
                                                             KEYFRAMES_DIR, d))]),
        'versions_entries': _n_versions(root),
        'total_bytes': _walk_stats(root)[0],
    }
    manifest = {
        'created': _iso(time.time()), 'created_unix': time.time(),
        'root': root, 'stamp': stamp,
        'active_version': scan['active'],
        'predicate': 'promoted_as or version_promoted_to, AND promoted_as in the active index',
        'selected_count': len(scan['selected_ids']),
        'selected_ids': scan['selected_ids'],
        'selected_bytes': scan['selected_bytes'],
        'selected_files': scan['selected_files'],
        'moved': moved,
        'removed_index_entries': removed,
        'appended_index_entries': appended,
        'before': before, 'after': after,
        'pre_phantoms': scan['pre_phantoms'],
        'quiet_scan_secs': scan['scan_secs'],
    }
    mpath = os.path.join(adir, MANIFEST_NAME)
    with open(mpath + '.d2ctmp', 'w', encoding='utf-8') as fh:
        json.dump(manifest, fh, indent=2)
        fh.write('\n')
    os.replace(mpath + '.d2ctmp', mpath)
    a('manifest         : %s' % mpath)

    # -- S7: verify -------------------------------------------------------
    ok = True
    exp_dirs = before['candidate_dirs'] - len(moved)
    if after['candidate_dirs'] != exp_dirs:
        a('VERIFY FAIL: candidate dirs %d, expected %d' % (after['candidate_dirs'], exp_dirs))
        ok = False
    if after['active_index'].get('md5') != before['active_index'].get('md5'):
        a('VERIFY FAIL: the ACTIVE index changed -- versions/ was touched')
        ok = False
    if after['versions_entries'] != before['versions_entries']:
        a('VERIFY FAIL: versions/ entry count %s -> %s'
          % (before['versions_entries'], after['versions_entries']))
        ok = False
    a('candidate dirs   : %d -> %d' % (before['candidate_dirs'], after['candidate_dirs']))
    a('versions/        : %d -> %d  (must be unchanged)' % (before['versions_entries'],
                                                             after['versions_entries']))
    a('active index md5 : %s (unchanged: %s)'
      % (str(after['active_index'].get('md5'))[:12],
         after['active_index'].get('md5') == before['active_index'].get('md5')))
    a('library total    : %s -> %s MiB' % (_mib(before['total_bytes']),
                                            _mib(after['total_bytes'])))

    print('\n'.join(L))
    print()
    print('== APPLIED ==' if ok else '== APPLIED WITH VERIFY FAILURES ==')
    print('  undo with:  %s --restore %s' % (os.path.basename(__file__), stamp))
    return 0 if ok else 3


def _unmove(moved: Sequence[Dict[str, str]], a) -> None:
    """Move directories back.  Never touches the index -- see the docstring:
    an abort means a writer is active, and a second write is the race."""
    for m in reversed(list(moved)):
        try:
            os.rename(m['dst'], m['src'])
        except OSError as exc:
            a('  ROLLBACK FAILED for %s: %s' % (m['id'], exc))
            a('  the directory is still at %s -- move it back by hand or run --restore'
              % m['dst'])
    a('  rolled back %d director(y/ies); index untouched' % len(moved))


# --------------------------------------------------------------------------
# restore
# --------------------------------------------------------------------------

def _restore(root: str, stamp: str) -> int:
    L: List[str] = []
    a = L.append
    adir = _archive_dir(root, stamp)
    mpath = os.path.join(adir, MANIFEST_NAME)
    man = _read_json(mpath)
    if not isinstance(man, dict):
        a('ABORT: cannot read %s' % mpath)
        return 2

    moved = man.get('moved') or []
    removed = man.get('removed_index_entries') or []
    sel = set(man.get('selected_ids') or [])
    a('manifest         : %s' % mpath)
    a('archive stamp    : %s  (%s)' % (stamp, man.get('created')))
    a('recorded         : %d moved, %d index entries removed'
      % (len(moved), len(removed)))

    bad: List[str] = []
    if not moved:
        bad.append('the manifest records nothing moved')
    for m in moved:
        if not os.path.isdir(m.get('dst', '')):
            bad.append('missing in archive: %s' % m.get('dst'))
        if os.path.exists(m.get('src', '')):
            bad.append('target already occupied: %s (refusing to overwrite)' % m.get('src'))
    idx_path = os.path.join(root, CANDIDATE_DIR, INDEX_NAME)
    cur = _read_json(idx_path)
    cur_ids = set(_ids_of(cur))
    already = sorted(sel & cur_ids)
    if already:
        bad.append('%d archived id(s) are already back in the index (%s) -- a partial '
                   'restore, or a re-capture; resolve by hand'
                   % (len(already), ', '.join(already[:5])))
    if not isinstance(cur, list):
        bad.append('candidate/%s is not a readable list' % INDEX_NAME)
    if bad:
        a('ABORT:')
        for b in bad:
            a('  - %s' % b)
        return 3

    # move back
    done: List[Dict[str, str]] = []
    for m in moved:
        try:
            os.rename(m['dst'], m['src'])
        except OSError as exc:
            a('ABORT: rename failed for %s: %s' % (m['id'], exc))
            _unmove(done, a)
            return 3
        done.append(m)
    a('moved back       : %d director(y/ies)' % len(done))

    # re-insert index entries at their recorded positions, ascending
    entries = list(cur)
    for r in sorted(removed, key=lambda r: int(r.get('pos', 0))):
        pos = int(r.get('pos', 0))
        if pos > len(entries):
            pos = len(entries)
        entries.insert(pos, r.get('entry'))
    _write_index_atomic(idx_path, entries)
    ident = _ident(idx_path)
    a('index            : %d -> %d entries, md5 now %s'
      % (len(cur), len(entries), str(ident.get('md5'))[:12]))

    # Verify against the ids this index is actually supposed to hold again, not
    # against every id that was moved.  The archiver's predicate makes the
    # difference real: clause 2 tests the ACTIVE version's descriptors index,
    # NOT candidate/index.json, so a selected directory can legitimately have
    # had no entry here at all.  It is moved, it is moved back, and it will
    # never be "present again" -- so cross-checking all of `sel` would report a
    # false alarm on exactly those and return 3 for a perfect restore.  A check
    # that cries wolf is worse than no check; the untracked ones are reported
    # separately instead.
    back = set(_ids_of(_read_json(idx_path)))
    tracked = [str(r.get('id')) for r in removed]
    missing = sorted(set(tracked) - back)
    untracked = sorted(sel - set(tracked))
    a('verify           : %d/%d archived id(s) present again: %s'
      % (len(tracked) - len(missing), len(tracked),
         'yes' if not missing else 'NO -> %s' % ', '.join(missing[:5])))
    if untracked:
        a('                   %d archived dir(s) had NO index entry to restore '
          '(clause 2 tests the ACTIVE index): %s'
          % (len(untracked), ', '.join(untracked[:5])))
    if os.path.exists(adir):
        try:
            rest = [n for n in os.listdir(adir)]
            a('archive folder   : now holds %s' % (rest or 'nothing'))
        except OSError:
            pass
    print('\n'.join(L))
    print()
    print('== RESTORED ==' if not missing else '== RESTORED WITH MISSING IDS ==')
    return 0 if not missing else 3


# --------------------------------------------------------------------------
# output
# --------------------------------------------------------------------------

def _fmt_plan(scan: Dict[str, Any], stamp: str) -> str:
    L: List[str] = []
    a = L.append
    a('== db_archive_promoted -- DRY RUN ==')
    a('root             : %s' % scan['root'])
    a('active version   : %s (%s)' % (scan['active'], scan['active_how']))
    a('active index ids : %d' % len(scan['active_ids']))
    a('scan             : %.3fs, candidate/%s stable across it: %s'
      % (scan['scan_secs'], INDEX_NAME, scan['index_stable']))
    a('')
    a('%-24s%12s' % ('', 'count'))
    a('%-24s%12d' % ('candidate dirs on disk', scan['candidate_dirs']))
    a('%-24s%12d' % ('candidate index entries', len(scan['index_entries'])))
    a('%-24s%12d' % ('absorbed (clause 1)', len(scan['clause1_only'])
                     + len(scan['selected'])))
    a('%-24s%12d' % ('  clause 1 only (kept)', len(scan['clause1_only'])))
    a('%-24s%12d' % ('SELECTED (1 AND 2)', len(scan['selected'])))
    a('%-24s%12d' % ('  of those, in the index', sum(1 for s in scan['selected']
                                                      if s['in_index'])))
    if scan['pre_phantoms']:
        a('%-24s%12d  <-- index entries with no directory (interrupted run?)'
          % ('pre-existing phantoms', len(scan['pre_phantoms'])))
    a('%-24s%12s' % ('reclaimable', _mib(scan['selected_bytes']) + ' MiB'))
    a('%-24s%12d' % ('', scan['selected_files']))
    a('%-24s%12s' % ('library total', _mib(scan['total_bytes']) + ' MiB'))
    a('')
    a('archive target   : %s' % _archive_dir(scan['root'], stamp))
    a('')
    a('== WOULD MOVE ==')
    if not scan['selected']:
        a('  (nothing)')
    for s in scan['selected']:
        a('  %-30s %8s MiB %5d files  promoted_as=%s'
          % (s['id'], _mib(s['bytes']), s['files'], s['promoted_as'] or '-'))
    if scan['clause1_only']:
        a('')
        a('== KEPT: clause 1 only (promoted, but NOT in the active index) ==')
        for kid in scan['clause1_only']:
            a('  %s' % kid)
    if scan['unreadable']:
        a('')
        a('== unreadable meta.yaml (would block --apply) ==')
        for kid in scan['unreadable']:
            a('  %s' % kid)
    a('')
    a('== WOULD REMOVE FROM candidate/%s ==' % INDEX_NAME)
    a('  %d entr(y/ies), positions recorded verbatim in %s' % (
        sum(1 for s in scan['selected'] if s['in_index']), MANIFEST_NAME))
    for s in scan['selected']:
        if s['in_index']:
            a('  pos %-6d %s' % (s['index_pos'], s['id']))
    a('')
    a('== guards for --apply (evaluated now, re-evaluated at apply time) ==')
    guards = _guards(scan)
    if guards:
        for g in guards:
            a('  BLOCK  %s' % g)
    else:
        a('  clear')
    sess = _last_session(scan['root'])
    if sess.get('found') and sess.get('readable'):
        a('  last session: %s state=%s stop=%s finished=%s (%s -> %s)'
          % (sess.get('build_session_id'), sess.get('state'), sess.get('stop_reason'),
             sess.get('finished'), _iso(sess.get('start_time')), _iso(sess.get('end_time'))))
    a('')
    a('== NOT DONE (by construction) ==')
    a('  nothing moved, nothing deleted, no index written. Re-run with --apply,')
    a('  after checking the SELECTED count against db_maintenance_report.py.')
    return '\n'.join(L)


def _plan_json(scan: Dict[str, Any], stamp: str) -> str:
    keep = {k: v for k, v in scan.items() if k not in ('index_entries', 'active_ids')}
    keep['selected'] = [{k: v for k, v in s.items() if k != 'index_entry'}
                        for s in scan['selected']]
    keep['archive_target'] = _archive_dir(scan['root'], stamp)
    keep['guards'] = _guards(scan)
    keep['last_session'] = _last_session(scan['root'])
    keep['dry_run'] = True
    return json.dumps(keep, indent=2, default=str)


# --------------------------------------------------------------------------
# entry
# --------------------------------------------------------------------------

def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        prog='db_archive_promoted.py',
        description='Archive candidates a version already absorbed. MOVE, never '
                    'delete. Default is a dry run.')
    ap.add_argument('--root', default=DEFAULT_ROOT,
                    help='visual DB root (default: %s)' % DEFAULT_ROOT)
    ap.add_argument('--apply', action='store_true',
                    help='actually move. Refuses unless every guard passes.')
    ap.add_argument('--restore', metavar='STAMP', default=None,
                    help='undo a previous --apply by its archive stamp')
    ap.add_argument('--stamp', default=None,
                    help='override the archive folder name (default: promoted_<UTC>)')
    ap.add_argument('--json', action='store_true',
                    help='emit the dry-run plan as JSON')
    args = ap.parse_args(argv)

    root = args.root
    if not os.path.isdir(root):
        print('ABORT: not a directory: %s' % root)
        return 2

    if args.restore is not None:
        if args.apply:
            print('ABORT: --restore and --apply are different directions; pick one')
            return 2
        return _restore(root, args.restore)

    stamp = args.stamp or time.strftime('promoted_%Y%m%d_%H%M%S', time.gmtime())

    scan = _scan(root)
    if not args.apply:
        print(_plan_json(scan, stamp) if args.json else _fmt_plan(scan, stamp))
        return 0

    guards = _guards(scan)
    if guards:
        print('ABORT: %d guard(s) failed -- nothing touched' % len(guards))
        for g in guards:
            print('  - %s' % g)
        return 3
    return _apply(root, scan, stamp)


if __name__ == '__main__':
    raise SystemExit(main())
