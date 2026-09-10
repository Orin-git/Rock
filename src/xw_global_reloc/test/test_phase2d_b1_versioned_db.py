#!/usr/bin/env python3
"""Phase2D-B1 tests: migration, pointer, reload, isolation, equivalence."""

from __future__ import annotations

import hashlib
import json
import math
import os
import shutil
import sys
import tempfile
import unittest
from pathlib import Path
from typing import Any, Dict, List

import numpy as np
import yaml

_SRC = Path(__file__).resolve().parents[1]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from xw_global_reloc.phase2d.version_store import (  # noqa: E402
    atomic_set_pointer,
    inventory_keyframe_hashes,
    load_visual_db_from_root,
    migrate_legacy_to_v1,
    read_pointer_version,
    resolve_active_root,
    validate_version_for_activate,
    version_dir,
    visual_root,
)
from xw_global_reloc.retrieval import retrieve_topk  # noqa: E402


PROD_MAPS = Path('/ros2_ws/maps')
if not PROD_MAPS.is_dir():
    PROD_MAPS = Path('/home/radxa/ros2_ws/maps')


def _sha(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


class TestResolveAndPointer(unittest.TestCase):
    def test_atomic_pointer_switch(self) -> None:
        tmp = Path(tempfile.mkdtemp(prefix='vptr_'))
        try:
            vroot = tmp / 'visual'
            for ver in ('vp_visual_v1.0', 'vp_visual_v1.0_testcopy'):
                d = version_dir(vroot, ver)
                d.mkdir(parents=True)
                (d / 'manifest.yaml').write_text(
                    yaml.safe_dump({'version': ver, 'map_hash': 'h', 'map_name': 'vp'}),
                    encoding='utf-8',
                )
                (d / 'descriptors').mkdir()
                (d / 'descriptors' / 'index.json').write_text('[]\n', encoding='utf-8')
                (d / 'keyframes').mkdir()
            atomic_set_pointer(vroot, 'vp_visual_v1.0')
            self.assertEqual(read_pointer_version(vroot), 'vp_visual_v1.0')
            atomic_set_pointer(vroot, 'vp_visual_v1.0_testcopy')
            self.assertEqual(read_pointer_version(vroot), 'vp_visual_v1.0_testcopy')
            root, ver, src = resolve_active_root(tmp, 'x', db_root_override='')
            # map_name unused when visual under tmp/x/visual — fix structure
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_resolve_prefers_pointer(self) -> None:
        tmp = Path(tempfile.mkdtemp(prefix='vres_'))
        try:
            maps = tmp
            vroot = visual_root(maps, 'vp')
            # legacy layout
            (vroot / 'keyframes').mkdir(parents=True)
            (vroot / 'descriptors').mkdir()
            (vroot / 'manifest.yaml').write_text('map_name: vp\nmap_hash: abc\n', encoding='utf-8')
            (vroot / 'descriptors' / 'index.json').write_text('[]\n', encoding='utf-8')
            # versioned
            ver = version_dir(vroot, 'vp_visual_v1.0')
            ver.mkdir(parents=True)
            (ver / 'manifest.yaml').write_text(
                'version: vp_visual_v1.0\nmap_name: vp\nmap_hash: abc\n', encoding='utf-8'
            )
            (ver / 'descriptors').mkdir()
            (ver / 'descriptors' / 'index.json').write_text('[]\n', encoding='utf-8')
            (ver / 'keyframes').mkdir()
            atomic_set_pointer(vroot, 'vp_visual_v1.0')
            root, version, source = resolve_active_root(maps, 'vp')
            self.assertEqual(source, 'current_active_version')
            self.assertEqual(version, 'vp_visual_v1.0')
            self.assertEqual(root.resolve(), ver.resolve())
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


@unittest.skipUnless((PROD_MAPS / 'vp' / 'visual' / 'manifest.yaml').is_file(), 'no production visual db')
class TestMigrationEquivalence(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.maps = PROD_MAPS
        cls.vroot = visual_root(cls.maps, 'vp')
        cls.pre_manifest = _sha(cls.vroot / 'manifest.yaml')
        cls.pre_index = _sha(cls.vroot / 'descriptors' / 'index.json')
        cls.pre_inv = inventory_keyframe_hashes(cls.vroot)

    def test_migrate_and_equivalence(self) -> None:
        result = migrate_legacy_to_v1(self.maps, 'vp', force=False)
        self.assertTrue(result.ok, msg=result.message)
        self.assertTrue(result.equivalence_ok, msg='keyframe bytes must match')
        self.assertEqual(result.active_version, 'vp_visual_v1.0')
        # Original production files untouched
        self.assertEqual(_sha(self.vroot / 'manifest.yaml'), self.pre_manifest)
        self.assertEqual(_sha(self.vroot / 'descriptors' / 'index.json'), self.pre_index)
        self.assertEqual(self.pre_inv['keyframe_count'], 37)
        ver = version_dir(self.vroot, 'vp_visual_v1.0')
        post = inventory_keyframe_hashes(ver)
        self.assertEqual(post['keyframe_count'], 37)
        self.assertEqual(post['index_sha256'], self.pre_inv['index_sha256'])
        for kid, pe in self.pre_inv['keyframes'].items():
            self.assertEqual(pe['files'], post['keyframes'][kid]['files'])
        # pointer
        self.assertEqual(read_pointer_version(self.vroot), 'vp_visual_v1.0')
        # legacy seed present
        self.assertTrue((self.vroot / 'legacy_seed' / 'manifest.yaml').is_file())
        # coverage + validation sidecars
        self.assertTrue((ver / 'coverage.json').is_file())
        self.assertTrue((ver / 'validation_report.json').is_file())
        man = yaml.safe_load((ver / 'manifest.yaml').read_text(encoding='utf-8'))
        self.assertEqual(man.get('version'), 'vp_visual_v1.0')
        self.assertEqual(int(man.get('schema_version')), 3)
        self.assertEqual(int(man.get('keyframe_count')), 37)

    def test_retrieval_equivalence_legacy_vs_v1(self) -> None:
        # Ensure migrated
        migrate_legacy_to_v1(self.maps, 'vp', force=False)
        legacy_root = self.vroot  # original production root still has keyframes
        v1 = version_dir(self.vroot, 'vp_visual_v1.0')
        a = load_visual_db_from_root(legacy_root)
        b = load_visual_db_from_root(v1)
        self.assertFalse(a.error)
        self.assertFalse(b.error)
        self.assertEqual(sorted(a.kf_by_id), sorted(b.kf_by_id))
        # Same query RGB from kf_000002
        import cv2

        q = cv2.imread(str(legacy_root / 'keyframes' / 'kf_000002' / 'rgb.jpg'))
        self.assertIsNotNone(q)
        ra = retrieve_topk(q, a.keyframes, top_k=5)
        rb = retrieve_topk(q, b.keyframes, top_k=5)
        ids_a = [c.keyframe_id for c in ra.candidates]
        ids_b = [c.keyframe_id for c in rb.candidates]
        self.assertEqual(ids_a, ids_b)
        for ca, cb in zip(ra.candidates, rb.candidates):
            self.assertAlmostEqual(ca.score, cb.score, places=9)

    def test_candidate_isolation(self) -> None:
        migrate_legacy_to_v1(self.maps, 'vp', force=False)
        cand_root = self.vroot / 'candidate' / 'keyframes' / 'cand_B1_ISOLATION_TEST'
        cand_root.mkdir(parents=True, exist_ok=True)
        # Copy descriptors from kf_000001 so it would match strongly if loaded
        src = self.vroot / 'keyframes' / 'kf_000001'
        for name in ('rgb.jpg', 'descriptors.npy', 'keypoints.npy'):
            shutil.copy2(src / name, cand_root / name)
        meta = {
            'keyframe_id': 'cand_B1_ISOLATION_TEST',
            'lifecycle': 'CANDIDATE',
            'retrieval_ready': True,
            'x': 0.0,
            'y': 0.0,
            'yaw': 0.0,
        }
        (cand_root / 'meta.yaml').write_text(yaml.safe_dump(meta), encoding='utf-8')
        # Also plant a trap index next to Active? Reloc must not scan candidate/
        active = load_visual_db_from_root(version_dir(self.vroot, 'vp_visual_v1.0'))
        ids = [k['id'] for k in active.keyframes]
        self.assertNotIn('cand_B1_ISOLATION_TEST', ids)
        # cleanup test artifact
        shutil.rmtree(cand_root, ignore_errors=True)

    def test_bad_version_validate(self) -> None:
        migrate_legacy_to_v1(self.maps, 'vp', force=False)
        vroot = self.vroot
        bad = version_dir(vroot, 'vp_visual_bad_hash')
        if bad.exists():
            shutil.rmtree(bad)
        shutil.copytree(version_dir(vroot, 'vp_visual_v1.0'), bad)
        man = yaml.safe_load((bad / 'manifest.yaml').read_text(encoding='utf-8'))
        man['map_hash'] = 'deadbeef' * 8
        man['version'] = 'vp_visual_bad_hash'
        (bad / 'manifest.yaml').write_text(yaml.safe_dump(man), encoding='utf-8')
        ok, reason = validate_version_for_activate(
            vroot, 'vp_visual_bad_hash', maps_dir=self.maps, map_name='vp'
        )
        self.assertFalse(ok)
        self.assertEqual(reason, 'MAP_HASH_MISMATCH')
        # broken index
        broken = version_dir(vroot, 'vp_visual_broken_index')
        if broken.exists():
            shutil.rmtree(broken)
        shutil.copytree(version_dir(vroot, 'vp_visual_v1.0'), broken)
        (broken / 'descriptors' / 'index.json').write_text('{not-json', encoding='utf-8')
        loaded = load_visual_db_from_root(broken)
        self.assertEqual(loaded.error, 'INDEX_INVALID')
        # cleanup
        shutil.rmtree(bad, ignore_errors=True)
        shutil.rmtree(broken, ignore_errors=True)

    def test_rollback_pointer(self) -> None:
        migrate_legacy_to_v1(self.maps, 'vp', force=False)
        vroot = self.vroot
        copy = version_dir(vroot, 'vp_visual_v1.0_testcopy')
        if copy.exists():
            shutil.rmtree(copy)
        shutil.copytree(version_dir(vroot, 'vp_visual_v1.0'), copy)
        man = yaml.safe_load((copy / 'manifest.yaml').read_text(encoding='utf-8'))
        man['version'] = 'vp_visual_v1.0_testcopy'
        (copy / 'manifest.yaml').write_text(yaml.safe_dump(man), encoding='utf-8')
        atomic_set_pointer(vroot, 'vp_visual_v1.0_testcopy')
        self.assertEqual(read_pointer_version(vroot), 'vp_visual_v1.0_testcopy')
        atomic_set_pointer(vroot, 'vp_visual_v1.0')
        self.assertEqual(read_pointer_version(vroot), 'vp_visual_v1.0')
        # keep testcopy for optional live reload tests; remove to avoid clutter
        shutil.rmtree(copy, ignore_errors=True)


class TestReloadBusyLogic(unittest.TestCase):
    def test_load_fails_do_not_clear_concept(self) -> None:
        """Simulate: keep old lists if new load errors (unit-level)."""
        old = [{'id': 'kf_000001'}]
        loaded = load_visual_db_from_root(Path('/tmp/does_not_exist_visual_db_xyz'))
        self.assertTrue(loaded.error)
        # caller must not assign on error — old remains
        new = old if loaded.error else loaded.keyframes
        self.assertEqual(new, old)


if __name__ == '__main__':
    unittest.main()
