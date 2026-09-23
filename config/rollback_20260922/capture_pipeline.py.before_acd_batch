"""Pure capture pipeline: Pose → Laser → Image → Policy/Dedup → Candidate write."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

import numpy as np
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

from xw_global_reloc.laser_verify import DistanceField
from xw_global_reloc.phase2d.candidate_writer import CandidateWriter
from xw_global_reloc.phase2d.capture_policy import CaptureDecision, evaluate_capture_policy
from xw_global_reloc.phase2d.coverage_model import CoverageModel, FrameRef, build_coverage_model
from xw_global_reloc.phase2d.image_quality_gate import ImageGateResult, evaluate_image_gate
from xw_global_reloc.phase2d.laser_quality_gate import LaserGateResult, evaluate_laser_gate
from xw_global_reloc.phase2d.pose_quality_gate import PoseGateInput, PoseGateResult, evaluate_pose_gate


@dataclass
class CaptureResult:
    status: str
    reason: str
    pose: Optional[PoseGateResult] = None
    laser: Optional[LaserGateResult] = None
    image: Optional[ImageGateResult] = None
    decision: Optional[CaptureDecision] = None
    written: bool = False
    dry_run: bool = False
    keyframe_id: Optional[str] = None
    path: Optional[str] = None
    meta: Dict[str, Any] = field(default_factory=dict)
    metrics: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            'status': self.status,
            'reason': self.reason,
            'written': self.written,
            'dry_run': self.dry_run,
            'keyframe_id': self.keyframe_id,
            'path': self.path,
            'metrics': self.metrics,
            'meta': self.meta,
            'capture_decision': self.decision.to_meta() if self.decision else None,
            'spatial_cell': self.decision.spatial_cell if self.decision else None,
            'yaw_bin': self.decision.yaw_bin if self.decision else None,
            'pose_reasons': list(self.pose.reasons) if self.pose else [],
            'laser_score': self.laser.laser_score if self.laser else None,
            'laser_matched_ratio': self.laser.laser_matched_ratio if self.laser else None,
            'image_quality': self.image.image_quality if self.image else None,
        }


def _scan_to_npz(scan: LaserScan) -> Dict[str, Any]:
    return {
        'ranges': np.asarray(scan.ranges, dtype=np.float32),
        'intensities': np.asarray(scan.intensities, dtype=np.float32)
        if scan.intensities
        else np.zeros(0, dtype=np.float32),
        'angle_min': float(scan.angle_min),
        'angle_max': float(scan.angle_max),
        'angle_increment': float(scan.angle_increment),
        'range_min': float(scan.range_min),
        'range_max': float(scan.range_max),
        'frame_id': str(scan.header.frame_id),
    }


def run_capture_pipeline(
    *,
    cfg: Dict[str, Any],
    pose_input: PoseGateInput,
    scan: Optional[LaserScan],
    occupancy_map: Optional[OccupancyGrid],
    field: Optional[DistanceField],
    bgr: Optional[np.ndarray],
    writer: Optional[CandidateWriter] = None,
    coverage: Optional[CoverageModel] = None,
    dry_run: bool = False,
    source: Optional[str] = None,
    scan_stamp: Optional[float] = None,
    build_session_id: Optional[str] = None,
) -> CaptureResult:
    """Pose → Laser → Image → Coverage policy/dedup → optional Candidate write."""
    writer = writer or CandidateWriter(cfg)
    model = coverage or build_coverage_model(cfg, load_descriptors=False)

    pose = evaluate_pose_gate(pose_input, cfg)
    if not pose.ok:
        writer.record_reject('rejected_pose')
        return CaptureResult(
            status='REJECTED_POSE',
            reason=pose.reason,
            pose=pose,
            dry_run=dry_run,
            metrics=dict(pose.metrics),
        )

    laser = evaluate_laser_gate(
        field=field,
        occupancy_map=occupancy_map,
        scan=scan,
        x=float(pose_input.x),
        y=float(pose_input.y),
        yaw=float(pose_input.yaw),
        cfg=cfg,
        scan_stamp=scan_stamp,
    )
    if not laser.ok:
        writer.record_reject('rejected_laser')
        return CaptureResult(
            status='REJECTED_LASER',
            reason=laser.reason,
            pose=pose,
            laser=laser,
            dry_run=dry_run,
            metrics={**pose.metrics, **laser.metrics},
        )

    image = evaluate_image_gate(bgr, cfg)
    if not image.ok:
        bucket = image.reject_bucket or 'rejected_invalid_frame'
        writer.record_reject(bucket)
        return CaptureResult(
            status='REJECTED_IMAGE',
            reason=image.reason,
            pose=pose,
            laser=laser,
            image=image,
            dry_run=dry_run,
            metrics={**pose.metrics, **laser.metrics, 'image_quality': image.image_quality},
        )

    assert image.orb is not None
    decision = evaluate_capture_policy(
        model,
        x=float(pose_input.x),
        y=float(pose_input.y),
        yaw=float(pose_input.yaw),
        query_descriptors=image.orb.descriptors,
        cfg=cfg,
    )

    if not decision.should_capture:
        skip_bucket = {
            'SKIP_DUPLICATE': 'skip_duplicate',
            'SKIP_COVERED': 'skip_covered',
            'SKIP_CELL_YAW_QUOTA': 'skip_cell_yaw_quota',
            'SKIP_SESSION_QUOTA': 'skip_session_quota',
        }.get(decision.reason, 'skip_covered')
        writer.record_reject(skip_bucket)
        return CaptureResult(
            status=decision.reason,
            reason=decision.reason,
            pose=pose,
            laser=laser,
            image=image,
            decision=decision,
            dry_run=dry_run,
            metrics={
                **pose.metrics,
                **laser.metrics,
                'image_quality': image.image_quality,
                'capture_decision': decision.to_meta(),
                'spatial_cell': decision.spatial_cell,
                'yaw_bin': decision.yaw_bin,
            },
        )

    src = source or str(cfg.get('capture', {}).get('source_default') or 'manual_test')
    meta: Dict[str, Any] = {
        'keyframe_id': None,
        'lifecycle': 'CANDIDATE',
        'map_name': pose_input.map_name,
        'map_hash': pose_input.map_hash,
        'camera_id': str(cfg.get('camera_id') or 'front_up'),
        'x': float(pose_input.x),
        'y': float(pose_input.y),
        'yaw': float(pose_input.yaw),
        'timestamp': float(time.time()),
        'laser_verified': True,
        'laser_score': float(laser.laser_score),
        'laser_matched_ratio': float(laser.laser_matched_ratio),
        'laser_valid_beams': int(laser.laser_valid_beams),
        'scan_stamp': scan_stamp,
        'amcl_cov_xy': pose_input.amcl_cov_xy,
        'amcl_cov_yaw': pose_input.amcl_cov_yaw,
        'localization_status': int(pose_input.localization_status or 0),
        'phase2c_state': pose_input.phase2c_state or pose_input.phase2c_loc_state or 'READY',
        'image_quality': dict(image.image_quality),
        'source': src,
        'build_session_id': build_session_id or '',
        'spatial_cell': decision.spatial_cell,
        'yaw_bin': int(decision.yaw_bin),
        'appearance_id': None,
        'possible_new_appearance': bool(decision.possible_new_appearance),
        'capture_decision': decision.to_meta(),
        'validation': {'status': 'pending'},
        'version_promoted_to': None,
        'retrieval_ready': True,
    }

    try:
        written = writer.write_candidate(
            bgr=bgr,
            orb=image.orb,
            meta=meta,
            scan_npz=_scan_to_npz(scan) if scan is not None else None,
            dry_run=dry_run,
        )
    except Exception as exc:  # noqa: BLE001
        return CaptureResult(
            status='ERROR',
            reason=str(exc),
            pose=pose,
            laser=laser,
            image=image,
            decision=decision,
            dry_run=dry_run,
            metrics={**pose.metrics, **laser.metrics},
        )

    kid = written.get('keyframe_id') or (written.get('meta') or {}).get('keyframe_id')
    # Keep session coverage coherent for consecutive captures (incl. dry-run).
    if kid or decision.should_capture:
        eid = str(kid) if kid else f'dry_{decision.spatial_cell}_{decision.yaw_bin}_{int(time.time()*1000)}'
        if written.get('written') or dry_run:
            if written.get('written'):
                model.session_writes += 1
            model.add_frame(
                FrameRef(
                    keyframe_id=eid,
                    lifecycle='CANDIDATE',
                    x=float(pose_input.x),
                    y=float(pose_input.y),
                    yaw=float(pose_input.yaw),
                    spatial_cell=decision.spatial_cell,
                    yaw_bin=int(decision.yaw_bin),
                    descriptors=image.orb.descriptors,
                    timestamp=float(meta['timestamp']),
                    source=src,
                ),
                cache_desc=False,
            )
            if decision.possible_new_appearance or decision.reason == 'CAPTURE_VISUAL_NOVELTY':
                writer.record_reject('capture_visual_novelty')

    return CaptureResult(
        status='ACCEPTED',
        reason=decision.reason,
        pose=pose,
        laser=laser,
        image=image,
        decision=decision,
        written=bool(written.get('written')),
        dry_run=dry_run,
        keyframe_id=str(kid) if kid else None,
        path=written.get('path'),
        meta=written.get('meta') or meta,
        metrics={
            **pose.metrics,
            **laser.metrics,
            'image_quality': image.image_quality,
            'keyframe_id': kid,
            'capture_decision': decision.to_meta(),
            'spatial_cell': decision.spatial_cell,
            'yaw_bin': decision.yaw_bin,
        },
    )
