"""Phase2D-A2/A3: pose/laser/image gates + coverage/policy/dedup + Candidate writer."""

from __future__ import annotations

from xw_global_reloc.phase2d.candidate_writer import CandidateWriter
from xw_global_reloc.phase2d.capture_pipeline import CaptureResult, run_capture_pipeline
from xw_global_reloc.phase2d.capture_policy import CaptureDecision, evaluate_capture_policy
from xw_global_reloc.phase2d.config_loader import (
    candidate_root_from_cfg,
    legacy_production_paths,
    load_phase2d_config,
)
from xw_global_reloc.phase2d.coverage_model import CoverageModel, build_coverage_model
from xw_global_reloc.phase2d.coverage_report import write_coverage_report
from xw_global_reloc.phase2d.dedup import DedupResult, evaluate_dedup
from xw_global_reloc.phase2d.image_quality_gate import ImageGateResult, evaluate_image_gate
from xw_global_reloc.phase2d.laser_quality_gate import LaserGateResult, evaluate_laser_gate
from xw_global_reloc.phase2d.pose_quality_gate import PoseGateInput, PoseGateResult, evaluate_pose_gate
from xw_global_reloc.phase2d.spatial import spatial_cell_id, yaw_bin_index

__all__ = [
    'load_phase2d_config',
    'candidate_root_from_cfg',
    'legacy_production_paths',
    'PoseGateInput',
    'PoseGateResult',
    'evaluate_pose_gate',
    'ImageGateResult',
    'evaluate_image_gate',
    'LaserGateResult',
    'evaluate_laser_gate',
    'CaptureResult',
    'run_capture_pipeline',
    'CandidateWriter',
    'CoverageModel',
    'build_coverage_model',
    'CaptureDecision',
    'evaluate_capture_policy',
    'DedupResult',
    'evaluate_dedup',
    'write_coverage_report',
    'spatial_cell_id',
    'yaw_bin_index',
]
