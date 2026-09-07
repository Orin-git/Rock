"""Unit tests for SE(2) candidate composition — Stage 4."""

from __future__ import annotations

import math

import numpy as np

from xw_global_reloc.transforms import (
    Pose2D,
    compose_candidate_base_pose,
    se3_from_xyz_rpy,
)


def test_identity_relative():
    kf = Pose2D(1.0, 2.0, 0.3)
    T_rel = np.eye(4)
    T_bc = se3_from_xyz_rpy(0.25, 0.0, 0.4, 0.0, 0.0, 0.0)
    pose, diag = compose_candidate_base_pose(kf, T_rel, T_bc)
    assert abs(pose.x - 1.0) < 1e-6
    assert abs(pose.y - 2.0) < 1e-6
    assert abs(pose.yaw - 0.3) < 1e-6
    assert diag['extrinsic_status'] == 'EXTRINSIC_UNCALIBRATED'


def test_pure_x_translation_in_camera():
    """Relative cam translation +X in query←kf should move base accordingly for identity extrinsic."""
    kf = Pose2D(0.0, 0.0, 0.0)
    T_rel = np.eye(4)
    T_rel[0, 3] = 0.5  # query sees kf points shifted — object in kf is 0.5m along +X_query
    T_bc = np.eye(4)  # cam = base
    pose, _ = compose_candidate_base_pose(kf, T_rel, T_bc)
    # T_map_query_cam = T_map_kf_cam * inv(T_query_from_kf)
    # inv shifts -0.5 in x for cam=base → query base at -0.5
    assert abs(pose.x - (-0.5)) < 1e-6
    assert abs(pose.y - 0.0) < 1e-6


def test_pure_yaw():
    kf = Pose2D(0.0, 0.0, 0.0)
    yaw = math.pi / 2
    T_rel = se3_from_xyz_rpy(0.0, 0.0, 0.0, 0.0, 0.0, yaw)
    T_bc = np.eye(4)
    pose, _ = compose_candidate_base_pose(kf, T_rel, T_bc)
    # inv(yaw)= -yaw → query yaw ≈ -pi/2
    assert abs(pose.yaw - (-yaw)) < 1e-6
