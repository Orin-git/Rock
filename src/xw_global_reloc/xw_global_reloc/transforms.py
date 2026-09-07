"""SE(2)/SE(3) helpers for candidate base pose.

Extrinsic currently URDF-derived → mark EXTRINSIC_UNCALIBRATED in callers.
Chain (PoC convention):
  T_map_base = T_map_base_kf * T_base_cam * T_cam_kf_to_query^{-1} * T_base_cam^{-1}
Or equivalently working in camera frames then projecting to SE(2).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float

    def as_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.yaw)


def yaw_to_rot2(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]], dtype=np.float64)


def se2_matrix(x: float, y: float, yaw: float) -> np.ndarray:
    T = np.eye(3, dtype=np.float64)
    T[:2, :2] = yaw_to_rot2(yaw)
    T[0, 2] = x
    T[1, 2] = y
    return T


def se2_from_matrix(T: np.ndarray) -> Pose2D:
    yaw = math.atan2(float(T[1, 0]), float(T[0, 0]))
    return Pose2D(float(T[0, 2]), float(T[1, 2]), yaw)


def se2_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return a @ b


def se2_inv(T: np.ndarray) -> np.ndarray:
    R = T[:2, :2]
    t = T[:2, 2]
    Ti = np.eye(3, dtype=np.float64)
    Ti[:2, :2] = R.T
    Ti[:2, 2] = -R.T @ t
    return Ti


def se3_from_xyz_rpy(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF rpy intrinsic ZYX (yaw→pitch→roll) common ROS convention."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    R = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def se3_inv(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4, dtype=np.float64)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def se3_to_se2_yaw(T: np.ndarray) -> Pose2D:
    """Project SE(3) base pose to ground SE(2); roll/pitch discarded for chassis."""
    x = float(T[0, 3])
    y = float(T[1, 3])
    yaw = math.atan2(float(T[1, 0]), float(T[0, 0]))
    return Pose2D(x, y, yaw)


def rpy_from_rot(R: np.ndarray) -> Tuple[float, float, float]:
    pitch = math.asin(float(np.clip(-R[2, 0], -1.0, 1.0)))
    roll = math.atan2(float(R[2, 1]), float(R[2, 2]))
    yaw = math.atan2(float(R[1, 0]), float(R[0, 0]))
    return roll, pitch, yaw


def compose_candidate_base_pose(
    kf_map_base: Pose2D,
    T_cam_kf_from_query: np.ndarray,
    T_base_from_cam: np.ndarray,
) -> Tuple[Pose2D, dict]:
    """Compose candidate map←base from keyframe pose + PnP relative cam transform.

    OpenCV solvePnP returns rvec/tvec of **object→camera** (world points in kf cam
    expressed in query cam): T_query_from_kf_cam (points_query = R * points_kf + t).

    We store T_cam_kf_from_query as that 4x4 (query←kf), i.e. T_query_cam_from_kf_cam.

    Then:
      T_map_from_query_cam = T_map_from_kf_cam * inv(T_query_from_kf)
      T_map_from_query_base = T_map_from_query_cam * inv(T_base_from_cam)^{-1}
                           = T_map_from_query_cam * T_cam_from_base

    With T_map_from_kf_cam = T_map_from_kf_base * T_base_from_cam
    """
    if T_cam_kf_from_query.shape != (4, 4):
        raise ValueError('T_cam_kf_from_query must be 4x4')
    if T_base_from_cam.shape != (4, 4):
        raise ValueError('T_base_from_cam must be 4x4')

    T_map_kf_base = se3_from_xyz_rpy(kf_map_base.x, kf_map_base.y, 0.0, 0.0, 0.0, kf_map_base.yaw)
    T_map_kf_cam = T_map_kf_base @ T_base_from_cam
    T_query_from_kf = T_cam_kf_from_query  # query_cam ← kf_cam
    T_map_query_cam = T_map_kf_cam @ se3_inv(T_query_from_kf)
    T_cam_from_base = se3_inv(T_base_from_cam)
    T_map_query_base = T_map_query_cam @ T_cam_from_base

    pose = se3_to_se2_yaw(T_map_query_base)
    roll, pitch, _ = rpy_from_rot(T_map_query_base[:3, :3])
    diag = {
        'extrinsic_source': 'URDF',
        'extrinsic_status': 'EXTRINSIC_UNCALIBRATED',
        'chain': 'T_map_base = T_map_kf_base * T_base_cam * inv(T_query_from_kf) * inv(T_base_cam)',
        'roll_diag': roll,
        'pitch_diag': pitch,
        'se2': pose.as_tuple(),
    }
    return pose, diag


# --- Unit-test helpers for identity / pure-x / pure-yaw ---

def identity_case_expected() -> Pose2D:
    return Pose2D(1.0, 2.0, 0.3)


def run_identity_composition() -> Pose2D:
    kf = Pose2D(1.0, 2.0, 0.3)
    T_rel = np.eye(4)
    T_bc = np.eye(4)
    T_bc[0, 3] = 0.25
    pose, _ = compose_candidate_base_pose(kf, T_rel, T_bc)
    return pose
