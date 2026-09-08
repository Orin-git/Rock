"""Laser prior scoring must ignore out-of-map endpoints and never read live TF."""

from __future__ import annotations

import ast
import math
from pathlib import Path

import numpy as np
from nav_msgs.msg import MapMetaData, OccupancyGrid
from sensor_msgs.msg import LaserScan

from xw_global_reloc.laser_verify import DistanceField, score_scan_at_pose, score_scan_at_poses, prepare_scan


def _corridor(w=120, h=40, res=0.05) -> OccupancyGrid:
    data = np.zeros((h, w), dtype=np.int8)
    data[0, :] = 100
    data[-1, :] = 100
    data[:, 0] = 100
    data[:, -1] = 100
    grid = OccupancyGrid()
    grid.info = MapMetaData()
    grid.info.resolution = res
    grid.info.width = w
    grid.info.height = h
    grid.info.origin.position.x = -1.0
    grid.info.origin.position.y = -1.0
    grid.data = data.flatten().tolist()
    return grid


def _scan_hitting_walls(n=180) -> LaserScan:
    """Beams that land on the corridor walls from (0, 0, 0) with lidar yaw = pi."""
    scan = LaserScan()
    scan.header.frame_id = 'lidar_link'
    scan.angle_min = -math.pi
    scan.angle_max = math.pi
    scan.angle_increment = (2 * math.pi) / n
    scan.range_min = 0.05
    scan.range_max = 8.0
    # map x from -1 to 5, y from -1 to 1. Pose (0,0,0), lidar yaw=pi.
    # North/south walls at y=±1. A beam at lidar angle 0 goes to map angle pi (west).
    ranges = []
    lyaw = math.pi
    for i in range(n):
        a = scan.angle_min + i * scan.angle_increment
        hit = 3.0
        for r in np.linspace(0.15, 4.5, 90):
            bx = r * math.cos(a)
            by = r * math.sin(a)
            lc, ls = math.cos(lyaw), math.sin(lyaw)
            mx = 0.0 + lc * bx - ls * by
            my = 0.0 + ls * bx + lc * by
            if abs(my) >= 0.95 or mx <= -0.95 or mx >= 4.95:
                hit = r
                break
        ranges.append(float(hit))
    scan.ranges = ranges
    return scan


def test_score_does_not_import_tf_or_amcl() -> None:
    src = Path(__file__).resolve().parents[1] / 'xw_global_reloc' / 'laser_verify.py'
    tree = ast.parse(src.read_text(encoding='utf-8'))
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(a.name for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            names.add(node.module)
        elif isinstance(node, ast.Attribute):
            names.add(node.attr)
        elif isinstance(node, ast.Name):
            names.add(node.id)
    banned = {'tf2_ros', 'tf2', 'lookup_transform', 'amcl_pose', 'TransformListener'}
    assert names.isdisjoint(banned), names & banned


def test_same_candidate_score_is_independent_of_any_external_pose() -> None:
    grid = _corridor()
    field = DistanceField(grid)
    scan = _scan_hitting_walls()
    # The scorer has no pose argument other than the candidate. Calling it twice
    # with the same scan+candidate must match even if a caller "thinks" AMCL moved.
    a = score_scan_at_pose(field, scan, 0.0, 0.0, 0.0, min_laser_score=0.38)
    b = score_scan_at_pose(field, scan, 0.0, 0.0, 0.0, min_laser_score=0.38)
    assert a.laser_score == b.laser_score
    assert a.mean_dist == b.mean_dist
    assert a.matched_ratio == b.matched_ratio


def test_one_oob_endpoint_does_not_zero_a_matching_pose() -> None:
    grid = _corridor()
    field = DistanceField(grid)
    scan = _scan_hitting_walls()
    # Push one finite beam to a range that leaves the map (map x max = 5.0).
    ranges = list(scan.ranges)
    ranges[10] = 7.5
    scan.ranges = ranges
    sc = score_scan_at_pose(field, scan, 0.0, 0.0, 0.0, min_laser_score=0.38)
    assert sc.valid_beams >= 20
    assert sc.mean_dist < 1.0
    assert sc.laser_score > 0.0
    # The 1e3 sentinel must not survive into the mean.
    assert sc.mean_dist < 50.0


def test_wrong_pose_still_rejected() -> None:
    grid = _corridor()
    field = DistanceField(grid)
    scan = _scan_hitting_walls()
    bad = score_scan_at_pose(field, scan, 8.0, 6.0, 1.2, min_laser_score=0.38)
    assert bad.accepted is False
    assert bad.laser_score < 0.38


def test_batch_matches_single_after_oob_filter() -> None:
    grid = _corridor()
    field = DistanceField(grid)
    scan = _scan_hitting_walls()
    prep = prepare_scan(scan, beam_stride=6)
    single = score_scan_at_pose(field, scan, 0.2, 0.0, 0.1, prepared=prep, min_laser_score=0.38)
    batch = score_scan_at_poses(
        field,
        prep,
        np.array([0.2]),
        np.array([0.0]),
        np.array([0.1]),
        min_laser_score=0.38,
    )
    assert abs(batch[0].laser_score - single.laser_score) < 1e-9
    assert abs(batch[0].mean_dist - single.mean_dist) < 1e-9
