#!/usr/bin/env python3
"""Gen2 frontier exploration node (ported from Gen1 autonomous_mapping_node).

Lifecycle is owned by xw_explore explore_session: process start/stop.
Subscribes SLAM /map, sends NavigateToPose goals; recovery Twist → /xw/cmd/nav.
"""

import math
import time
import threading
from collections import deque

import numpy as np
import cv2

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Twist
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, Int16
from visualization_msgs.msg import Marker

from tf2_ros import Buffer, TransformListener
from action_msgs.msg import GoalStatus


def quaternion_from_euler(roll, pitch, yaw):
    """yaw/pitch/roll -> (x, y, z, w)，避免依赖未安装的 tf_transformations。"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class ExploreNode(Node):
    def __init__(self):
        super().__init__('xw_explore_frontier')
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameters(namespace='', parameters=[
            ('obstacle_inflation_cells', 6),
            ('frontier_cluster_epsilon', 0.55),
            ('min_frontier_size', 8),
            ('frontier_min_obstacle_distance_cells', 6),
            ('frontier_min_component_size', 12),
            ('max_goal_distance', 12.0),
            ('min_goal_distance', 1.2),
            ('goal_tolerance', 1.0),
            ('blacklist_radius', 0.55),
            ('max_navigation_time', 55.0),
            ('stuck_timeout_sec', 15.0),
            ('stuck_movement_radius', 0.10),
            ('recovery_rotation', 30.0),
            ('recovery_backup', 0.10),
            ('recovery_angular_speed', 0.10),
            ('recovery_settle_sec', 1.5),
            ('iteration_limit', 80),
            ('no_frontier_rotate', 75.0),
            ('no_frontier_limit', 12),
            ('no_goal_limit', 30),
            ('forward_probe_distance', 0.30),
            ('forward_probe_speed', 0.08),
            ('doorway_preferred_dist_min', 0.45),
            ('doorway_preferred_dist_max', 1.6),
            ('doorway_min_clearance_m', 0.10),
            ('doorway_max_clearance_m', 0.42),
            ('doorway_pass_distance', 0.35),
            ('doorway_pass_speed', 0.06),
            ('doorway_align_speed', 0.08),
            ('doorway_max_attempts', 2),
            ('prefer_open_space', False),
            ('min_open_clearance_m', 0.20),
            ('open_space_weight', 0.25),
            ('size_weight', 0.06),
            ('info_gain_weight', 0.05),
            ('info_gain_radius_m', 1.2),
            ('unknown_adjacent_min_cells', 8),
            ('unknown_adjacent_min_clearance_m', 0.08),
            ('skip_pullback_unknown_adjacent', True),
            ('narrow_corridor_width_min_m', 0.55),
            ('narrow_corridor_width_max_m', 0.90),
            ('goal_pullback_clearance_m', 0.30),
            ('goal_pullback_max_m', 0.50),
            ('blind_spot_penalty', 1.5),
            ('goal_commit_sec', 25.0),
            ('goal_reselect_progress_m', 0.35),
            ('map_topic', '/map'),
            ('robot_frame', 'base_link'),
            ('global_frame', 'map'),
            # Recovery / probe must go through Gen2 arbiter (never /cmd_vel).
            ('cmd_vel_topic', '/xw/cmd/nav'),
        ])

        self.obstacle_inflation_cells = self.get_parameter('obstacle_inflation_cells').value
        self.frontier_cluster_epsilon = self.get_parameter('frontier_cluster_epsilon').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.frontier_min_obstacle_distance_cells = self.get_parameter('frontier_min_obstacle_distance_cells').value
        self.frontier_min_component_size = self.get_parameter('frontier_min_component_size').value
        self.max_goal_distance = self.get_parameter('max_goal_distance').value
        self.min_goal_distance = self.get_parameter('min_goal_distance').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.blacklist_radius = self.get_parameter('blacklist_radius').value
        self.max_navigation_time = self.get_parameter('max_navigation_time').value
        self.stuck_timeout_sec = self.get_parameter('stuck_timeout_sec').value
        self.stuck_movement_radius = self.get_parameter('stuck_movement_radius').value
        self.recovery_rotation = self.get_parameter('recovery_rotation').value
        self.recovery_backup = self.get_parameter('recovery_backup').value
        self.recovery_angular_speed = self.get_parameter('recovery_angular_speed').value
        self.recovery_settle_sec = self.get_parameter('recovery_settle_sec').value
        self.iteration_limit = self.get_parameter('iteration_limit').value
        self.no_frontier_rotate = self.get_parameter('no_frontier_rotate').value
        self.no_frontier_limit = self.get_parameter('no_frontier_limit').value
        self.no_goal_limit = self.get_parameter('no_goal_limit').value
        self.forward_probe_distance = self.get_parameter('forward_probe_distance').value
        self.forward_probe_speed = self.get_parameter('forward_probe_speed').value
        self.doorway_preferred_dist_min = self.get_parameter('doorway_preferred_dist_min').value
        self.doorway_preferred_dist_max = self.get_parameter('doorway_preferred_dist_max').value
        self.doorway_min_clearance_m = self.get_parameter('doorway_min_clearance_m').value
        self.doorway_max_clearance_m = self.get_parameter('doorway_max_clearance_m').value
        self.doorway_pass_distance = self.get_parameter('doorway_pass_distance').value
        self.doorway_pass_speed = self.get_parameter('doorway_pass_speed').value
        self.doorway_align_speed = self.get_parameter('doorway_align_speed').value
        self.doorway_max_attempts = self.get_parameter('doorway_max_attempts').value
        self.prefer_open_space = self.get_parameter('prefer_open_space').value
        self.min_open_clearance_m = self.get_parameter('min_open_clearance_m').value
        self.open_space_weight = self.get_parameter('open_space_weight').value
        self.size_weight = self.get_parameter('size_weight').value
        self.info_gain_weight = self.get_parameter('info_gain_weight').value
        self.info_gain_radius_m = self.get_parameter('info_gain_radius_m').value
        self.unknown_adjacent_min_cells = self.get_parameter('unknown_adjacent_min_cells').value
        self.unknown_adjacent_min_clearance_m = self.get_parameter(
            'unknown_adjacent_min_clearance_m').value
        self.skip_pullback_unknown_adjacent = self.get_parameter(
            'skip_pullback_unknown_adjacent').value
        self.narrow_corridor_width_min_m = self.get_parameter('narrow_corridor_width_min_m').value
        self.narrow_corridor_width_max_m = self.get_parameter('narrow_corridor_width_max_m').value
        self.goal_pullback_clearance_m = self.get_parameter('goal_pullback_clearance_m').value
        self.goal_pullback_max_m = self.get_parameter('goal_pullback_max_m').value
        self.blind_spot_penalty = self.get_parameter('blind_spot_penalty').value
        self.goal_commit_sec = self.get_parameter('goal_commit_sec').value
        self.goal_reselect_progress_m = self.get_parameter('goal_reselect_progress_m').value
        self.map_topic = self.get_parameter('map_topic').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.global_frame = self.get_parameter('global_frame').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.current_map = None
        self.map_lock = threading.Lock()
        self.exploration_active = True
        self.iteration = 0
        self.no_frontier_count = 0   # 连续“地图上无 frontier 簇”
        self.no_goal_count = 0       # 连续“有 frontier 但选不出目标”
        self.recovery_rotation_total = 0.0
        self.blacklist = []  # (x, y, yaw) 历史失败/到达点
        self.goal_history = []  # (x, y) 到达过的目标
        self.timeout_goals = []  # 超时重试目标
        self.last_pose = None
        self.last_pose_time = 0.0
        self.active_goal = None  # Point
        self.active_goal_start_pose = None
        self.active_goal_start_time = 0.0
        self.cluster_sizes = {}  # (qx,qy) -> size
        self.doorway_pass_attempts = {}  # (qx,qy) -> count
        self.last_nav_infra_failure = False
        self.last_nav_stuck = False
        self.last_stuck_pose = None
        self.stuck_iteration_count = 0
        self.goal_stuck_counts = {}  # (qx,qy) -> 连续卡住次数
        self.did_doorway_this_cycle = False

        self.nav_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose', callback_group=self.cb_group)
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.frontier_marker_pub = self.create_publisher(Marker, 'frontier_markers', 10)
        self.centroid_marker_pub = self.create_publisher(Marker, 'centroid_markers', 10)
        self.iteration_pub = self.create_publisher(Int16, 'planner_iteration', 10)
        self.status_pub = self.create_publisher(Bool, '/xw/explore/frontier_active', 10)
        self.finished_pub = self.create_publisher(Bool, '/xw/explore/finished', 10)

        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, 1,
            callback_group=self.cb_group)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 探索在独立线程跑；导航等待用 future.result()，由 MultiThreadedExecutor 推进回调
        self.exploration_thread = threading.Thread(target=self.run_exploration, daemon=True)
        self.exploration_thread.start()

        self.status_timer = self.create_timer(
            1.0, self.publish_status, callback_group=self.cb_group)
        self.get_logger().info('xw_explore frontier ready, waiting for /map...')

    def map_callback(self, msg):
        with self.map_lock:
            self.current_map = msg

    def publish_status(self):
        msg = Bool()
        msg.data = self.exploration_active
        self.status_pub.publish(msg)

    def get_current_pose(self):
        pose = self.get_current_pose_with_yaw()
        if pose is None:
            return None
        return (pose[0], pose[1], 0.0)

    def get_current_pose_with_yaw(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_frame,
                rclpy.time.Time(seconds=0, nanoseconds=0),
                rclpy.duration.Duration(seconds=1.0))
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return (x, y, yaw)
        except Exception as e:
            self.get_logger().warn(f'获取当前位姿失败: {e}')
            return None

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def goal_key(self, point):
        return (round(point.x, 2), round(point.y, 2))

    def publish_finished(self, reason: str):
        if not rclpy.ok():
            return
        msg = Bool()
        msg.data = True
        try:
            self.finished_pub.publish(msg)
            for _ in range(2):
                if not rclpy.ok():
                    break
                time.sleep(0.2)
                self.finished_pub.publish(msg)
            self.get_logger().info(f'已发布探索完成信号: {reason}')
        except Exception as exc:
            self.get_logger().warn(f'发布探索完成信号失败: {exc}')

    def inflate_map(self, occupancy_grid):
        data = np.array(occupancy_grid.data).reshape(
            occupancy_grid.info.height, occupancy_grid.info.width)
        binary_map = np.where(data >= 65, 1, 0).astype(np.uint8)
        binary_map = np.where(data == -1, 0, binary_map)
        kernel = np.ones((self.obstacle_inflation_cells * 2 + 1, self.obstacle_inflation_cells * 2 + 1), np.uint8)
        inflated = cv2.dilate(binary_map, kernel)
        result = data.copy()
        result[inflated == 1] = 100
        return result

    def _ray_distance_to_obstacle(self, obstacle_mask, y, x, dy, dx, max_cells, res):
        h, w = obstacle_mask.shape
        for step in range(1, max_cells + 1):
            ny = y + dy * step
            nx = x + dx * step
            if ny < 0 or nx < 0 or ny >= h or nx >= w:
                return None
            if obstacle_mask[ny, nx]:
                return step * res
        return None

    def is_narrow_corridor_cell(self, obstacle_mask, y, x, res):
        """两侧有墙且通道宽约 [narrow_min, narrow_max] 时视为可通行窄缝。"""
        width_min = float(self.narrow_corridor_width_min_m)
        width_max = float(self.narrow_corridor_width_max_m)
        max_cells = int(math.ceil(width_max / max(res, 1e-6))) + 2
        direction_pairs = (
            ((0, 1), (0, -1)),
            ((1, 0), (-1, 0)),
            ((1, 1), (-1, -1)),
            ((1, -1), (-1, 1)),
        )
        for d1, d2 in direction_pairs:
            a = self._ray_distance_to_obstacle(obstacle_mask, y, x, d1[0], d1[1], max_cells, res)
            b = self._ray_distance_to_obstacle(obstacle_mask, y, x, d2[0], d2[1], max_cells, res)
            if a is None or b is None:
                continue
            width = a + b
            # 浮点累加误差：0.1*9 可能略大于 0.90
            if width_min - 1e-6 <= width <= width_max + 1e-6:
                return True
        return False

    def detect_frontiers(self, occupancy_grid):
        data = np.array(occupancy_grid.data).reshape(
            occupancy_grid.info.height, occupancy_grid.info.width)
        inflated = self.inflate_map(occupancy_grid)
        res = occupancy_grid.info.resolution
        ox = occupancy_grid.info.origin.position.x
        oy = occupancy_grid.info.origin.position.y

        free_mask = (data == 0)
        unknown_mask = (data == -1)
        inflated_obstacle_mask = (inflated >= 65)
        raw_obstacle_mask = (data >= 65)

        kernel = np.ones((3, 3), np.uint8)
        # Navigation goals must be placed on known-free cells, not unknown frontier cells.
        dilated_unknown = cv2.dilate(unknown_mask.astype(np.uint8), kernel)
        raw_frontier_mask = (dilated_unknown == 1) & free_mask

        free_of_inflated = (~inflated_obstacle_mask).astype(np.uint8)
        inflated_distance = cv2.distanceTransform(free_of_inflated, cv2.DIST_L2, 3)
        safe_mask = raw_frontier_mask & (
            inflated_distance >= self.frontier_min_obstacle_distance_cells)

        # 窄缝例外：离障过滤会误杀门洞/窄通道中的 frontier，对两侧有墙的 0.55–0.9m 通道放行
        rejected = raw_frontier_mask & (~safe_mask)
        if np.any(rejected):
            restore = np.zeros_like(rejected, dtype=bool)
            for y, x in np.column_stack(np.where(rejected)):
                if self.is_narrow_corridor_cell(raw_obstacle_mask, int(y), int(x), res):
                    restore[y, x] = True
            frontier_mask = safe_mask | restore
        else:
            frontier_mask = safe_mask

        labeled_count, labeled = cv2.connectedComponents(frontier_mask.astype(np.uint8))
        num_labels = int(labeled_count) - 1
        if num_labels > 0:
            component_sizes = np.bincount(labeled.ravel())
            valid = np.zeros_like(component_sizes, dtype=bool)
            valid[component_sizes >= self.frontier_min_component_size] = True
            valid[0] = False
            frontier_mask = valid[labeled]

        coords = np.column_stack(np.where(frontier_mask))
        frontiers = []
        for y, x in coords:
            frontiers.append(Point(
                x=float(x * res + ox),
                y=float(y * res + oy),
                z=0.0))
        return frontiers

    def pullback_goal(self, point, occupancy_grid):
        """把目标沿 clearance 梯度回撤到更空旷的已知自由格，减少贴墙下发。"""
        if point is None or occupancy_grid is None:
            return point
        target = float(self.goal_pullback_clearance_m)
        max_travel = float(self.goal_pullback_max_m)
        if target <= 0.0 or max_travel <= 0.0:
            return point

        clearance_map, res, ox, oy = self.build_clearance_map(occupancy_grid)
        height, width = clearance_map.shape
        data = np.array(occupancy_grid.data).reshape(height, width)

        def cell_of(px, py):
            mx = int((px - ox) / res)
            my = int((py - oy) / res)
            return mx, my

        def is_free(mx, my):
            if mx < 0 or my < 0 or mx >= width or my >= height:
                return False
            return data[my, mx] == 0

        x, y = float(point.x), float(point.y)
        mx, my = cell_of(x, y)
        if not is_free(mx, my):
            return point

        current_clearance = float(clearance_map[my, mx] * res)
        if current_clearance >= target:
            return point

        traveled = 0.0
        neighbors = (
            (1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (1, -1), (-1, 1), (-1, -1),
        )
        while traveled < max_travel:
            mx, my = cell_of(x, y)
            current_clearance = float(clearance_map[my, mx] * res)
            if current_clearance >= target:
                break
            best = None
            best_clearance = current_clearance
            for dy, dx in neighbors:
                ny, nx = my + dy, mx + dx
                if not is_free(nx, ny):
                    continue
                c = float(clearance_map[ny, nx] * res)
                if c > best_clearance + 1e-6:
                    best_clearance = c
                    best = (nx, ny, c)
            if best is None:
                break
            nx, ny, _ = best
            x = nx * res + ox
            y = ny * res + oy
            traveled += res

        return Point(x=float(x), y=float(y), z=0.0)

    def cluster_frontiers(self, frontiers):
        """简易连通聚类：返回 (representative_points, sizes)。"""
        if len(frontiers) == 0:
            return [], []
        points = np.array([[p.x, p.y] for p in frontiers], dtype=np.float64)
        n = len(points)
        eps = float(self.frontier_cluster_epsilon)
        min_size = int(self.min_frontier_size)
        visited = np.zeros(n, dtype=bool)
        centroids = []
        sizes = []
        for i in range(n):
            if visited[i]:
                continue
            queue = [i]
            visited[i] = True
            members = [i]
            while queue:
                idx = queue.pop()
                d = np.hypot(points[:, 0] - points[idx, 0], points[:, 1] - points[idx, 1])
                neighbors = np.where((~visited) & (d <= eps))[0]
                for j in neighbors:
                    visited[j] = True
                    queue.append(int(j))
                    members.append(int(j))
            if len(members) < min_size:
                continue
            cluster_points = points[members]
            centroid = np.mean(cluster_points, axis=0)
            distances = np.hypot(cluster_points[:, 0] - centroid[0], cluster_points[:, 1] - centroid[1])
            representative = cluster_points[int(np.argmin(distances))]
            centroids.append(Point(x=float(representative[0]), y=float(representative[1]), z=0.0))
            sizes.append(len(members))
        return centroids, sizes

    def build_clearance_map(self, occupancy_grid):
        data = np.array(occupancy_grid.data).reshape(
            occupancy_grid.info.height, occupancy_grid.info.width)
        free_of_obstacle = ((data >= 0) & (data < 65)).astype(np.uint8)
        dist = cv2.distanceTransform(free_of_obstacle, cv2.DIST_L2, 3)
        return dist, occupancy_grid.info.resolution, occupancy_grid.info.origin.position.x, occupancy_grid.info.origin.position.y

    def clearance_at(self, clearance_map, res, ox, oy, width, height, x, y):
        mx = int((x - ox) / res)
        my = int((y - oy) / res)
        if mx < 0 or my < 0 or mx >= width or my >= height:
            return 0.0
        return float(clearance_map[my, mx] * res)

    def count_unknown_near(self, occupancy_grid, x, y, radius_m=None):
        """统计目标附近未知格数量，作为信息增益 / 邻接大未知区的近似。"""
        if occupancy_grid is None:
            return 0
        res = float(occupancy_grid.info.resolution)
        if res <= 0.0:
            return 0
        radius = float(radius_m if radius_m is not None else self.info_gain_radius_m)
        ox = occupancy_grid.info.origin.position.x
        oy = occupancy_grid.info.origin.position.y
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        data = np.asarray(occupancy_grid.data).reshape(height, width)
        mx = int((x - ox) / res)
        my = int((y - oy) / res)
        r_cells = max(1, int(math.ceil(radius / res)))
        x0 = max(0, mx - r_cells)
        x1 = min(width, mx + r_cells + 1)
        y0 = max(0, my - r_cells)
        y1 = min(height, my + r_cells + 1)
        if x0 >= x1 or y0 >= y1:
            return 0
        patch = data[y0:y1, x0:x1]
        # 圆形窗口，避免角落里的无关未知灌水
        yy, xx = np.ogrid[y0:y1, x0:x1]
        mask = ((xx - mx) * res) ** 2 + ((yy - my) * res) ** 2 <= radius ** 2
        return int(np.count_nonzero((patch == -1) & mask))

    def is_unknown_adjacent(self, occupancy_grid, x, y):
        return self.count_unknown_near(occupancy_grid, x, y) >= int(
            self.unknown_adjacent_min_cells)

    def should_skip_pullback(self, point, occupancy_grid):
        """邻接大未知区 / 窄门口的目标不回撤，否则会撤回已知空地进不去门。"""
        if point is None or occupancy_grid is None:
            return False
        if not bool(self.skip_pullback_unknown_adjacent):
            return False
        if self.is_unknown_adjacent(occupancy_grid, point.x, point.y):
            return True
        clearance_map, res, ox, oy = self.build_clearance_map(occupancy_grid)
        clearance = self.clearance_at(
            clearance_map, res, ox, oy,
            occupancy_grid.info.width, occupancy_grid.info.height,
            point.x, point.y)
        # 低 clearance 更像门口/窄缝，回撤会把它拽离入口
        return clearance < float(self.min_open_clearance_m)

    def is_blacklisted(self, point, radius=None):
        r = radius if radius is not None else self.blacklist_radius
        for bx, by, _ in self.blacklist:
            if math.hypot(point.x - bx, point.y - by) < r:
                return True
        return False

    def add_to_blacklist(self, x, y, yaw):
        self.blacklist.append((x, y, yaw))
        # 避免小区域反复失败时黑名单无限膨胀
        if len(self.blacklist) > 12:
            self.blacklist = self.blacklist[-8:]

    def effective_max_goal_distance(self):
        """连续选不出目标时逐步放宽距离上限，避免卡在门口。"""
        base = float(self.max_goal_distance)
        if self.no_goal_count >= 8:
            return base * 2.0
        if self.no_goal_count >= 4:
            return base * 1.5
        if self.no_goal_count >= 2:
            return base * 1.25
        return base

    def maybe_trim_blacklist_for_stuck(self):
        if self.no_goal_count >= 4 and len(self.blacklist) > 2:
            dropped = len(self.blacklist) - 2
            self.blacklist = self.blacklist[-2:]
            self.get_logger().warn(
                f'连续 {self.no_goal_count} 次无目标，裁剪黑名单 {dropped} 条')

    def clear_blacklist_if_blocking_all(self, centroids):
        if not centroids:
            return
        blocked = sum(1 for p in centroids if self.is_blacklisted(p))
        if blocked >= len(centroids):
            self.get_logger().warn(
                f'全部 {len(centroids)} 个 frontier 均在黑名单，清空黑名单')
            self.blacklist.clear()
            self.doorway_pass_attempts.clear()

    def update_stuck_watchdog(self, current_pose):
        if current_pose is None:
            return
        if self.last_stuck_pose is None:
            self.last_stuck_pose = (current_pose[0], current_pose[1])
            self.stuck_iteration_count = 0
            return
        moved = math.hypot(
            current_pose[0] - self.last_stuck_pose[0],
            current_pose[1] - self.last_stuck_pose[1])
        if moved < 0.35:
            self.stuck_iteration_count += 1
        else:
            self.last_stuck_pose = (current_pose[0], current_pose[1])
            self.stuck_iteration_count = 0

    def rotate_toward_yaw(self, target_yaw, speed=None, tolerance_deg=8.0, max_duration=15.0):
        """闭环原地旋转对准，直到偏差足够小或超时。"""
        align_speed = float(speed if speed is not None else self.doorway_align_speed)
        align_speed = min(abs(align_speed), abs(float(self.recovery_angular_speed)))
        start = time.monotonic()
        last_log = start
        while time.monotonic() - start < max_duration and rclpy.ok():
            pose = self.get_current_pose_with_yaw()
            if pose is None:
                break
            dyaw = self.normalize_angle(target_yaw - pose[2])
            if abs(dyaw) <= math.radians(tolerance_deg):
                self.get_logger().info(
                    f'对准完成, 残余 {math.degrees(dyaw):.1f}°')
                break
            if time.monotonic() - last_log > 3.0:
                self.get_logger().info(
                    f'原地对准中: 残余 {math.degrees(dyaw):.1f}°')
                last_log = time.monotonic()
            twist = Twist()
            twist.angular.z = align_speed if dyaw > 0 else -align_speed
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.08)
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.12)
        settle = float(self.recovery_settle_sec)
        if settle > 0:
            time.sleep(settle)
        return True

    def drive_forward(self, distance, speed=0.05):
        twist = Twist()
        twist.linear.x = speed
        duration = max(0.1, distance / speed)
        end_time = time.monotonic() + duration
        while time.monotonic() < end_time and rclpy.ok():
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.08)
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)

    def corner_escape_forward(self, goal=None):
        """角点/门口长时间不动：先对准目标方向再慢速前探。"""
        pose = self.get_current_pose_with_yaw()
        if pose is None:
            return
        cx, cy, yaw = pose
        target_yaw = yaw
        if goal is not None:
            target_yaw = math.atan2(goal.y - cy, goal.x - cx)
            self.rotate_toward_yaw(target_yaw)
        speed = 0.05
        distance = 0.45
        self.get_logger().warn(
            f'角点脱困: 朝 {math.degrees(target_yaw):.0f}° 慢速前进 {distance:.2f}m')
        self.drive_forward(distance, speed)
        time.sleep(float(self.recovery_settle_sec))

    def select_best_goal(self, centroids, sizes, current_pose, occupancy_grid, strict=True):
        """选择最佳 frontier 目标。

        优先：大簇 + 邻接未知（信息增益 / 窄门口），允许较远距离（默认 12m）。
        空旷优先默认关闭；邻接大未知区时放宽 clearance，避免忽略窄门。
        """
        if not centroids or current_pose is None:
            return None
        cx, cy, _ = current_pose
        clearance_map, res, ox, oy = self.build_clearance_map(occupancy_grid)
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        best = None
        best_score = float('inf')

        min_goal_dist = float(self.min_goal_distance) if strict else 0.12
        max_goal_dist = self.effective_max_goal_distance()
        base_min_clearance = float(self.min_open_clearance_m) * 0.5 if strict else 0.05
        relaxed_clearance = float(self.unknown_adjacent_min_clearance_m)
        if not strict:
            relaxed_clearance = min(relaxed_clearance, 0.05)

        reject_reasons = {'distance': 0, 'blacklist': 0, 'clearance': 0}

        for idx, p in enumerate(centroids):
            dx = p.x - cx
            dy = p.y - cy
            dist = math.hypot(dx, dy)
            angle = math.atan2(dy, dx)

            if dist < min_goal_dist or dist > max_goal_dist:
                reject_reasons['distance'] += 1
                continue
            if self.is_blacklisted(p):
                reject_reasons['blacklist'] += 1
                continue

            unknown_cells = self.count_unknown_near(occupancy_grid, p.x, p.y)
            unknown_adjacent = unknown_cells >= int(self.unknown_adjacent_min_cells)
            clearance = self.clearance_at(
                clearance_map, res, ox, oy, width, height, p.x, p.y)
            min_clearance = relaxed_clearance if unknown_adjacent else base_min_clearance
            if clearance < min_clearance:
                reject_reasons['clearance'] += 1
                continue

            size = sizes[idx] if idx < len(sizes) else 1
            turn_penalty = 1.0 - math.cos(angle)

            # 雷达左右盲区：侧方 frontier 加软惩罚，但不直接丢弃
            abs_angle = abs(angle)
            blind_penalty = 0.0
            if math.radians(55) < abs_angle < math.radians(125):
                blind_penalty = float(self.blind_spot_penalty) * dist * 0.35

            # 软距离项：允许远端大簇赢过近处碎边界，仍略偏近
            dist_term = 0.18 * dist + 0.35 * turn_penalty * dist
            size_bonus = -float(self.size_weight) * min(size, 120)
            info_bonus = -float(self.info_gain_weight) * min(unknown_cells, 220)

            open_term = 0.0
            if self.prefer_open_space and not unknown_adjacent:
                w = float(self.open_space_weight)
                open_term = -w * min(clearance, 2.5)
                if clearance < float(self.min_open_clearance_m):
                    open_term += 2.0 * w * (
                        float(self.min_open_clearance_m) - clearance)

            score = dist_term + size_bonus + info_bonus + open_term + blind_penalty
            if score < best_score:
                best_score = score
                best = p

        if best is None and strict:
            total = len(centroids)
            self.get_logger().warn(
                f'严格目标选择失败: {total} 个候选, '
                f'dist过滤{reject_reasons["distance"]}, '
                f'黑名单{reject_reasons["blacklist"]}, '
                f'clearance过滤{reject_reasons["clearance"]}')
        elif best is not None:
            unk = self.count_unknown_near(occupancy_grid, best.x, best.y)
            self.get_logger().info(
                f'选中目标评分 {best_score:.2f}: ({best.x:.2f}, {best.y:.2f}), '
                f'未知邻接={unk}')
        return best

    def select_doorway_goal(self, centroids, sizes, current_pose, occupancy_grid):
        """门口优先：选前方 0.45~1.6m、通道偏窄但可通行的 frontier。"""
        if not centroids or current_pose is None:
            return None
        pose = self.get_current_pose_with_yaw()
        if pose is None:
            return None
        cx, cy, robot_yaw = pose
        clearance_map, res, ox, oy = self.build_clearance_map(occupancy_grid)
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        best = None
        best_score = float('inf')
        dist_max = float(self.doorway_preferred_dist_max)
        if self.no_goal_count >= 3:
            dist_max = min(self.effective_max_goal_distance(), 8.0)

        for idx, p in enumerate(centroids):
            if self.is_blacklisted(p) and self.no_goal_count < 6:
                continue
            dx = p.x - cx
            dy = p.y - cy
            dist = math.hypot(dx, dy)
            if dist < float(self.doorway_preferred_dist_min) or dist > dist_max:
                continue

            bearing = math.atan2(dy, dx)
            rel_angle = abs(self.normalize_angle(bearing - robot_yaw))
            if rel_angle > math.radians(40):
                continue

            clearance = self.clearance_at(clearance_map, res, ox, oy, width, height, p.x, p.y)
            if clearance < float(self.doorway_min_clearance_m):
                continue
            if clearance > float(self.doorway_max_clearance_m):
                continue

            size = sizes[idx] if idx < len(sizes) else 1
            score = dist + 1.5 * rel_angle - 0.4 * min(size, 40)
            if score < best_score:
                best_score = score
                best = p

        if best is not None:
            self.get_logger().info(
                f'门口优先目标: ({best.x:.2f}, {best.y:.2f}), 评分 {best_score:.2f}')
        return best

    def select_emergency_goal(self, centroids, current_pose):
        """兜底：选最近、未黑名单的 frontier，尽量让机器人先动起来。"""
        if not centroids or current_pose is None:
            return None
        cx, cy, _ = current_pose
        best = None
        best_dist = float('inf')
        max_dist = self.effective_max_goal_distance()
        all_blocked = all(self.is_blacklisted(p) for p in centroids)
        ignore_blacklist = self.no_goal_count >= 2 or all_blocked or len(centroids) <= 3
        for p in centroids:
            if not ignore_blacklist and self.is_blacklisted(p):
                continue
            dist = math.hypot(p.x - cx, p.y - cy)
            if dist < 0.10 or dist > max_dist:
                continue
            if dist < best_dist:
                best_dist = dist
                best = p
        if best is not None:
            self.get_logger().warn(
                f'启用兜底目标: ({best.x:.2f}, {best.y:.2f}), 距离 {best_dist:.2f}m')
        return best

    def should_keep_active_goal(self, current_pose):
        """目标承诺：未超时且仍在推进时，不频繁换点。"""
        if self.active_goal is None or current_pose is None:
            return False
        elapsed = time.monotonic() - self.active_goal_start_time
        if elapsed >= float(self.goal_commit_sec):
            return False
        if self.is_blacklisted(self.active_goal):
            return False
        if self.active_goal_start_pose is not None:
            moved = math.hypot(
                current_pose[0] - self.active_goal_start_pose[0],
                current_pose[1] - self.active_goal_start_pose[1])
            # 已经走了一段，继续坚持当前目标
            if moved >= float(self.goal_reselect_progress_m):
                return True
            # 几乎没动且已过一小段时间，允许换
            if elapsed > 8.0 and moved < 0.08:
                return False
        return True

    def navigate_to_pose(self, point, yaw=0.0):
        goal = PoseStamped()
        goal.header.frame_id = self.global_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = point.x
        goal.pose.position.y = point.y
        goal.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, yaw)
        goal.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.last_nav_infra_failure = False
        self.last_nav_stuck = False
        self.did_doorway_this_cycle = False
        if not self.nav_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('/navigate_to_pose Action Server 未就绪')
            self.last_nav_infra_failure = True
            return False

        self.current_goal = (point.x, point.y, yaw)
        self.last_pose = self.get_current_pose()
        self.last_pose_time = time.monotonic()

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal
        send_goal_future = self.nav_client.send_goal_async(nav_goal)
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and self.exploration_active and not send_goal_future.done():
            if time.monotonic() > deadline:
                self.get_logger().warn('发送导航目标超时')
                return False
            time.sleep(0.05)
        if not send_goal_future.done():
            return False
        try:
            goal_handle = send_goal_future.result()
        except Exception as exc:
            self.get_logger().warn(f'发送导航目标失败: {exc}')
            return False
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('目标被拒绝')
            return False

        self.get_logger().info(f'导航目标: ({point.x:.2f}, {point.y:.2f})')
        result_future = goal_handle.get_result_async()

        start_time = time.monotonic()
        while rclpy.ok() and self.exploration_active:
            if result_future.done():
                try:
                    wrapped = result_future.result()
                except Exception as exc:
                    self.get_logger().warn(f'获取导航结果失败: {exc}')
                    return False
                if wrapped is None:
                    return False
                return wrapped.status == GoalStatus.STATUS_SUCCEEDED

            elapsed = time.monotonic() - start_time
            if elapsed > self.max_navigation_time:
                self.get_logger().warn(f'导航目标超时 ({self.max_navigation_time}s)，取消')
                goal_handle.cancel_goal_async()
                return False

            if self.check_stuck():
                self.get_logger().warn('检测到卡住，取消当前目标')
                self.last_nav_stuck = True
                goal_handle.cancel_goal_async()
                if self.should_try_doorway_pass(point, self.get_current_pose()):
                    self.get_logger().warn('门口附近卡住，尝试对准慢速穿门')
                    self.record_doorway_attempt(point)
                    self.doorway_pass_maneuver(point)
                    self.did_doorway_this_cycle = True
                else:
                    self.get_logger().warn('穿门次数用尽，大角度对准后前探')
                    self.corner_escape_forward(point)
                return False

            time.sleep(0.1)

        goal_handle.cancel_goal_async()
        return False

    def check_stuck(self):
        pose = self.get_current_pose()
        if pose is None:
            return False
        now = time.monotonic()
        if self.last_pose is None:
            self.last_pose = pose
            self.last_pose_time = now
            return False
        moved = math.hypot(pose[0] - self.last_pose[0], pose[1] - self.last_pose[1])
        if moved > self.stuck_movement_radius:
            self.last_pose = pose
            self.last_pose_time = now
            return False
        if now - self.last_pose_time > self.stuck_timeout_sec:
            return True
        return False

    def recovery_maneuver(self, rotation_deg=None):
        if rotation_deg is None:
            rotation_deg = float(self.recovery_rotation)
        twist = Twist()
        # 后退
        backup = float(self.recovery_backup)
        twist.linear.x = -0.06
        self.cmd_vel_pub.publish(twist)
        time.sleep(max(0.1, backup / 0.06))
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.15)

        # 慢转，减轻 SLAM 旋转漂移（与手推建图速度接近）
        ang_speed = float(self.recovery_angular_speed)
        if rotation_deg < 0:
            ang_speed = -ang_speed
        twist.angular.z = ang_speed
        duration = abs(math.radians(rotation_deg)) / abs(ang_speed)
        end_time = time.monotonic() + duration
        while time.monotonic() < end_time and rclpy.ok():
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        settle = float(self.recovery_settle_sec)
        if settle > 0:
            time.sleep(settle)
        self.get_logger().info(
            f'恢复动作完成: 后退{backup:.2f}m, 旋转{rotation_deg:.1f}° @ {abs(ang_speed):.2f}rad/s')
        self.recovery_rotation_total += abs(float(rotation_deg))

    def doorway_pass_maneuver(self, goal):
        """门口窄通道：完整对准目标方向后低速直进。"""
        pose = self.get_current_pose_with_yaw()
        if pose is None or goal is None:
            return False
        cx, cy, robot_yaw = pose
        target_yaw = math.atan2(goal.y - cy, goal.x - cx)
        dyaw = self.normalize_angle(target_yaw - robot_yaw)

        self.get_logger().info(
            f'门口穿门: 目标({goal.x:.2f},{goal.y:.2f}), '
            f'对准偏差 {math.degrees(dyaw):.1f}°')

        # 大角度（如 180° 反向）必须转够，不能 8s 截断后朝错误方向前进
        if abs(dyaw) > math.radians(90):
            backup = float(self.recovery_backup)
            twist = Twist()
            twist.linear.x = -0.05
            self.cmd_vel_pub.publish(twist)
            time.sleep(max(0.1, backup / 0.05))
            twist.linear.x = 0.0
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.15)

        self.rotate_toward_yaw(target_yaw)

        speed = float(self.doorway_pass_speed)
        distance = float(self.doorway_pass_distance)
        self.drive_forward(distance, speed)
        self.get_logger().info(f'门口穿门完成: 直进 {distance:.2f}m @ {speed:.2f}m/s')
        return True

    def should_try_doorway_pass(self, goal, current_pose):
        if goal is None or current_pose is None:
            return False
        cx, cy = current_pose[0], current_pose[1]
        dist = math.hypot(goal.x - cx, goal.y - cy)
        max_dist = 5.0 if self.no_goal_count >= 2 or self.last_nav_stuck else 3.5
        if dist > max_dist:
            return False
        key = self.goal_key(goal)
        max_attempts = int(self.doorway_max_attempts) + (
            2 if self.no_goal_count >= 3 else 0)
        return self.doorway_pass_attempts.get(key, 0) < max_attempts

    def record_doorway_attempt(self, goal):
        key = self.goal_key(goal)
        self.doorway_pass_attempts[key] = self.doorway_pass_attempts.get(key, 0) + 1

    def forward_probe(self, occupancy_grid, current_pose):
        """朝未知区域方向慢速前进，扩展地图视野。"""
        if current_pose is None or occupancy_grid is None:
            return
        cx, cy, _ = current_pose
        data = np.array(occupancy_grid.data).reshape(
            occupancy_grid.info.height, occupancy_grid.info.width)
        res = occupancy_grid.info.resolution
        ox = occupancy_grid.info.origin.position.x
        oy = occupancy_grid.info.origin.position.y
        height, width = data.shape

        best_angle = 0.0
        best_score = -1.0
        for deg in range(-50, 51, 10):
            rad = math.radians(deg)
            score = 0
            for step in range(3, 14):
                px = cx + math.cos(rad) * step * res * 3
                py = cy + math.sin(rad) * step * res * 3
                mx = int((px - ox) / res)
                my = int((py - oy) / res)
                if mx < 0 or my < 0 or mx >= width or my >= height:
                    break
                if data[my, mx] == -1:
                    score += 1
                elif data[my, mx] >= 65:
                    score -= 3
                    break
            if score > best_score:
                best_score = score
                best_angle = rad

        # 先转向最佳方向
        if abs(best_angle) > 0.05:
            self.recovery_maneuver(math.degrees(best_angle))

        speed = float(self.forward_probe_speed)
        distance = float(self.forward_probe_distance)
        twist = Twist()
        twist.linear.x = speed
        duration = max(0.1, distance / speed)
        self.cmd_vel_pub.publish(twist)
        time.sleep(duration)
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(
            f'向前探路 {distance:.2f}m, 方向 {math.degrees(best_angle):.0f}°, 未知评分 {best_score}')

    def explore_recovery(self, occupancy_grid=None, current_pose=None):
        """无目标时的恢复：旋转观察 + 向前探路，不应轻易触发探索结束。"""
        self.get_logger().info(
            f'探索恢复: no_goal={self.no_goal_count}, no_frontier={self.no_frontier_count}, '
            f'累计旋转={self.recovery_rotation_total:.0f}°, '
            f'last={getattr(self, "last_recovery_type", "rotate")}')

        last = getattr(self, 'last_recovery_type', 'rotate')
        big_rot = float(self.no_frontier_rotate)
        phase = self.no_goal_count if self.no_goal_count > 0 else self.no_frontier_count

        if phase % 4 == 1:
            self.recovery_maneuver(30.0)
            self.last_recovery_type = 'rotate'
        elif phase % 4 == 2:
            self.recovery_maneuver(big_rot)
            self.last_recovery_type = 'rotate45'
        elif phase % 4 == 3:
            rot = big_rot if last != 'rotate45' else -big_rot
            self.recovery_maneuver(rot)
            self.last_recovery_type = 'rotate_big'
        else:
            self.forward_probe(occupancy_grid, current_pose)
            self.last_recovery_type = 'forward'

    def publish_frontiers(self, centroids):
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'centroids'
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.points = centroids
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.color.r = 1.0
        marker.color.a = 1.0
        self.centroid_marker_pub.publish(marker)

    def run_exploration(self):
        self.get_logger().info('等待 TF 与 /map 就绪...')
        while rclpy.ok() and self.exploration_active:
            pose = self.get_current_pose()
            with self.map_lock:
                has_map = self.current_map is not None
            if pose is not None and has_map:
                break
            time.sleep(1.0)

        if not (rclpy.ok() and self.exploration_active):
            self.get_logger().info('探索在就绪前被取消')
            return

        self.get_logger().info('开始自主探索主循环（优先大簇/邻接未知，窄门口放宽）')

        try:
            while rclpy.ok() and self.exploration_active and self.iteration < self.iteration_limit:
                self.iteration += 1
                self.iteration_pub.publish(Int16(data=self.iteration))

                with self.map_lock:
                    occupancy_grid = self.current_map
                if occupancy_grid is None:
                    time.sleep(0.5)
                    continue

                frontiers = self.detect_frontiers(occupancy_grid)
                centroids, sizes = self.cluster_frontiers(frontiers)
                self.publish_frontiers(centroids)
                self.get_logger().info(f'迭代 {self.iteration}: 发现 {len(centroids)} 个 frontier')

                if not centroids:
                    self.no_frontier_count += 1
                    self.no_goal_count = 0
                    # 只有地图上真的没有任何 frontier，且已转够一圈，才认为探索完成
                    if (self.no_frontier_count >= self.no_frontier_limit
                            and self.recovery_rotation_total >= 270.0):
                        self.get_logger().info(
                            f'连续 {self.no_frontier_count} 次无 frontier 且已扫描 '
                            f'{self.recovery_rotation_total:.0f}°，探索完成')
                        self.publish_finished('no_frontier')
                        break
                    self.explore_recovery(occupancy_grid, self.get_current_pose())
                    continue

                self.no_frontier_count = 0
                current_pose = self.get_current_pose()
                self.update_stuck_watchdog(current_pose)
                self.clear_blacklist_if_blocking_all(centroids)

                if self.stuck_iteration_count >= 5:
                    self.get_logger().warn(
                        f'同区域滞留 {self.stuck_iteration_count} 轮，执行角点脱困')
                    self.blacklist.clear()
                    escape_goal = self.active_goal
                    if escape_goal is None and centroids:
                        escape_goal = centroids[0]
                    self.corner_escape_forward(escape_goal)
                    self.stuck_iteration_count = 0
                    self.last_stuck_pose = current_pose[:2] if current_pose else None
                    continue

                # 目标承诺：减少频繁换点导致原地打转
                if self.should_keep_active_goal(current_pose):
                    goal = self.active_goal
                    self.get_logger().info(
                        f'坚持当前目标 ({goal.x:.2f}, {goal.y:.2f})')
                else:
                    goal = self.select_best_goal(
                        centroids, sizes, current_pose, occupancy_grid)
                    if goal is None:
                        self.get_logger().warn('严格目标选择失败，尝试回退目标选择...')
                        goal = self.select_best_goal(
                            centroids, sizes, current_pose, occupancy_grid, strict=False)
                    if goal is None:
                        goal = self.select_doorway_goal(
                            centroids, sizes, current_pose, occupancy_grid)
                    if goal is None:
                        goal = self.select_emergency_goal(centroids, current_pose)
                    if goal is not None:
                        if self.should_skip_pullback(goal, occupancy_grid):
                            self.get_logger().info(
                                f'邻接未知/窄门目标跳过回撤: ({goal.x:.2f}, {goal.y:.2f})')
                        else:
                            pulled = self.pullback_goal(goal, occupancy_grid)
                            if (abs(pulled.x - goal.x) > 1e-3
                                    or abs(pulled.y - goal.y) > 1e-3):
                                self.get_logger().info(
                                    f'目标回撤: ({goal.x:.2f}, {goal.y:.2f}) -> '
                                    f'({pulled.x:.2f}, {pulled.y:.2f})')
                            goal = pulled
                        self.active_goal = goal
                        self.active_goal_start_pose = current_pose
                        self.active_goal_start_time = time.monotonic()

                if goal is None:
                    self.no_goal_count += 1
                    self.maybe_trim_blacklist_for_stuck()
                    if self.no_goal_count >= self.no_goal_limit:
                        self.get_logger().warn(
                            f'连续 {self.no_goal_count} 次无法选出任何目标，探索结束')
                        self.publish_finished('no_reachable_frontier')
                        break
                    self.get_logger().warn(
                        f'仍无有效目标，执行探索恢复（已连续 {self.no_goal_count} 次）...')
                    self.explore_recovery(occupancy_grid, current_pose)
                    continue

                self.no_goal_count = 0
                goal_key = self.goal_key(goal)
                stuck_count = self.goal_stuck_counts.get(goal_key, 0)
                if stuck_count >= 4:
                    self.get_logger().warn(
                        f'目标 ({goal.x:.2f},{goal.y:.2f}) 穿门 {stuck_count} 次仍卡，'
                        f'改探索恢复（旋转+探路）')
                    self.goal_stuck_counts[goal_key] = 0
                    self.doorway_pass_attempts.pop(goal_key, None)
                    self.explore_recovery(occupancy_grid, current_pose)
                    self.active_goal = None
                    continue
                if stuck_count >= 2:
                    self.get_logger().warn(
                        f'目标 ({goal.x:.2f},{goal.y:.2f}) 已连续卡住 {stuck_count} 次，'
                        f'跳过 Nav2 直接穿门/前探')
                    self.corner_escape_forward(goal)
                    self.goal_stuck_counts[goal_key] = stuck_count + 1
                    self.active_goal = None
                    continue

                yaw = math.atan2(goal.y - current_pose[1], goal.x - current_pose[0])
                success = self.navigate_to_pose(goal, yaw)

                if success:
                    self.get_logger().info(f'到达目标 ({goal.x:.2f}, {goal.y:.2f})')
                    self.add_to_blacklist(goal.x, goal.y, yaw)
                    self.doorway_pass_attempts.pop(goal_key, None)
                    self.goal_stuck_counts.pop(goal_key, None)
                    self.active_goal = None
                elif self.last_nav_infra_failure:
                    self.get_logger().warn(
                        f'Nav2 未就绪，跳过黑名单: ({goal.x:.2f}, {goal.y:.2f})')
                    self.active_goal = None
                    time.sleep(3.0)
                elif self.last_nav_stuck:
                    self.goal_stuck_counts[goal_key] = stuck_count + 1
                    self.get_logger().warn(
                        f'导航卡住，不加入黑名单: ({goal.x:.2f}, {goal.y:.2f}), '
                        f'累计 {self.goal_stuck_counts[goal_key]} 次')
                    if not self.did_doorway_this_cycle:
                        if self.should_try_doorway_pass(goal, current_pose):
                            self.record_doorway_attempt(goal)
                            self.doorway_pass_maneuver(goal)
                        else:
                            self.corner_escape_forward(goal)
                    self.active_goal = None
                else:
                    self.get_logger().warn(f'目标失败或超时: ({goal.x:.2f}, {goal.y:.2f})')
                    dist = math.hypot(goal.x - current_pose[0], goal.y - current_pose[1])
                    if self.should_try_doorway_pass(goal, current_pose):
                        self.get_logger().warn('导航失败但目标较近，尝试门口穿门')
                        self.record_doorway_attempt(goal)
                        self.doorway_pass_maneuver(goal)
                    elif dist > 4.0:
                        self.add_to_blacklist(goal.x, goal.y, yaw)
                    self.active_goal = None

            if self.iteration >= self.iteration_limit and self.exploration_active:
                self.get_logger().info(f'达到迭代上限 {self.iteration_limit}，探索完成')
                self.publish_finished('iteration_limit')
        except Exception as exc:
            self.get_logger().error(f'探索主循环异常: {exc}')
            self.publish_finished(f'exception:{exc}')
        finally:
            self.exploration_active = False
            self.get_logger().info('自主探索主循环结束')

    def stop(self):
        self.exploration_active = False
        self.active_goal = None


def main(args=None):
    rclpy.init(args=args)
    node = ExploreNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        if node.exploration_thread.is_alive():
            node.exploration_thread.join(timeout=2.0)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
