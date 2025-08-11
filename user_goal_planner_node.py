#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PointStamped, PoseStamped
import tf2_ros
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from tf2_geometry_msgs import do_transform_point
from rclpy.time import Time
import heapq

import numpy as np
import cv2
from scipy.interpolate import splprep, splev

class UserGoalPlanner(Node):
    def __init__(self):
        super().__init__('user_goal_planner')

        # 파라미터 선언
        self.declare_parameter('map_topic',   '/pointcloud_2d_map')
        self.declare_parameter('goal_topic',  '/user_tracking')
        self.declare_parameter('path_topic',  '/global_plan')
        self.declare_parameter('base_frame',  'camera_base')
        self.declare_parameter('robot_width',  0.9)
        self.declare_parameter('robot_length', 1.2)
        self.declare_parameter('start_x',      0.0)  # fallback
        self.declare_parameter('start_y',      0.0)  # fallback

        map_topic   = self.get_parameter('map_topic').value
        goal_topic  = self.get_parameter('goal_topic').value
        path_topic  = self.get_parameter('path_topic').value
        self.base_frame   = self.get_parameter('base_frame').value
        self.robot_width  = self.get_parameter('robot_width').value
        self.robot_length = self.get_parameter('robot_length').value
        self.start_x      = self.get_parameter('start_x').value
        self.start_y      = self.get_parameter('start_y').value

        # TF setup
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS for Path publisher (Transient Local)
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # Subscriptions / Publisher
        self.map_sub  = self.create_subscription(OccupancyGrid, map_topic, self.map_cb, 10)
        self.goal_sub = self.create_subscription(PointStamped,   goal_topic, self.goal_cb, 10)
        self.path_pub = self.create_publisher(Path, path_topic, qos)

        self.latest_map   = None
        self.current_goal = None
        self.last_goal    = None

    def map_cb(self, msg: OccupancyGrid):
        self.get_logger().info('map received')
        self.latest_map = msg
        self.try_plan()

    def goal_cb(self, msg: PointStamped):
        try:
            trans = self.tf_buffer.lookup_transform('map', msg.header.frame_id, msg.header.stamp)
            goal_map = do_transform_point(msg, trans)
            goal_map.header.frame_id = 'map'
            self.current_goal = goal_map
            self.last_goal    = goal_map
        except Exception:
            # fallback: assume point in map frame
            fallback = PointStamped()
            fallback.header.frame_id = 'map'
            fallback.header.stamp    = self.get_clock().now().to_msg()
            fallback.point = msg.point
            self.current_goal = fallback
            self.last_goal    = fallback
            self.get_logger().warn('goal transform failed, using fallback')
        self.try_plan()

    def try_plan(self):
        if self.latest_map is None:
            return
        if self.current_goal is None and self.last_goal is None:
            return
        if self.current_goal is None:
            self.current_goal = self.last_goal

        grid     = self.latest_map
        frame    = grid.header.frame_id
        w, h     = grid.info.width, grid.info.height
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y
        res      = grid.info.resolution

        # 1) Inflate obstacles
        grid_mat     = np.array(grid.data, dtype=np.int8).reshape(h, w)
        obs          = (grid_mat == 100).astype(np.uint8)
        half_w       = self.robot_width  / 2.0
        half_l       = self.robot_length / 2.0
        cells_w      = int(np.ceil(half_w / res))
        cells_l      = int(np.ceil(half_l / res))
        kernel       = np.ones((2*cells_l+1, 2*cells_w+1), dtype=np.uint8)
        obs_inflated = cv2.dilate(obs, kernel)
        grid_mat[obs_inflated == 1] = 100
        grid.data   = grid_mat.flatten().tolist()

        # 2) Determine start point with fallback
        try:
            trans_start = self.tf_buffer.lookup_transform(frame, self.base_frame, Time())
            start_x = trans_start.transform.translation.x
            start_y = trans_start.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f'start transform failed: {e}, falling back')
            start_x = self.start_x
            start_y = self.start_y

        # 3) Transform goal to grid frame
        try:
            trans_goal = self.tf_buffer.lookup_transform(
                frame, self.current_goal.header.frame_id, Time()
            )
            goal_grid   = do_transform_point(self.current_goal, trans_goal)
            goal_x, goal_y = goal_grid.point.x, goal_grid.point.y
        except Exception as e:
            self.get_logger().warn(f'goal->grid transform failed: {e}, using direct coords')
            # 직접 좌표 사용
            goal_x = self.current_goal.point.x
            goal_y = self.current_goal.point.y

        # 4) Convert to grid indices
        start_idx = self.world_to_grid(grid, start_x, start_y)
        goal_idx  = self.world_to_grid(grid, goal_x, goal_y)
        self.get_logger().info(f'start_idx={start_idx}, goal_idx={goal_idx}')

        # 5) Clear goal cell + neighbors
        gi, gj   = goal_idx
        idx_goal = gj * w + gi
        grid.data[idx_goal] = 0
        r = int(0.5 / res)
        for dx in range(-r, r+1):
            for dy in range(-r, r+1):
                xi, yj = gi+dx, gj+dy
                if 0 <= xi < w and 0 <= yj < h:
                    grid.data[yj*w + xi] = 0

        # 6) Bounds check
        if not (0 <= start_idx[0] < w and 0 <= start_idx[1] < h):
            return
        if not (0 <= goal_idx[0] < w and 0 <= goal_idx[1] < h):
            return

        # 7) A* search
        raw_path = self.astar(grid, start_idx, goal_idx)
        if not raw_path:
            self.get_logger().warn('no path')
            return

        # 8) Convert to world points
        world_pts = [(origin_x + (i+0.5)*res, origin_y + (j+0.5)*res) for i,j in raw_path]

        # 9) Spline smoothing
        try:
            smooth_pts = self.smooth_spline(world_pts)
        except Exception as e:
            self.get_logger().warn(f'spline smoothing failed: {e}')
            smooth_pts = world_pts

        # 10) Publish Path
        plan = Path()
        plan.header.frame_id = frame
        plan.header.stamp    = self.get_clock().now().to_msg()
        for x, y in smooth_pts:
            ps = PoseStamped()
            ps.header = plan.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            plan.poses.append(ps)
        self.path_pub.publish(plan)

    def world_to_grid(self, grid: OccupancyGrid, x: float, y: float):
        i = int((x - grid.info.origin.position.x) / grid.info.resolution)
        j = int((y - grid.info.origin.position.y) / grid.info.resolution)
        return (i, j)

    def astar(self, grid: OccupancyGrid, start, goal):
        w, h = grid.info.width, grid.info.height
        data = grid.data
        nbrs = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]
        def h_cost(a,b): return abs(a[0]-b[0]) + abs(a[1]-b[1])

        open_set = [(h_cost(start,goal), 0, start)]
        came_from, g_score, visited = {}, {start:0}, set()
        while open_set:
            f, g, cur = heapq.heappop(open_set)
            if cur in visited: continue
            visited.add(cur)
            if cur == goal: break
            for dx, dy in nbrs:
                nbr = (cur[0]+dx, cur[1]+dy)
                if not (0 <= nbr[0] < w and 0 <= nbr[1] < h): continue
                if data[nbr[1]*w + nbr[0]] == 100: continue
                cost = g + (1.4 if dx and dy else 1.0)
                if cost < g_score.get(nbr, float('inf')):
                    g_score[nbr] = cost
                    heapq.heappush(open_set, (cost + h_cost(nbr,goal), cost, nbr))
                    came_from[nbr] = cur
        if goal not in came_from:
            return []
        path, node = [], goal
        while node:
            path.append(node)
            node = came_from.get(node)
        return path[::-1]

    def smooth_spline(self, pts, smooth_factor=0, resolution_factor=10):
        if len(pts) < 2:
            return pts
        xs, ys = zip(*pts)
        tck, u = splprep([xs, ys], s=smooth_factor)
        unew   = np.linspace(0, 1, len(xs)*resolution_factor)
        out    = splev(unew, tck)
        return list(zip(out[0], out[1]))


def main(args=None):
    rclpy.init(args=args)
    node = UserGoalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
