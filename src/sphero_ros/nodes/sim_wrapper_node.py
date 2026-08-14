import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from sphero_env.envs.maze_generator import MazeGenerator, grid_to_world
from sphero_env.envs.sphero_env import SpheroEnv
from sphero_ros.conversions import (
    env_heading_to_ros_yaw,
    quaternion_from_yaw,
    ros_velocity_to_env_action,
    wrap_angle,
)


class SpheroSimWrapperNode(Node):
    @staticmethod
    def _grid_index_to_world(i: int, j: int, h: int, w: int, resolution: float) -> tuple[float, float]:
        x = (float(j) + 0.5 - (w / 2.0)) * resolution
        y = ((h / 2.0) - (float(i) + 0.5)) * resolution
        return x, y

    @staticmethod
    def _choose_safe_spawn_xy(
        occupancy_grid: np.ndarray,
        resolution: float,
        min_clearance_m: float,
    ) -> tuple[float, float, tuple[int, int], float]:
        h, w = occupancy_grid.shape
        free_cells = np.argwhere(occupancy_grid == 0)
        if free_cells.size == 0:
            raise RuntimeError("No free cells in occupancy grid for simulator spawn.")

        occupied_cells = np.argwhere(occupancy_grid == 1)
        if occupied_cells.size == 0:
            # No walls: pick center-most free cell.
            center = np.array([h / 2.0, w / 2.0], dtype=np.float32)
            distances = np.sum((free_cells.astype(np.float32) - center) ** 2, axis=1)
            i, j = free_cells[int(np.argmin(distances))]
            clearance_cells = float("inf")
        else:
            # Maximize minimum distance-to-wall so startup is robust even with inflated footprint.
            min_clearance_cells = max(0.0, float(min_clearance_m) / resolution)
            best_idx = 0
            best_clearance = -1.0
            best_valid_idx = None
            best_valid_clearance = -1.0
            for idx, (fi, fj) in enumerate(free_cells):
                deltas = occupied_cells.astype(np.float32) - np.array([fi, fj], dtype=np.float32)
                d2 = np.sum(deltas * deltas, axis=1)
                min_d2 = float(np.min(d2))
                if min_d2 > best_clearance:
                    best_clearance = min_d2
                    best_idx = idx
                min_d = float(np.sqrt(min_d2))
                if min_d >= min_clearance_cells and min_d2 > best_valid_clearance:
                    best_valid_clearance = min_d2
                    best_valid_idx = idx

            chosen_idx = best_valid_idx if best_valid_idx is not None else best_idx
            i, j = free_cells[chosen_idx]
            chosen_clearance_sq = best_valid_clearance if best_valid_idx is not None else best_clearance
            clearance_cells = float(np.sqrt(chosen_clearance_sq))

        x, y = SpheroSimWrapperNode._grid_index_to_world(int(i), int(j), h, w, resolution)
        return x, y, (int(i), int(j)), clearance_cells

    def __init__(self):
        super().__init__("sphero_sim_wrapper")

        self.declare_parameter("dt", 0.01)
        self.declare_parameter("max_steps", 5000)
        self.declare_parameter("vel_limit", 0.5)
        self.declare_parameter("use_default_maze", True)
        self.declare_parameter("maze_grid_size", 25)
        self.declare_parameter("maze_resolution", 0.1)
        self.declare_parameter("maze_seed", 0)
        self.declare_parameter("spawn_min_clearance_m", 0.20)
        self.declare_parameter("world_width", 2.0)
        self.declare_parameter("world_height", 2.0)
        self.declare_parameter("goal_x", 0.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("pose_topic", "/sphero/pose")
        self.declare_parameter("odom_topic", "/sphero/odom")
        self.declare_parameter("map_topic", "/sphero/map")
        self.declare_parameter("collision_topic", "/sphero/collision")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        dt = float(self.get_parameter("dt").value)
        max_steps = int(self.get_parameter("max_steps").value)
        vel_limit = float(self.get_parameter("vel_limit").value)
        use_default_maze = bool(self.get_parameter("use_default_maze").value)
        maze_grid_size = int(self.get_parameter("maze_grid_size").value)
        maze_resolution = float(self.get_parameter("maze_resolution").value)
        maze_seed = int(self.get_parameter("maze_seed").value)
        spawn_min_clearance_m = float(self.get_parameter("spawn_min_clearance_m").value)
        world_width = float(self.get_parameter("world_width").value)
        world_height = float(self.get_parameter("world_height").value)
        goal_x = float(self.get_parameter("goal_x").value)
        goal_y = float(self.get_parameter("goal_y").value)

        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.collision_topic = str(self.get_parameter("collision_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

        occupancy_grid = None
        grid_resolution = maze_resolution
        start_xy = (0.0, 0.0)
        start_idx = None
        start_clearance_m = None
        if use_default_maze:
            generator = MazeGenerator(width=maze_grid_size, height=maze_grid_size, seed=maze_seed)
            occupancy_grid, _start_cell, goal_cell = generator.generate()
            world_width = occupancy_grid.shape[1] * grid_resolution
            world_height = occupancy_grid.shape[0] * grid_resolution
            sx, sy, start_idx, start_clearance_cells = self._choose_safe_spawn_xy(
                occupancy_grid,
                grid_resolution,
                spawn_min_clearance_m,
            )
            start_xy = (sx, sy)
            start_clearance_m = start_clearance_cells * grid_resolution
            goal_x, goal_y = grid_to_world(
                goal_cell,
                maze_width=occupancy_grid.shape[1],
                maze_height=occupancy_grid.shape[0],
                cell_size=grid_resolution,
            )
            self.get_logger().info(
                f"Loaded default maze {occupancy_grid.shape[1]}x{occupancy_grid.shape[0]} at {grid_resolution:.3f} m/cell"
            )
            self.get_logger().info(
                f"Selected safe spawn at grid {start_idx} -> world ({start_xy[0]:.3f}, {start_xy[1]:.3f})"
            )
            if start_clearance_m is not None:
                self.get_logger().info(
                    f"Spawn clearance to nearest wall: {start_clearance_m:.3f} m (target >= {spawn_min_clearance_m:.3f} m)"
                )

        self.env = SpheroEnv(
            dt=dt,
            max_steps=max_steps,
            vel_limit=vel_limit,
            world_width=world_width,
            world_height=world_height,
            goal_pos=(goal_x, goal_y),
            occupancy_grid=occupancy_grid,
            grid_resolution=grid_resolution,
            obs_noise_std_pos=0.0,
            obs_noise_std_vel=0.0,
            process_noise_std_heading=0.0,
            process_noise_std_speed=0.0,
            render_mode=None,
        )
        self.env.reset(options={"init_pose": [start_xy[0], start_xy[1], 0.0]})
        if self.env._is_collision(start_xy[0], start_xy[1]):
            raise RuntimeError("Spawn pose resolved to an occupied cell; aborting startup.")

        self._cmd_linear_x: float = 0.0
        self._cmd_linear_y: float = 0.0
        self._last_published_yaw: Optional[float] = None

        self._pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self._odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self._collision_pub = self.create_publisher(Bool, self.collision_topic, 10)
        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        self._map_pub = self.create_publisher(OccupancyGrid, self.map_topic, map_qos)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._publish_map()

        self.create_subscription(Twist, self.cmd_vel_topic, self._on_twist, 10)
        self._timer = self.create_timer(dt, self._on_timer)

        self.get_logger().info(
            f"Sim wrapper started. Subscribing {self.cmd_vel_topic}, publishing "
            f"{self.pose_topic}, {self.odom_topic}, {self.map_topic}, and {self.collision_topic}."
        )

    def _publish_map(self):
        if self.env.occupancy_grid is None:
            return

        grid = self.env.occupancy_grid
        height, width = grid.shape
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = self.map_frame
        map_msg.info.map_load_time = map_msg.header.stamp
        map_msg.info.resolution = float(self.env.grid_resolution)
        map_msg.info.width = int(width)
        map_msg.info.height = int(height)
        map_msg.info.origin.position.x = -0.5 * width * self.env.grid_resolution
        map_msg.info.origin.position.y = -0.5 * height * self.env.grid_resolution
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        # Env indexing is top-to-bottom; OccupancyGrid expects bottom-to-top row order.
        data = np.flipud(grid).astype(np.int8)
        data = np.where(data > 0, 100, 0)
        map_msg.data = data.reshape(-1).tolist()
        self._map_pub.publish(map_msg)

    def _on_twist(self, msg: Twist):
        self._cmd_linear_x = float(msg.linear.x)
        self._cmd_linear_y = float(msg.linear.y)

    def _on_timer(self):
        speed, env_heading = ros_velocity_to_env_action(
            self._cmd_linear_x, self._cmd_linear_y
        )
        action = np.array([speed, env_heading], dtype=np.float32)

        _, _, _, _, info = self.env.step(action)

        self._publish_state(info)

    def _publish_state(self, info):
        state_odom = info["state_odom"]
        x, y, heading_env, speed = [float(v) for v in state_odom]
        yaw_ros = env_heading_to_ros_yaw(heading_env)
        qx, qy, qz, qw = quaternion_from_yaw(yaw_ros)

        stamp = self.get_clock().now().to_msg()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.map_frame
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self._pose_pub.publish(pose_msg)

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.map_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose = pose_msg.pose
        odom_msg.twist.twist.linear.x = speed * math.cos(yaw_ros)
        odom_msg.twist.twist.linear.y = speed * math.sin(yaw_ros)
        if self._last_published_yaw is None:
            odom_msg.twist.twist.angular.z = 0.0
        else:
            yaw_delta = wrap_angle(yaw_ros - self._last_published_yaw)
            odom_msg.twist.twist.angular.z = yaw_delta / self.env.dt
        self._last_published_yaw = yaw_ros
        self._odom_pub.publish(odom_msg)

        collision_msg = Bool()
        collision_msg.data = bool(info.get("collision", False))
        self._collision_pub.publish(collision_msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.map_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf_msg)

    def destroy_node(self):
        self.env.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpheroSimWrapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
