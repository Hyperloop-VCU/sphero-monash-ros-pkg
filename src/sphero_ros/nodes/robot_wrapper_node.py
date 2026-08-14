import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from sphero_env.robot.connect import scan_and_connect
from sphero_env.robot.robot import Robot
from sphero_ros.conversions import env_heading_to_ros_yaw, quaternion_from_yaw, ros_yaw_to_env_heading, wrap_angle


class SpheroRobotWrapperNode(Node):
    def __init__(self):
        super().__init__("sphero_robot_wrapper")

        self.declare_parameter("dt", 0.01)
        self.declare_parameter("max_steps", 5000)
        self.declare_parameter("vel_limit", 0.5)
        self.declare_parameter("goal_x", 0.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("pose_topic", "/sphero/pose")
        self.declare_parameter("odom_topic", "/sphero/odom")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        dt = float(self.get_parameter("dt").value)
        max_steps = int(self.get_parameter("max_steps").value)
        vel_limit = float(self.get_parameter("vel_limit").value)
        goal_x = float(self.get_parameter("goal_x").value)
        goal_y = float(self.get_parameter("goal_y").value)

        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

        toy, api_context = scan_and_connect()
        self.get_logger().info(f"Connected to {toy.name}")
        self._api_context = api_context
        self._api = self._api_context.__enter__()

        self.robot = Robot(
            api=self._api,
            dt=dt,
            max_steps=max_steps,
            vel_limit=vel_limit,
            goal_pos=(goal_x, goal_y),
            render_mode=None,
        )
        self.robot.reset()

        self._cmd_linear_x: float = 0.0
        self._cmd_angular_z: float = 0.0
        self._target_yaw: float = 0.0
        self._last_published_yaw: Optional[float] = None

        self._pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self._odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Twist, self.cmd_vel_topic, self._on_twist, 10)
        self._timer = self.create_timer(dt, self._on_timer)

        self.get_logger().info(
            f"Robot wrapper started. Subscribing {self.cmd_vel_topic}, publishing {self.pose_topic} and {self.odom_topic}."
        )

    def _on_twist(self, msg: Twist):
        self._cmd_linear_x = float(msg.linear.x)
        self._cmd_angular_z = float(msg.angular.z)

    def _on_timer(self):
        self._target_yaw = wrap_angle(self._target_yaw + self._cmd_angular_z * self.robot.dt)
        env_heading = ros_yaw_to_env_heading(self._target_yaw)
        action = np.array([self._cmd_linear_x, env_heading], dtype=np.float32)

        _, _, _, _, info = self.robot.step(action)

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
            odom_msg.twist.twist.angular.z = yaw_delta / self.robot.dt
        self._last_published_yaw = yaw_ros
        self._odom_pub.publish(odom_msg)

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
        self.robot.emergency_stop()
        self.robot.close()
        if self._api_context is not None:
            self._api_context.__exit__(None, None, None)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpheroRobotWrapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
