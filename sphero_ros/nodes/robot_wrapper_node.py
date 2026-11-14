import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, ColorRGBA, String
from tf2_ros import TransformBroadcaster
from sphero_unsw.commands.io import IO
from sphero_unsw.controls.v2 import Processors
from sphero_unsw.toy.boltplus import BOLTPLUS
from sphero_unsw.types import Color

BOLTPLUS_BACK_LED_CHANNELS = range(0, 9)
BOLTPLUS_FRONT_LED_CHANNELS = range(9, 18)

from sphero_env.robot.connect import scan_and_connect
from sphero_env.robot.robot import Robot
from sphero_ros.conversions import (
    env_heading_to_ros_yaw,
    quaternion_from_euler,
    quaternion_from_yaw,
    ros_velocity_to_env_action,
    wrap_angle,
)


class SpheroRobotWrapperNode(Node):
    def __init__(self, api):
        super().__init__("sphero_robot_wrapper")

        self.declare_parameter("dt", 0.01)
        self.declare_parameter("max_steps", 5000)
        self.declare_parameter("vel_limit", 0.5)
        self.declare_parameter("goal_x", 0.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("pose_topic", "/sphero/pose")
        self.declare_parameter("odom_topic", "/sphero/odom")
        self.declare_parameter("imu_topic", "/sphero/imu")
        self.declare_parameter("collision_topic", "/sphero/collision")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("led_topic_prefix", "/sphero/led")

        dt =float(self.get_parameter("dt").value)
        max_steps = int(self.get_parameter("max_steps").value)
        vel_limit = float(self.get_parameter("vel_limit").value)
        goal_x = float(self.get_parameter("goal_x").value)
        goal_y = float(self.get_parameter("goal_y").value)

        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.collision_topic = str(self.get_parameter("collision_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

        self._api = api
        self.get_logger().info("Sphero API session opened; initializing robot state...")

        self.robot = Robot(
            api=self._api,
            dt=dt,
            max_steps=max_steps,
            vel_limit=vel_limit,
            goal_pos=(goal_x, goal_y),
            render_mode=None,
        )
        self.robot.reset()
        self.get_logger().info("Robot state initialized; starting ROS publishers and timer...")

        self._cmd_linear_x: float = 0.0
        self._cmd_linear_y: float = 0.0
        self._last_published_yaw: Optional[float] = None

        self._pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self._odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self._imu_pub = self.create_publisher(Imu, self.imu_topic, 10)
        self._collision_pub = self.create_publisher(Bool, self.collision_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Twist, self.cmd_vel_topic, self._on_twist, 10)

        # LED control. Colours are std_msgs/ColorRGBA with r, g, b in [0, 1];
        # alpha is ignored. The matrix character topic takes a single character.
        led_prefix = str(self.get_parameter("led_topic_prefix").value).rstrip("/")
        self._matrix_color = Color(255, 255, 255)
        led_handlers = {
            "main": lambda c: self._api.set_main_led(c),
            "front": self._set_front_led,
            "back": self._set_back_led,
            "matrix": self._fill_matrix,
        }
        for name, handler in led_handlers.items():
            self.create_subscription(
                ColorRGBA,
                f"{led_prefix}/{name}",
                lambda msg, n=name, h=handler: self._on_led_color(n, h, msg),
                10,
            )
        self.create_subscription(
            String, f"{led_prefix}/matrix_char", self._on_matrix_char, 10
        )

        self._timer = self.create_timer(dt, self._on_timer)

        self.get_logger().info(
            f"Robot wrapper started. Subscribing {self.cmd_vel_topic}, publishing "
            f"{self.pose_topic}, {self.odom_topic}, {self.imu_topic}, and {self.collision_topic}. "
            f"LED topics under {led_prefix}/(main|front|back|matrix|matrix_char)."
        )

    def _on_twist(self, msg: Twist):
        self._cmd_linear_x = float(msg.linear.x)
        self._cmd_linear_y = float(msg.linear.y)

    @staticmethod
    def _to_color(msg: ColorRGBA) -> Color:
        def channel(v: float) -> int:
            return int(round(min(max(float(v), 0.0), 1.0) * 255))

        return Color(channel(msg.r), channel(msg.g), channel(msg.b))

    def _raw_toy(self):
        return getattr(self._api, "_SpheroEduAPI__toy", None)

    def _set_boltplus_leds(self, channels: range, color: Color):
        """Set a group of BOLT+ LEDs with one 32-bit-mask command."""
        mask = 0
        for channel in channels:
            mask |= 1 << channel
        # Values go in ascending channel order: r, g, b for each LED.
        values = [color.r, color.g, color.b] * (len(channels) // 3)
        IO.set_all_leds_with_32_bit_mask(
            self._raw_toy(), mask, values, proc=Processors.PRIMARY
        )

    def _set_front_led(self, color: Color):
        if isinstance(self._raw_toy(), BOLTPLUS):
            self._set_boltplus_leds(BOLTPLUS_FRONT_LED_CHANNELS, color)
        else:
            self._api.set_front_led(color)

    def _set_back_led(self, color: Color):
        if isinstance(self._raw_toy(), BOLTPLUS):
            self._set_boltplus_leds(BOLTPLUS_BACK_LED_CHANNELS, color)
        else:
            self._api.set_back_led(color)

    def _fill_matrix(self, color: Color):
        self._matrix_color = color
        self._api.set_matrix_fill(0, 0, 7, 7, color)

    def _on_led_color(self, name: str, handler, msg: ColorRGBA):
        try:
            handler(self._to_color(msg))
        except Exception as exc:  # keep driving even if an LED call fails
            self.get_logger().warn(f"Failed to set {name} LED: {exc}")

    def _on_matrix_char(self, msg: String):
        if not msg.data:
            try:
                self._api.clear_matrix()
            except Exception as exc:
                self.get_logger().warn(f"Failed to clear matrix: {exc}")
            return
        try:
            self._api.set_matrix_character(msg.data[0], self._matrix_color)
        except Exception as exc:
            self.get_logger().warn(f"Failed to set matrix character: {exc}")

    def _on_timer(self):
        speed, env_heading = ros_velocity_to_env_action(
            self._cmd_linear_x, self._cmd_linear_y
        )
        action = np.array([speed, env_heading], dtype=np.float32)

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

        self._publish_imu(info, stamp)

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

    def _publish_imu(self, info, stamp):
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.base_frame

        acceleration = info.get("acceleration") or {}
        imu_msg.linear_acceleration.x = float(acceleration.get("x", 0.0)) * 9.80665
        imu_msg.linear_acceleration.y = float(acceleration.get("y", 0.0)) * 9.80665
        imu_msg.linear_acceleration.z = float(acceleration.get("z", 0.0)) * 9.80665

        gyroscope = info.get("gyroscope") or {}
        imu_msg.angular_velocity.x = math.radians(float(gyroscope.get("x", gyroscope.get("pitch", 0.0))))
        imu_msg.angular_velocity.y = math.radians(float(gyroscope.get("y", gyroscope.get("roll", 0.0))))
        imu_msg.angular_velocity.z = math.radians(float(gyroscope.get("z", gyroscope.get("yaw", 0.0))))

        orientation = info.get("orientation") or {}
        roll = math.radians(float(orientation.get("roll", orientation.get("x", 0.0))))
        pitch = math.radians(float(orientation.get("pitch", orientation.get("y", 0.0))))
        yaw = math.radians(float(orientation.get("yaw", orientation.get("z", 0.0))))
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw

        self._imu_pub.publish(imu_msg)

    def destroy_node(self):
        self.robot.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        toy, api_context = scan_and_connect()
        print(f"Connected to {toy.name}", flush=True)
        with api_context as api:
            node = SpheroRobotWrapperNode(api)
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                pass
            finally:
                node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
