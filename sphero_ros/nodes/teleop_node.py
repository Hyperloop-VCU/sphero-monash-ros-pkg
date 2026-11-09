import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class SpheroTwistTeleopNode(Node):
    """Keyboard teleop ROS2 node that publishes Twist on cmd_vel."""

    def __init__(self):
        super().__init__("sphero_twist_teleop")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("speed_norm", 0.25)
        self.declare_parameter("speed_step", 0.1)
        self.declare_parameter("linear_scale", 0.5)
        self.declare_parameter("key_timeout", 0.25)

        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.speed_norm = float(self.get_parameter("speed_norm").value)
        self.speed_step = float(self.get_parameter("speed_step").value)
        self.linear_scale = float(self.get_parameter("linear_scale").value)
        self.key_timeout = float(self.get_parameter("key_timeout").value)
        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.get_logger().info(
            f"Teleop node started. Publishing Twist on {self.cmd_vel_topic}."
        )

    def publish(self, linear_x: float, linear_y: float):
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        self.publisher.publish(msg)


def _read_key(timeout: float = 0.1):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if ready:
        return sys.stdin.read(1)
    return None


def main(args=None):
    rclpy.init(args=args)
    node = SpheroTwistTeleopNode()

    print("Keyboard teleop started")
    print("q/w/e: forward-left/forward/forward-right")
    print("a/s/d: left/backward/right")
    print("+/-: change speed")
    print("space: stop")
    print("Q: quit")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    linear_x_command = 0.0
    linear_y_command = 0.0
    last_input_time = 0.0
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = _read_key()
            now = time.monotonic()

            if key in {"q", "w", "e", "a", "s", "d"}:
                command_vectors = {
                    "q": (1.0, 1.0),
                    "w": (1.0, 0.0),
                    "e": (1.0, -1.0),
                    "a": (0.0, 1.0),
                    "s": (-1.0, 0.0),
                    "d": (0.0, -1.0),
                }
                vector_x, vector_y = command_vectors[key]
                magnitude = node.speed_norm * node.linear_scale
                if vector_x != 0.0 and vector_y != 0.0:
                    magnitude /= 2.0**0.5
                linear_x_command = magnitude * vector_x
                linear_y_command = magnitude * vector_y
                last_input_time = now
            elif key in ("+", "="):
                node.speed_norm = min(1.0, node.speed_norm + node.speed_step)
                print(f"Speed: {node.speed_norm:.1f}")
            elif key == "-":
                node.speed_norm = max(0.0, node.speed_norm - node.speed_step)
                print(f"Speed: {node.speed_norm:.1f}")
            elif key == " ":
                linear_x_command = 0.0
                linear_y_command = 0.0
                node.publish(0.0, 0.0)
            elif key == "Q":
                node.publish(0.0, 0.0)
                break

            if now - last_input_time > node.key_timeout:
                linear_x_command = 0.0
                linear_y_command = 0.0

            node.publish(linear_x_command, linear_y_command)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
