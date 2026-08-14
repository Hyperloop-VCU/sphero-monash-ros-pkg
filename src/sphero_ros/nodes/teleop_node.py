import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class SpheroTwistTeleopNode(Node):
    """Keyboard teleop ROS2 node that publishes Twist on cmd_vel."""

    def __init__(self):
        super().__init__("sphero_twist_teleop")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("linear_speed", 0.2)
        self.declare_parameter("angular_speed", 1.0)

        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.get_logger().info(
            f"Teleop node started. Publishing Twist on {self.cmd_vel_topic}."
        )

    def publish(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
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
    print("w/s: forward/backward")
    print("a/d: left/right turn")
    print("space: stop")
    print("q: quit")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = _read_key()
            if key is None:
                continue

            if key == "w":
                node.publish(node.linear_speed, 0.0)
            elif key == "s":
                node.publish(-node.linear_speed, 0.0)
            elif key == "a":
                node.publish(0.0, node.angular_speed)
            elif key == "d":
                node.publish(0.0, -node.angular_speed)
            elif key == " ":
                node.publish(0.0, 0.0)
            elif key == "q":
                node.publish(0.0, 0.0)
                break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
