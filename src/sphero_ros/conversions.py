import math
from dataclasses import dataclass


@dataclass
class TwistCommand:
    linear_x: float
    angular_z: float


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def ros_yaw_to_env_heading(yaw_ros: float) -> float:
    """Convert ROS yaw (0 on +x) to env heading (0 on +y)."""
    return wrap_angle((math.pi / 2.0) - yaw_ros)


def env_heading_to_ros_yaw(heading_env: float) -> float:
    """Convert env heading (0 on +y) to ROS yaw (0 on +x)."""
    return wrap_angle((math.pi / 2.0) - heading_env)


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)
