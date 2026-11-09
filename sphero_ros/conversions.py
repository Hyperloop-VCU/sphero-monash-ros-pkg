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


def ros_velocity_to_env_action(
    linear_x: float, linear_y: float
) -> tuple[float, float]:
    """Convert ROS body velocity to Sphero speed and heading.

    ROS uses +x forward and +y left. Sphero uses 0 degrees forward and
    positive headings toward the right, matching the env heading convention.
    """
    speed = math.hypot(linear_x, linear_y)
    if speed == 0.0:
        return 0.0, 0.0
    heading_env = math.atan2(-linear_y, linear_x)
    return speed, wrap_angle(heading_env)


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quaternion_from_euler(
    roll: float, pitch: float, yaw: float
) -> tuple[float, float, float, float]:
    cr = math.cos(0.5 * roll)
    sr = math.sin(0.5 * roll)
    cp = math.cos(0.5 * pitch)
    sp = math.sin(0.5 * pitch)
    cy = math.cos(0.5 * yaw)
    sy = math.sin(0.5 * yaw)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )
