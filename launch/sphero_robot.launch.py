from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    rviz_config = LaunchConfiguration("rviz_config")

    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    default_rviz = os.path.join(repo_root, "rviz", "sphero_pose.rviz")
    urdf_path = os.path.join(repo_root, "urdf", "sphero_bolt_plus.urdf")
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            ExecuteProcess(
                cmd=["ros2", "run", "robot_state_publisher", "robot_state_publisher", urdf_path],
                output="screen",
            ),
            ExecuteProcess(cmd=["rviz2", "-d", rviz_config], output="screen"),
        ]
    )
