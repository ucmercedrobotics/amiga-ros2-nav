from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='amiga_localization',
            executable='wheel_odometry_node',
            name='wheel_odometry_node',
            parameters=[os.path.join(get_package_share_directory("amiga_localization"), "config", "wheel_odom.yaml")],
        ),
    ])