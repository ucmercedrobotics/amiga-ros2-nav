from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_vectornav = LaunchConfiguration("use_vectornav")

    launch_dir = os.path.join(
        get_package_share_directory("amiga_localization"), "launch"
    )

    wheel_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "wheel_odom.launch.py"))
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "ekf.launch.py")),
        launch_arguments={"use_vectornav": use_vectornav}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_vectornav",
                default_value="false",
                description="Use vectornav_ekf.yaml (true) or base_ekf.yaml (false)",
            ),
            wheel_odom_launch,
            ekf_launch,
        ]
    )
