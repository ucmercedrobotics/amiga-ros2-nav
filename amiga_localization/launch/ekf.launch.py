from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ekf_config_path = os.path.join(
        get_package_share_directory("amiga_localization"), "config", "amiga_ekf.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("output_final_position", default_value="false"),
            DeclareLaunchArgument(
                "output_location", default_value="~/dual_ekf_navsat_example_debug.txt"
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_local_filter_node",
                output="screen",
                parameters=[ekf_config_path],
                remappings=[("odometry/filtered", "odometry/filtered/local")],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_global_filter_node",
                output="screen",
                parameters=[ekf_config_path],
                remappings=[("odometry/filtered", "odometry/filtered/global")],
            ),
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform_node",
                output="screen",
                parameters=[ekf_config_path],
                remappings=[
                    # -- Inputs
                    ("imu/data", "oak0/imu"),
                    ("gps/fix", "ublox_gps_node/fix"),
                    ("odometry/filtered", "odometry/filtered/global"),
                    # -- Outputs
                    ("gps/filtered", "gps/filtered"),
                    ("odometry/gps", "odometry/gps"),
                ],
            ),
        ]
    )
