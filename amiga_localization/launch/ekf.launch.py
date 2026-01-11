from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    use_vectornav = LaunchConfiguration("use_vectornav")
    use_gps = LaunchConfiguration("use_gps")
    gps_topic = LaunchConfiguration("gps_topic")

    pkg_share = get_package_share_directory("amiga_localization")

    base_ekf_config_path = os.path.join(pkg_share, "config", "base_ekf.yaml")
    vectornav_ekf_config_path = os.path.join(pkg_share, "config", "vectornav_ekf.yaml")

    ekf_config_path = PythonExpression(
        [
            "'",
            vectornav_ekf_config_path,
            "' if '",
            use_vectornav,
            "'.lower() == 'true' else '",
            base_ekf_config_path,
            "'",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_vectornav",
                default_value="false",
                description="Use vectornav_ekf.yaml (true) or base_ekf.yaml (false)",
            ),
            DeclareLaunchArgument(
                "use_gps",
                default_value="true",
                description="Enable GPS/navsat_transform_node (true) or disable (false)",
            ),
            DeclareLaunchArgument(
                "gps_topic",
                default_value="/gps/pvt",
                description="GPS fix topic to remap to /gps/fix",
            ),
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
            # GPS node - only launched if use_gps=true
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform_node",
                output="screen",
                parameters=[ekf_config_path],
                remappings=[
                    # -- Inputs
                    ("odometry/filtered", "odometry/filtered/global"),
                    ("/gps/fix", gps_topic),
                ],
                condition=IfCondition(use_gps),
            ),
        ]
    )
