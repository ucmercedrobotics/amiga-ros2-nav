from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory("amiga_localization"), "launch")


    gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'ublox.launch.py'))
    )
    
    wheel_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheel_odom.launch.py'))
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'ekf.launch.py'))
    )

    return LaunchDescription([gps_launch, wheel_odom_launch, ekf_launch])