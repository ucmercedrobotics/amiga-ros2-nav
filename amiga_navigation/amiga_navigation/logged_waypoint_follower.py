# From: https://github.com/ros-navigation/navigation2_tutorials/blob/rolling/nav2_gps_waypoint_follower_demo/nav2_gps_waypoint_follower_demo/logged_waypoint_follower.py
from typing import Optional, List
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import sys
import time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import Buffer, TransformListener, LookupException, TransformException
from tf2_geometry_msgs import do_transform_pose
import math

from amiga_interfaces.srv import GpsToWaypoint
from amiga_navigation.utils.gps_utils import to_utm, quaternion_from_euler

def parse_wps(wps_path: str) -> list[dict]:
    """Parse a set of gps waypoints from a yaml file."""
    with open(wps_path, 'r') as wps_file:
        wps_dict = yaml.safe_load(wps_file)
    wps = [wp for wp in wps_dict["waypoints"]]
    return wps

class GpsWpClient(Node):
    def __init__(self):
        super().__init__('gps_waypoint_client')
        self.cli = self.create_client(GpsToWaypoint, 'gps_to_waypoint')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for gps_to_waypoint service...')

    def convert_waypoints(self, gps_wps: list[dict]) -> List[PoseStamped]:
        pose_list = []
        for wp in gps_wps:
            req = GpsToWaypoint.Request()
            req.latitude = wp['latitude']
            req.longitude = wp['longitude']
            req.yaw = wp['yaw']

            future = self.cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None:
                pose_list.append(future.result().waypoint)
            else:
                self.get_logger().warn('Failed to convert waypoint')
        return pose_list
    
class GpsWpFollower:
    """
    Class to use nav2 gps waypoint follower to follow a set of waypoints logged in a yaml file
    """

    def __init__(self):
        self.navigator = BasicNavigator("basic_navigator")

    def start_wpf(self, waypoints: List[PoseStamped]):
        """
        Function to start the waypoint following.
        """
        self.navigator.waitUntilNav2Active(localizer='bt_navigator')
        self.navigator.followWaypoints(waypoints)
        while (not self.navigator.isTaskComplete()):
            time.sleep(0.1)
        print("wps completed successfully")


def main():
    rclpy.init()

    # allow to pass the waypoints file as an argument
    default_yaml_file_path = os.path.join(get_package_share_directory(
        "amiga_navigation"), "config", "waypoints.yaml")
    gps_wps = parse_wps(default_yaml_file_path)

    # -- Spin up GpsWpClient, get the waypoints, close node
    wp_client = GpsWpClient()
    waypoints = wp_client.convert_waypoints(gps_wps)
    wp_client.destroy_node()
    
    # -- Run the Follower
    gps_wpf = GpsWpFollower()
    gps_wpf.start_wpf(waypoints)
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()