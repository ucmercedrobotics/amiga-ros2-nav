import time
import math
import utm

import numpy as np

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix

from amiga_interfaces.action import Wpfollow # type: ignore


class WaypointFollowerActionServer(Node):
    
    def __init__(self):
        """
        After the inizialization of some usefull variables the node is created, 
        the navigator is istanciated and the action server is defined, as well 
        as the subscription to the /ublox_gps_node/fix topic (providing the 
        robot gps position)

        """
        super().__init__('waypoint_follower') # type: ignore

        self.gps_position = []
        self.utm_position= []

        # Nav2 default navigator
        self.navigator = BasicNavigator()

        # Wait until the behavior tree is active, because we don't use a
        # proper localizer such as AMCL
        self.navigator.waitUntilNav2Active(localizer="bt_navigator")   

        # create a subscription for current gnss location
        # NOTE: this is needed only for the heading of the robot, 
        #       for that reason we are using the gps location alone
        #        and not the output of the EKF
        self._gps_sub = self.create_subscription(msg_type=NavSatFix,
            topic='/ublox_gps_node/fix', 
            callback=self.pose_cb, 
            qos_profile=10)
        
        # create the action server to go to the desired position
        self._action_server_waypoint_follow = ActionServer(
            node=self,
            action_type=Wpfollow,
            action_name='follow_gps_waypoints',
            execute_callback=self.goto_callback)

    def pose_cb(self, msg): # save current location
        """
        Method called each time a new robot gps position is received, it replace
        the old gps robot position (self.gps_position) with the update one.

        Args:
        msg (NavSatFix): message received under the /ublox_gps_node/fix topic
        (provided by the sender)
        """
        self.gps_position = (msg.latitude, msg.longitude)
        self.utm_position = utm.from_latlon(self.gps_position[0], self.gps_position[1])
        
    def goto_callback(self, goal_handle):
        """
        Method used when the action is called by another node, after initializing
        the feedbak message, the required new waypoint is converted in utm coordinates.
        the goal message to send to Nav2 is created setting the goal heading equal
        to the direction between the current robot position and the goal waypoint. 
        finally the method send to Nav2 the new waypoint and keep listen to it in
        orther to give feedback on the action, until the task is completed and its
        result is sent to the action client.

        Args:
        goal_handle (Wpfollow): new waypoint to reach expressed in gps coordinates
        (provided by the action client)

        Returns: 
        result (Wpfollow.result): current gps position of the robot at goal reached
        """
        feedback_msg = Wpfollow.Feedback()

        self.get_logger().info('going to: %f, %f' % (goal_handle.request.lat, goal_handle.request.lon)) # log the waypoint where we are going
        utm_coord = utm.from_latlon(goal_handle.request.lat, goal_handle.request.lon) # convert the waypoint in utm (note they are 4 numbers)
        
        # compose the ROS message
        # compute goal position:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'utm'   # Publishing the utm -> map transform, we can specify the goal directly in utm coordinates
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = utm_coord[0]
        goal_pose.pose.position.y = utm_coord[1]

        # compute goal heading:
        while self.utm_position == []: # wait for the current gps location
            self.get_logger().info('waiting for current location...')
            rclpy.spin_once(self)

        yaw = np.arctan2(utm_coord[1]-self.utm_position[1], utm_coord[0]-self.utm_position[0]) # compute heading among the next waypoint and the current position
        goal_pose.pose.orientation.w = np.cos(yaw/2)
        goal_pose.pose.orientation.z = np.sin(yaw/2)

        self.navigator.followWaypoints([goal_pose]) # send the goal to the navigator

        # while reaching the goal
        while not self.navigator.isTaskComplete():
            # fill the feedback message 
            feedback_msg.dist = math.dist([self.utm_position[0], self.utm_position[1]], [goal_pose.pose.position.x, goal_pose.pose.position.y])
            self.get_logger().info('distance to goal %f' % feedback_msg.dist)
            goal_handle.publish_feedback(feedback_msg) # send the feedback message
            time.sleep(0.5)
        
        # at goal reached
        goal_handle.succeed()
        self.get_logger().info(f"Waypoint finished with: {self.navigator.getResult()}")
        
        # send result:
        result = Wpfollow.Result()
        result.lat = self.gps_position[0] 
        result.lon = self.gps_position[1]
        return result


def main(args=None):
    rclpy.init(args=args)

    wp_folower_server = WaypointFollowerActionServer()

    rclpy.spin(wp_folower_server)


if __name__ == '__main__':
    main()