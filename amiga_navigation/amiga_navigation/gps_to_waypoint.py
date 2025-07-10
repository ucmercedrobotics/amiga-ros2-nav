import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from tf2_ros import Buffer, TransformListener, LookupException, TransformException
from tf2_geometry_msgs import do_transform_pose
import math

from amiga_interfaces.srv import GpsToWaypoint
from amiga_navigation.utils.gps_utils import to_utm, quaternion_from_euler

class GpsToWaypointService(Node):
    def __init__(self):
        super().__init__('gps_to_waypoint_service')

        # Waypoint TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.transform = None

        self.waypoint_tf_srv = self.create_service(GpsToWaypoint, 'gps_to_waypoint', self.handle_waypoint_tf)
        self.get_logger().info("Service 'gps_to_waypoint' ready.")

    def handle_waypoint_tf(self, request, response):
        lat = request.latitude
        lon = request.longitude
        yaw = request.yaw

        # -- Convert to UTM
        utm_x, utm_y = to_utm(lat, lon)
        
        # -- Create map-frame PoseStamped from UTM
        self.update_gps_waypoint_tf()
        self.get_logger().info(f"{utm_x}, {utm_y}")
        self.get_logger().info(f"{self.transform}")
        response.waypoint = self.create_waypoint(utm_x, utm_y, yaw)

        return response

    def update_gps_waypoint_tf(self):
        """Get the yaw offset based om the utm->map transform"""
        from_frame = 'utm'
        to_frame = 'map'
        when = rclpy.time.Time()
        while rclpy.ok():
            try:
                self.transform = self.tf_buffer.lookup_transform(
                    to_frame, from_frame, when
                )
                self.get_logger().info("Transform acquired.")
                break
            except LookupException as e:
                self.get_logger().warn(f"Transform not yet available: {e}")
                time.sleep(0.5)  # Sleep and retry
            

    def create_waypoint(self, utm_x: float, utm_y: float, yaw: float) -> PoseStamped:
        """Create a PoseStamped in the 'map' frame given GPS coordiantes."""
        # -- 1. Create Pose from UTM
        pose = Pose()
        pose.position.x = utm_x
        pose.position.y = utm_y
        pose.position.z = 0.0

        quat = quaternion_from_euler(0.0, 0.0, yaw)
        pose.orientation = quat

        # -- 2. Apply transform
        if self.transform is None:
            self.get_logger().warn("No transform available — cannot project to map frame.")
            return PoseStamped()

        try:
            pose = do_transform_pose(pose, self.transform)
        except Exception as e:
            self.get_logger().error(f"Failed to apply pose transform: {e}")
            return PoseStamped()

        pose_stamp = PoseStamped()
        pose_stamp.header.frame_id = "map"
        pose_stamp.header.stamp = self.get_clock().now().to_msg()
        pose_stamp.pose = pose
        
        return pose_stamp




def main():
    rclpy.init()
    node = GpsToWaypointService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()