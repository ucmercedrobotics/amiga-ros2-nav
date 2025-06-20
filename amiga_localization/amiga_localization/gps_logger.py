import rclpy
from rclpy.node import Node
from ublox_msgs.msg import NavPVT
import sys
import geopy.distance

class LogGpsWaypoints(Node):
    # ROS 2 node to log gps waypoints in lat long format in a text file.
    # While teleoperating the robot with the GPS active, this node log a waypoint each 5m.

    def __init__(self):
        super().__init__('gps_logger')
        self.get_logger().info("LogGpsWaypoints starting...")
        
        # Initialize and declare the filepath for logging the waypoints
        self.declare_parameter("waypoints_path", "./waypoints.txt")
        waypoints_path = self.get_parameter("waypoints_path").get_parameter_value().string_value
        
        self._navpvt_sub = self.create_subscription(NavPVT, "/ublox_gps_node/navpvt", self.navpvt_callback, 10)
        
        # Initialize variables
        self.old_coord = (0, 0)   # In this way, the first waypoint is the robot's starting position
        self.filehandle = open(waypoints_path,'w')
        self.wp_num = 0           # Number of logged waypoints
        
    def __del__(self):
        self.filehandle.close()

    def navpvt_callback(self, navpvt: NavPVT) -> None:
        new_coord = (navpvt.lat/1e7, navpvt.lon/1e7)   # Extract lat long coordinates from the navpvt message
        distance = geopy.distance.geodesic(new_coord, self.old_coord).meters   # Compute the distance in meters from previous waypoint
        if (distance > 5):
            self.wp_num = self.wp_num + 1
            # Log new coordinates in format "lat long take_picture"; take_picture is defaulted to 0 for this mode
            # waypoints with pictures can be toggled using manual joystick waypoint capture
            self.filehandle.write(str(new_coord[0])+" "+str(new_coord[1])+ " 0" + "\n")
            self.old_coord = new_coord
            self.get_logger().info('New waypoint identified, distance from previous one: %f' % distance)

def main():
    try:
        rclpy.init()
        node = LogGpsWaypoints()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C received - exiting...')
        sys.exit(0)
    finally:
        node.get_logger().info('ROS node LogGpsWaypoints shutdown, waypoints identified: %d' % node.wp_num)
        del node
        rclpy.shutdown()

if __name__ == '__main__':
    main()