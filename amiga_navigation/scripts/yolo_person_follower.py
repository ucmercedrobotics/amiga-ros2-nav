#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import threading
from amiga_navigation_interfaces.action import FollowPerson
import time


LEFT_WRIST_CLASS: int = 9
RIGHT_WRIST_CLASS: int = 10
LEFT_SHOULDER_CLASS: int = 5
RIGHT_SHOULDER_CLASS: int = 6
PERSON_CLASS: int = 0


class YOLOPersonFollower(Node):
    def __init__(self):
        super().__init__("yolo_person_follower")

        self.declare_parameter("yolo_model", "yolo11n-pose.pt")
        self.declare_parameter("camera_topic", "/oak0/rgb/image_raw")
        self.declare_parameter("depth_topic", "/oak0/stereo/image_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("max_linear_velocity", 1.2)
        self.declare_parameter("max_angular_velocity", 1.0)
        self.declare_parameter("follow_distance", 1.5)
        self.declare_parameter("distance_tolerance", 0.5)
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("frame_center_tolerance", 50)
        self.declare_parameter("use_pose_detection", True)
        self.declare_parameter("hand_raise_threshold", 0)

        self.yolo_model = self.get_parameter("yolo_model").value
        self.camera_topic = self.get_parameter("camera_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.max_linear_vel = self.get_parameter("max_linear_velocity").value
        self.max_angular_vel = self.get_parameter("max_angular_velocity").value
        self.follow_distance = self.get_parameter("follow_distance").value
        self.distance_tolerance = self.get_parameter("distance_tolerance").value
        self.confidence_threshold = self.get_parameter("confidence_threshold").value
        self.frame_center_tolerance = self.get_parameter("frame_center_tolerance").value
        self.use_pose_detection = self.get_parameter("use_pose_detection").value
        self.hand_raise_threshold = self.get_parameter("hand_raise_threshold").value

        self.get_logger().info(f"Loading YOLO model: {self.yolo_model}")
        self.model = YOLO(self.yolo_model)
        self.get_logger().info("YOLO model loaded successfully")

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.image_sub = self.create_subscription(
            Image, self.camera_topic, self.image_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, 10
        )

        self.bridge = CvBridge()

        self.person_detected: bool = False
        self.current_depth_image = None
        self.frame_lock = threading.Lock()
        self.frame_width = None
        self.frame_height = None
        self.depth_width = None
        self.depth_height = None
        self.hand_raised = False
        self._active_goal = False
        self._current_goal_handle = None

        self._action_server = ActionServer(
            self,
            FollowPerson,
            "follow_person",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info("YOLO Person Follower node initialized")

    def depth_callback(self, msg: Image):
        try:
            with self.frame_lock:
                # Depth image can be 16UC1 or 32FC1
                if msg.encoding == "16UC1":
                    self.current_depth_image = self.bridge.imgmsg_to_cv2(
                        msg, desired_encoding="16UC1"
                    )
                elif msg.encoding == "32FC1":
                    self.current_depth_image = self.bridge.imgmsg_to_cv2(
                        msg, desired_encoding="32FC1"
                    )
                else:
                    self.current_depth_image = self.bridge.imgmsg_to_cv2(
                        msg, desired_encoding="passthrough"
                    )

                # Store depth image dimensions
                if self.current_depth_image is not None:
                    self.depth_height, self.depth_width = (
                        self.current_depth_image.shape[:2]
                    )
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {str(e)}")

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Extract frame dimensions from image
            if self.frame_width is None or self.frame_height is None:
                self.frame_height, self.frame_width = cv_image.shape[:2]

            results = self.model(cv_image, verbose=False)

            self.process_detections(results)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

    def process_detections(self, results):
        cmd_vel = Twist()
        person_detected = False
        self.hand_raised = False

        for result in results:
            if result.boxes is not None:
                for box in result.boxes:
                    if (
                        int(box.cls) == PERSON_CLASS
                        and box.conf >= self.confidence_threshold
                    ):
                        person_detected = True

                        x1, y1, x2, y2 = map(int, box.xyxy[0])

                        person_center_x = (x1 + x2) / 2

                        if (
                            self.use_pose_detection
                            and hasattr(result, "keypoints")
                            and result.keypoints is not None
                        ):
                            self.hand_raised = self.check_hand_raised(result.keypoints)

                        person_distance = self.get_person_distance(x1, y1, x2, y2)

                        self.person_detected = True

                        if not self.hand_raised:
                            cmd_vel = self.calculate_cmd_vel(
                                person_center_x, person_distance
                            )
                        else:
                            self.get_logger().info(
                                "Hand raised detected - stopping robot"
                            )
                            cmd_vel = Twist()
                        break

                if person_detected:
                    break

        if not person_detected:
            self.person_detected = False
            cmd_vel = Twist()

        if self._active_goal:
            self.cmd_vel_pub.publish(cmd_vel)

    def get_person_distance(self, x1, y1, x2, y2):
        """Calculate the median distance to the person from depth image."""
        with self.frame_lock:
            if self.current_depth_image is None:
                return None

            try:
                # Scale bounding box coordinates if RGB and depth have different resolutions
                if (
                    self.frame_width is not None
                    and self.frame_height is not None
                    and self.depth_width is not None
                    and self.depth_height is not None
                ):

                    # Calculate scale factors
                    scale_x = self.depth_width / self.frame_width
                    scale_y = self.depth_height / self.frame_height

                    # Scale coordinates to depth image resolution
                    x1_depth = int(x1 * scale_x)
                    y1_depth = int(y1 * scale_y)
                    x2_depth = int(x2 * scale_x)
                    y2_depth = int(y2 * scale_y)

                    # Clamp to depth image bounds
                    x1_depth = max(0, min(x1_depth, self.depth_width - 1))
                    x2_depth = max(0, min(x2_depth, self.depth_width - 1))
                    y1_depth = max(0, min(y1_depth, self.depth_height - 1))
                    y2_depth = max(0, min(y2_depth, self.depth_height - 1))
                else:
                    # Fallback: assume same resolution
                    x1_depth, y1_depth, x2_depth, y2_depth = x1, y1, x2, y2

                depth_roi = self.current_depth_image[
                    y1_depth:y2_depth, x1_depth:x2_depth
                ]

                valid_depths = depth_roi[depth_roi > 0]

                if len(valid_depths) == 0:
                    return None

                if self.current_depth_image.dtype == np.uint16:
                    median_distance = np.median(valid_depths) / 1000.0
                else:
                    median_distance = np.median(valid_depths)

                return float(median_distance)
            except Exception as e:
                self.get_logger().debug(f"Error calculating person distance: {str(e)}")
                return None

    def calculate_cmd_vel(self, person_center_x, person_distance):
        cmd_vel = Twist()

        if self.frame_width is None:
            return cmd_vel

        # Angular velocity: center the person in frame
        frame_center_x = self.frame_width / 2
        h_error = person_center_x - frame_center_x

        if abs(h_error) > self.frame_center_tolerance:
            normalized_error = h_error / (self.frame_width / 2)
            cmd_vel.angular.z = -np.clip(
                normalized_error * self.max_angular_vel,
                -self.max_angular_vel,
                self.max_angular_vel,
            )

        if person_distance is not None:
            distance_error = person_distance - self.follow_distance
            if abs(distance_error) > self.distance_tolerance:
                cmd_vel.linear.x = np.clip(
                    0.5 * distance_error,
                    -self.max_linear_vel,
                    self.max_linear_vel,
                )

        return cmd_vel

    def check_hand_raised(self, keypoints):
        try:
            kps = keypoints
            if kps is not None and len(kps) > 0:

                pts = kps.xy[0]
                conf = kps.conf[0]

                # Require wrist confidence
                if (
                    conf[LEFT_WRIST_CLASS] > self.confidence_threshold
                    and conf[RIGHT_WRIST_CLASS] > self.confidence_threshold
                ):

                    _, lsy = pts[LEFT_SHOULDER_CLASS]
                    _, rsy = pts[RIGHT_SHOULDER_CLASS]
                    _, lwy = pts[LEFT_WRIST_CLASS]
                    _, rwy = pts[RIGHT_WRIST_CLASS]

                    if (
                        lwy < lsy - self.hand_raise_threshold
                        or rwy < rsy - self.hand_raise_threshold
                    ):
                        return True
        except Exception as e:
            self.get_logger().debug(f"Error checking hand raised: {str(e)}")
            return False

    def goal_callback(self, goal_request):
        self.get_logger().info("Received follow_person goal request")
        if self._active_goal:
            self.get_logger().info("Rejecting new goal, another goal is active")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request for follow_person")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info("FollowPerson goal started")
        self._active_goal = True
        self._current_goal_handle = goal_handle

        feedback = FollowPerson.Feedback()

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result = FollowPerson.Result()
                    result.success = False
                    result.message = "canceled"
                    self._active_goal = False
                    self._current_goal_handle = None
                    return result

                with self.frame_lock:
                    hand = self.hand_raised

                if hand:
                    feedback.status = "hand_raised"
                    goal_handle.publish_feedback(feedback)
                    result = FollowPerson.Result()
                    result.success = True
                    result.message = "stopped_by_hand"
                    goal_handle.succeed()
                    self._active_goal = False
                    self._current_goal_handle = None
                    return result

                status = "following" if self.person_detected else "no_person"
                feedback.status = status
                goal_handle.publish_feedback(feedback)

                time.sleep(0.1)

        except Exception as e:
            self.get_logger().error(f"Error in action execute: {e}")
            result = FollowPerson.Result()
            result.success = False
            result.message = f"error: {e}"
            self._active_goal = False
            self._current_goal_handle = None
            try:
                goal_handle.abort()
            except Exception:
                pass
            return result


def main(args=None):
    rclpy.init(args=args)
    node = YOLOPersonFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
