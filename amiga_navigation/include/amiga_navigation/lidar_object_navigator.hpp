#pragma once

#include "amiga_navigation_interfaces/action/navigate_to_pose_in_frame.hpp"
#include "amiga_navigation_interfaces/action/navigate_via_lidar.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#define AZIMUTH_TOLERANCE 0.5f  // ~28.6 degrees
#define MIN_OBJECT_HEIGHT 1.0f
#define MAX_OBJECT_HEIGHT 1.5f
#define MAX_OBJECT_DISTANCE 7.0f

namespace amiga_navigation {

class LidarObjectNavigator : public rclcpp::Node {
 public:
  using NavigateToPoseInFrameAction =
      amiga_navigation_interfaces::action::NavigateToPoseInFrame;
  using GoalHandleNavigateToPoseInFrame =
      rclcpp_action::ClientGoalHandle<NavigateToPoseInFrameAction>;
  using NavigateViaLidar = amiga_navigation_interfaces::action::NavigateViaLidar;
  using GoalHandleNavigateViaLidar =
      rclcpp_action::ServerGoalHandle<NavigateViaLidar>;

  explicit LidarObjectNavigator(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~LidarObjectNavigator() = default;

 private:
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Action server callbacks
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const NavigateViaLidar::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleNavigateViaLidar> goal_handle);
  void handle_accepted(
      const std::shared_ptr<GoalHandleNavigateViaLidar> goal_handle);
  void execute(const std::shared_ptr<GoalHandleNavigateViaLidar> goal_handle);

  // NavigateToPoseInFrame action client callbacks
  void goal_response_callback(
      const GoalHandleNavigateToPoseInFrame::SharedPtr& goal_handle);
  void feedback_callback(
      GoalHandleNavigateToPoseInFrame::SharedPtr,
      const std::shared_ptr<const NavigateToPoseInFrameAction::Feedback>
          feedback);
  void result_callback(
      const GoalHandleNavigateToPoseInFrame::WrappedResult& result);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::Client<NavigateToPoseInFrameAction>::SharedPtr
      navigate_to_pose_in_frame_client_;
  rclcpp_action::Server<NavigateViaLidar>::SharedPtr action_server_;

    sensor_msgs::msg::PointCloud2::SharedPtr latest_scan_;
    std::string base_frame_;
    float safety_distance_;
    float lidar_offset_distance_;
    float current_yaw_ = 0.0f;

  std::shared_ptr<GoalHandleNavigateViaLidar> active_goal_handle_;
};

}  // namespace amiga_navigation
