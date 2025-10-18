#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "amiga_interfaces/action/navigate_to_object.hpp"
#include "amiga_interfaces/action/navigate_to_pose_in_frame.hpp"

#define SAFETY_DISTANCE 0.75f

namespace amiga_navigation
{

class LidarObjectNavigator : public rclcpp::Node
{
public:
  using NavigateToPoseInFrameAction = amiga_interfaces::action::NavigateToPoseInFrame;
  using GoalHandleNavigateToPoseInFrame = rclcpp_action::ClientGoalHandle<NavigateToPoseInFrameAction>;
  using NavigateToObject = amiga_interfaces::action::NavigateToObject;
  using GoalHandleNavigateToObject = rclcpp_action::ServerGoalHandle<NavigateToObject>;

  explicit LidarObjectNavigator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LidarObjectNavigator() = default;

private:
  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  // Action server callbacks
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const NavigateToObject::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNavigateToObject> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleNavigateToObject> goal_handle);
  void execute(const std::shared_ptr<GoalHandleNavigateToObject> goal_handle);

    // NavigateToPoseInFrame action client callbacks
    void goal_response_callback(const GoalHandleNavigateToPoseInFrame::SharedPtr & goal_handle);
    void feedback_callback(GoalHandleNavigateToPoseInFrame::SharedPtr, const std::shared_ptr<const NavigateToPoseInFrameAction::Feedback> feedback);
    void result_callback(const GoalHandleNavigateToPoseInFrame::WrappedResult & result);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp_action::Client<NavigateToPoseInFrameAction>::SharedPtr navigate_to_pose_in_frame_client_;
  rclcpp_action::Server<NavigateToObject>::SharedPtr action_server_;

  float closest_object_distance_ = 0.0f;
  float closest_object_angle_ = 0.0f;

  std::shared_ptr<GoalHandleNavigateToObject> active_goal_handle_;
};

}  // namespace amiga_navigation
