#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "amiga_interfaces/action/navigate_to_pose_in_frame.hpp"

static constexpr double MIN_LINEAR_VELOCITY = 0.2f;
static constexpr double MAX_LINEAR_VELOCITY = 1.0f;
static constexpr double MAX_ANGULAR_VELOCITY = 0.5f;

namespace amiga_navigation {

class LinearVelo : public rclcpp::Node {
 public:
  using NavigateToPoseInFrameAction =
      amiga_interfaces::action::NavigateToPoseInFrame;
  using GoalHandleNavigateToPoseInFrame =
      rclcpp_action::ServerGoalHandle<NavigateToPoseInFrameAction>;

  explicit LinearVelo(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~LinearVelo() = default;

 private:
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const NavigateToPoseInFrameAction::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleNavigateToPoseInFrame> goal_handle);
  void handle_accepted(
      const std::shared_ptr<GoalHandleNavigateToPoseInFrame> goal_handle);
  void execute(
      const std::shared_ptr<GoalHandleNavigateToPoseInFrame> goal_handle);

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void stop_robot();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::Server<NavigateToPoseInFrameAction>::SharedPtr action_server_;

  double current_x_ = 0.0, current_y_ = 0.0, current_yaw_ = 0.0;
  double current_linear_speed_ = 0.0;
  double current_angular_speed_ = 0.0;
  double forward_speed_cmd_ = MAX_LINEAR_VELOCITY;
  double angular_speed_cmd_ = MAX_ANGULAR_VELOCITY;
};

}  // namespace amiga_navigation
