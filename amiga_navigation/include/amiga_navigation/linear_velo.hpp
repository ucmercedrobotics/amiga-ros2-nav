#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "amiga_navigation_interfaces/action/move_in_frame.hpp"
#include "amiga_navigation_interfaces/action/rotate_in_frame.hpp"

static constexpr double MIN_LINEAR_VELOCITY = 0.1f;
static constexpr double MAX_LINEAR_VELOCITY = 0.5f;
static constexpr double MAX_ANGULAR_VELOCITY = 0.4f;
static constexpr double HEADING_TOL = 0.15f;
static constexpr double YAW_TOL = 0.15f;
static constexpr double YAW_SLOWDOWN = 0.5f;

namespace amiga_navigation {

class LinearVelo : public rclcpp::Node {
 public:
  using MoveInFrameAction = amiga_navigation_interfaces::action::MoveInFrame;
  using RotateInFrameAction = amiga_navigation_interfaces::action::RotateInFrame;
  using GoalHandleMoveInFrame =
      rclcpp_action::ServerGoalHandle<MoveInFrameAction>;
  using GoalHandleRotateInFrame =
      rclcpp_action::ServerGoalHandle<RotateInFrameAction>;

  explicit LinearVelo(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~LinearVelo() = default;

 private:
  rclcpp_action::GoalResponse handle_move_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const MoveInFrameAction::Goal> goal);
  rclcpp_action::CancelResponse handle_move_cancel(
      const std::shared_ptr<GoalHandleMoveInFrame> goal_handle);
  void handle_move_accepted(
      const std::shared_ptr<GoalHandleMoveInFrame> goal_handle);
  void execute_move(
      const std::shared_ptr<GoalHandleMoveInFrame> goal_handle);

  rclcpp_action::GoalResponse handle_rotate_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const RotateInFrameAction::Goal> goal);
  rclcpp_action::CancelResponse handle_rotate_cancel(
      const std::shared_ptr<GoalHandleRotateInFrame> goal_handle);
  void handle_rotate_accepted(
      const std::shared_ptr<GoalHandleRotateInFrame> goal_handle);
  void execute_rotate(
      const std::shared_ptr<GoalHandleRotateInFrame> goal_handle);

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void stop_robot();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::Server<MoveInFrameAction>::SharedPtr move_action_server_;
  rclcpp_action::Server<RotateInFrameAction>::SharedPtr rotate_action_server_;

  double current_x_ = 0.0, current_y_ = 0.0, current_yaw_ = 0.0;
  double current_linear_speed_ = 0.0;
  double current_angular_speed_ = 0.0;
  double forward_speed_cmd_ = MAX_LINEAR_VELOCITY;
  double angular_speed_cmd_ = MAX_ANGULAR_VELOCITY;
};

}  // namespace amiga_navigation
