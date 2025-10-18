#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "amiga_interfaces/action/navigate_to_pose_in_frame.hpp"
// TF2
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.h"

namespace amiga_navigation
{

class NavigateToPoseInFrame : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using NavigateToPoseInFrameAction = amiga_interfaces::action::NavigateToPoseInFrame;
  using GoalHandleNavigateToPoseInFrame = rclcpp_action::ServerGoalHandle<NavigateToPoseInFrameAction>;

  explicit NavigateToPoseInFrame(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~NavigateToPoseInFrame() = default;

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToPoseInFrameAction::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigateToPoseInFrame> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleNavigateToPoseInFrame> goal_handle);
  void execute(const std::shared_ptr<GoalHandleNavigateToPoseInFrame> goal_handle);

  void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle);
  void feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void result_callback(const GoalHandleNavigateToPose::WrappedResult & result);

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Server<NavigateToPoseInFrameAction>::SharedPtr action_server_;
  
  std::shared_ptr<GoalHandleNavigateToPoseInFrame> active_goal_handle_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace amiga_navigation
