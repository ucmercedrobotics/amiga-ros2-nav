#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "amiga_navigation/navigate_to_pose_in_frame.hpp"

using namespace std::placeholders;

namespace amiga_navigation
{

NavigateToPoseInFrame::NavigateToPoseInFrame(const rclcpp::NodeOptions & options)
: Node("navigate_to_pose_in_frame", options)
{
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  action_server_ = rclcpp_action::create_server<NavigateToPoseInFrameAction>(
    this,
    "navigate_to_pose_in_frame",
    std::bind(&NavigateToPoseInFrame::handle_goal, this, _1, _2),
    std::bind(&NavigateToPoseInFrame::handle_cancel, this, _1),
    std::bind(&NavigateToPoseInFrame::handle_accepted, this, _1));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(this->get_logger(), "NavigateToPoseInFrame initialized");
}

rclcpp_action::GoalResponse NavigateToPoseInFrame::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const NavigateToPoseInFrameAction::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(
    this->get_logger(),
    "Received goal to move relative: x=%.2f, y=%.2f, yaw=%.2f rad",
    goal->x, goal->y, goal->yaw);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavigateToPoseInFrame::handle_cancel(
  const std::shared_ptr<GoalHandleNavigateToPoseInFrame> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void NavigateToPoseInFrame::handle_accepted(
  const std::shared_ptr<GoalHandleNavigateToPoseInFrame> goal_handle)
{
  std::thread{std::bind(&NavigateToPoseInFrame::execute, this, _1), goal_handle}.detach();
}

void NavigateToPoseInFrame::execute(const std::shared_ptr<GoalHandleNavigateToPoseInFrame> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing relative move goal");
  active_goal_handle_ = goal_handle;

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<NavigateToPoseInFrameAction::Result>();

  // Check if Nav2 action server is available
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available!");
    result->success = false;
    result->message = "Nav2 action server not available";
    goal_handle->abort(result);
    return;
  }

  geometry_msgs::msg::PoseStamped pose_in_base_link;
  pose_in_base_link.header.stamp = this->now();
  pose_in_base_link.header.frame_id = "base_link";
  pose_in_base_link.pose.position.x = goal->x;
  pose_in_base_link.pose.position.y = goal->y;
  pose_in_base_link.pose.position.z = 0.0;
  // If absolute orientation requested, keep relative yaw 0 here; we'll override later
  double relative_yaw = goal->absolute ? 0.0 : static_cast<double>(goal->yaw);
  pose_in_base_link.pose.orientation.x = 0.0;
  pose_in_base_link.pose.orientation.y = 0.0;
  pose_in_base_link.pose.orientation.z = std::sin(relative_yaw / 2.0);
  pose_in_base_link.pose.orientation.w = std::cos(relative_yaw / 2.0);

  // Transform to map so Nav2 gets a consistent frame
  geometry_msgs::msg::PoseStamped pose_in_map;
  const int max_retries = 10;
  const auto retry_delay = std::chrono::milliseconds(100);

  bool transformed = false;
  for (int attempt = 0; attempt < max_retries; ++attempt) {
    try {
      pose_in_map = tf_buffer_->transform(
        pose_in_base_link, "map", tf2::durationFromSec(0.5));
      transformed = true;
      RCLCPP_INFO(
        this->get_logger(),
        "Transformed pose from base_link (%.2f, %.2f) to map (%.2f, %.2f)",
        goal->x, goal->y,
        pose_in_map.pose.position.x, pose_in_map.pose.position.y);
      break;
    } catch (const tf2::TransformException & ex) {
      if (attempt < max_retries - 1) {
        RCLCPP_WARN(
          this->get_logger(),
          "Transform attempt %d/%d failed: %s. Retrying...",
          attempt + 1, max_retries, ex.what());
        std::this_thread::sleep_for(retry_delay);
      } else {
        RCLCPP_ERROR(
          this->get_logger(),
          "Could not transform pose after %d attempts: %s",
          max_retries, ex.what());
      }
    }
  }

  if (!transformed) {
    result->success = false;
    result->message = "TF transform to map failed";
    goal_handle->abort(result);
    active_goal_handle_.reset();
    return;
  }

  auto nav_goal = NavigateToPose::Goal();
  nav_goal.pose = pose_in_map;

  RCLCPP_INFO(
    this->get_logger(),
    "Sending goal to Nav2: frame=%s, x=%.2f, y=%.2f, yaw=%.2f rad (absolute=%s)",
    nav_goal.pose.header.frame_id.c_str(),
    nav_goal.pose.pose.position.x,
    nav_goal.pose.pose.position.y,
    goal->yaw,
    goal->absolute ? "true" : "false");

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback = 
    std::bind(&NavigateToPoseInFrame::goal_response_callback, this, _1);
  send_goal_options.feedback_callback = 
    std::bind(&NavigateToPoseInFrame::feedback_callback, this, _1, _2);
  send_goal_options.result_callback = 
    std::bind(&NavigateToPoseInFrame::result_callback, this, _1);

  nav_client_->async_send_goal(nav_goal, send_goal_options);
}

void NavigateToPoseInFrame::goal_response_callback(
  const GoalHandleNavigateToPose::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by Nav2");
    if (active_goal_handle_) {
      auto result = std::make_shared<NavigateToPoseInFrameAction::Result>();
      result->success = false;
      result->message = "Nav2 goal rejected";
      active_goal_handle_->abort(result);
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by Nav2, navigating...");
  }
}

void NavigateToPoseInFrame::feedback_callback(
  GoalHandleNavigateToPose::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Distance remaining: %.2f m", feedback->distance_remaining);
  
  if (active_goal_handle_) {
  auto fb = std::make_shared<NavigateToPoseInFrameAction::Feedback>();
  fb->distance_remaining = feedback->distance_remaining;
  active_goal_handle_->publish_feedback(fb);
  }
}

void NavigateToPoseInFrame::result_callback(
  const GoalHandleNavigateToPose::WrappedResult & result)
{
  auto action_result = std::make_shared<NavigateToPoseInFrameAction::Result>();
  
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(this->get_logger(), "Navigation succeeded!");
    action_result->success = true;
    action_result->message = "Navigation succeeded";
    active_goal_handle_->succeed(action_result);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Navigation failed");
    action_result->success = false;
    action_result->message = "Navigation failed";
    active_goal_handle_->abort(action_result);
  }
  
  active_goal_handle_.reset();
}

}  // namespace amiga_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amiga_navigation::NavigateToPoseInFrame>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
