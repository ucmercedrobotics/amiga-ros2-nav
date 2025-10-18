#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "amiga_navigation/lidar_object_navigator.hpp"

using namespace std::placeholders;

namespace amiga_navigation
{

LidarObjectNavigator::LidarObjectNavigator(const rclcpp::NodeOptions & options)
: Node("lidar_object_navigator", options),
  closest_object_distance_(std::numeric_limits<float>::max()),
  closest_object_angle_(0.0f)
{
  this->declare_parameter<std::string>("lidar_topic", "/scan");
  std::string lidar_topic = this->get_parameter("lidar_topic").as_string();

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    lidar_topic, qos,
    std::bind(&LidarObjectNavigator::lidar_callback, this, _1));

  navigate_to_pose_in_frame_client_ = rclcpp_action::create_client<NavigateToPoseInFrameAction>(this, "navigate_to_pose_in_frame");

  action_server_ = rclcpp_action::create_server<NavigateToObject>(
    this,
    "navigate_to_object",
    std::bind(&LidarObjectNavigator::handle_goal, this, _1, _2),
    std::bind(&LidarObjectNavigator::handle_cancel, this, _1),
    std::bind(&LidarObjectNavigator::handle_accepted, this, _1));

  RCLCPP_INFO(this->get_logger(), "LidarObjectNavigator initialized");
}

void LidarObjectNavigator::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  RCLCPP_DEBUG(
    this->get_logger(),
    "Received LaserScan with %zu points", 
    msg->ranges.size());
  
  float min_range = std::numeric_limits<float>::max();
  size_t min_index = 0;
  
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    if (msg->ranges[i] < min_range && 
        msg->ranges[i] >= msg->range_min && 
        msg->ranges[i] <= msg->range_max) {
      min_range = msg->ranges[i];
      min_index = i;
    }
  }
  
  if (min_range < std::numeric_limits<float>::max()) {
    float angle = msg->angle_min + min_index * msg->angle_increment;
    
    closest_object_distance_ = min_range;
    closest_object_angle_ = angle;
    
    RCLCPP_DEBUG(
      this->get_logger(),
      "Closest obstacle at %.2f meters, angle %.2f rad", 
      min_range, angle);
  }
}

rclcpp_action::GoalResponse LidarObjectNavigator::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const NavigateToObject::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(this->get_logger(), "Received goal to navigate to object index %d", goal->object_index);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LidarObjectNavigator::handle_cancel(
  const std::shared_ptr<GoalHandleNavigateToObject> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void LidarObjectNavigator::handle_accepted(
  const std::shared_ptr<GoalHandleNavigateToObject> goal_handle)
{
  std::thread{std::bind(&LidarObjectNavigator::execute, this, _1), goal_handle}.detach();
}

void LidarObjectNavigator::execute(const std::shared_ptr<GoalHandleNavigateToObject> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal: navigating to selected object");
  active_goal_handle_ = goal_handle;

  auto feedback = std::make_shared<NavigateToObject::Feedback>();
  auto result = std::make_shared<NavigateToObject::Result>();

  // TODO: Replace with logic to select object by index
  // For now, use closest object as before
  if (closest_object_distance_ == 0.0f) {
    RCLCPP_WARN(this->get_logger(), "Either at location or no LiDAR data. Cannot navigate to object.");
    result->success = false;
    result->message = "No valid lidar data";
    goal_handle->abort(result);
    return;
  }

    if (!navigate_to_pose_in_frame_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "NavigateToPoseInFrame action server not available!");
    result->success = false;
      result->message = "NavigateToPoseInFrame action server not available";
    goal_handle->abort(result);
    return;
  }

  float safety_distance = SAFETY_DISTANCE;
  float target_distance = std::max(0.0f, closest_object_distance_ - safety_distance);
  float target_x = target_distance * std::cos(closest_object_angle_);
  float target_y = target_distance * std::sin(closest_object_angle_);

  RCLCPP_INFO(this->get_logger(), "Sending relative move goal: (%.2f, %.2f)", target_x, target_y);

  auto goal_msg = NavigateToPoseInFrameAction::Goal();
  goal_msg.x = target_x;
  goal_msg.y = target_y;
  goal_msg.yaw = closest_object_angle_;
  goal_msg.absolute = false;  // orientation is relative to base_link

  auto send_goal_options = rclcpp_action::Client<NavigateToPoseInFrameAction>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&LidarObjectNavigator::goal_response_callback, this, _1);
  send_goal_options.feedback_callback = std::bind(&LidarObjectNavigator::feedback_callback, this, _1, _2);
  send_goal_options.result_callback = std::bind(&LidarObjectNavigator::result_callback, this, _1);

  navigate_to_pose_in_frame_client_->async_send_goal(goal_msg, send_goal_options);
}


void LidarObjectNavigator::goal_response_callback(const GoalHandleNavigateToPoseInFrame::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by NavigateToPoseInFrameAction server");
    if (active_goal_handle_) {
      auto result = std::make_shared<NavigateToObject::Result>();
      result->success = false;
      result->message = "NavigateToPoseInFrameAction goal rejected";
      active_goal_handle_->abort(result);
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by NavigateToPoseInFrameAction server, navigating...");
  }
}

void LidarObjectNavigator::feedback_callback(
  GoalHandleNavigateToPoseInFrame::SharedPtr,
  const std::shared_ptr<const NavigateToPoseInFrameAction::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Distance remaining: %.2f m", feedback->distance_remaining);
  if (active_goal_handle_) {
  auto fb = std::make_shared<NavigateToObject::Feedback>();
  fb->distance_remaining = feedback->distance_remaining;
  active_goal_handle_->publish_feedback(fb);
  }
}

  void LidarObjectNavigator::result_callback(const GoalHandleNavigateToPoseInFrame::WrappedResult & result)
{
  auto action_result = std::make_shared<NavigateToObject::Result>();
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

  auto node = std::make_shared<amiga_navigation::LidarObjectNavigator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
