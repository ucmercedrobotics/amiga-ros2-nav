#include "amiga_navigation/lidar_object_navigator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

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

  action_server_ = rclcpp_action::create_server<NavigateToPose>(
    this,
    "navigate_to_pose",
    std::bind(&LidarObjectNavigator::handle_goal, this, _1, _2),
    std::bind(&LidarObjectNavigator::handle_cancel, this, _1),
    std::bind(&LidarObjectNavigator::handle_accepted, this, _1));
  
  RCLCPP_INFO(this->get_logger(), "LidarObjectNavigator initialized");
  RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", lidar_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Action server 'navigate_to_pose' ready");
}

void LidarObjectNavigator::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  RCLCPP_DEBUG(
    this->get_logger(),
    "Received LaserScan with %zu points", 
    msg->ranges.size());
  
  // TODO: Add your lidar processing logic here
  // Example: Find closest obstacle
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
  std::shared_ptr<const NavigateToPose::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(
    this->get_logger(),
    "Received goal request to navigate to position (%.2f, %.2f, %.2f)",
    goal->pose.pose.position.x,
    goal->pose.pose.position.y,
    goal->pose.pose.position.z);
  
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LidarObjectNavigator::handle_cancel(
  const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void LidarObjectNavigator::handle_accepted(
  const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  std::thread{std::bind(&LidarObjectNavigator::execute, this, _1), goal_handle}.detach();
}

void LidarObjectNavigator::execute(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<NavigateToPose::Feedback>();
  auto result = std::make_shared<NavigateToPose::Result>();
  
  rclcpp::Rate loop_rate(10);
  
  for (int i = 0; i < 50 && rclcpp::ok(); ++i) {
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }
    
    feedback->current_pose.pose.position.x = i * 0.1;
    feedback->current_pose.pose.position.y = i * 0.1;
    feedback->distance_remaining = 5.0 - (i * 0.1);
    
    goal_handle->publish_feedback(feedback);
    RCLCPP_DEBUG(this->get_logger(), "Publishing feedback");
    
    loop_rate.sleep();
  }
  
  if (rclcpp::ok()) {
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
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
