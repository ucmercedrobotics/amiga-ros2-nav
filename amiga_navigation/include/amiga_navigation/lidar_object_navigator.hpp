#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace amiga_navigation
{

class LidarObjectNavigator : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  explicit LidarObjectNavigator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LidarObjectNavigator() = default;

private:
  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  
  // Action server callbacks
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);

  void execute(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);
  
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
  
  float closest_object_distance_;
  float closest_object_angle_;
};

}  // namespace amiga_navigation
