#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>  // for std::clamp
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "amiga_interfaces/action/navigate_to_pose_in_frame.hpp"

class LinearVelo : public rclcpp::Node {
 public:
  using NavigateToPoseInFrame = amiga_interfaces::action::NavigateToPoseInFrame;
  using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPoseInFrame>;

  LinearVelo() : Node("navigate_to_pose_in_frame_server") {
    cmd_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&LinearVelo::odom_callback, this, std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<NavigateToPoseInFrame>(
        this, "navigate_to_pose_in_frame",
        std::bind(&LinearVelo::handle_goal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&LinearVelo::handle_cancel, this, std::placeholders::_1),
        std::bind(&LinearVelo::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "NavigateToPoseInFrame Action Server ready.");
  }

 private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::Server<NavigateToPoseInFrame>::SharedPtr action_server_;

  double current_x_ = 0.0, current_y_ = 0.0, current_yaw_ = 0.0;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_yaw_ = yaw;
  }

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const NavigateToPoseInFrame::Goal> goal) {
    RCLCPP_INFO(get_logger(),
                "Received goal: x=%.2f, y=%.2f, yaw=%.2f, absolute=%d", goal->x,
                goal->y, goal->yaw, goal->absolute);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandle> goal_handle) {
    RCLCPP_WARN(get_logger(), "Goal canceled.");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    std::thread{std::bind(&LinearVelo::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<NavigateToPoseInFrame::Feedback>();
    auto result = std::make_shared<NavigateToPoseInFrame::Result>();

    rclcpp::Rate rate(10);
    RCLCPP_INFO(get_logger(), "Executing navigation goal...");

    double start_yaw = current_yaw_;
    double target_yaw = goal->yaw;
    if (!goal->absolute)
      target_yaw = start_yaw + goal->yaw;  // relative rotation

    bool has_yaw_goal = std::fabs(goal->yaw) > 1e-3;
    bool position_done = false;
    bool yaw_done = false;

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        stop_robot();
        result->success = false;
        result->message = "Goal canceled.";
        goal_handle->canceled(result);
        return;
      }

      geometry_msgs::msg::Twist cmd;

      // Compute yaw error
      double yaw_error =
          atan2(sin(target_yaw - current_yaw_), cos(target_yaw - current_yaw_));

      // Compute distance error
      double dx = goal->x - current_x_;
      double dy = goal->y - current_y_;
      double distance = std::sqrt(dx * dx + dy * dy);

      feedback->distance_remaining = distance;
      feedback->yaw_remaining = yaw_error;
      goal_handle->publish_feedback(feedback);

      // --- Rotation-only mode ---
      if (has_yaw_goal && (fabs(goal->x) < 1e-3 && fabs(goal->y) < 1e-3)) {
        if (fabs(yaw_error) > 0.25) {
          cmd.angular.z = 0.6 * yaw_error;
        } else {
          yaw_done = true;
        }
      }
      // --- Translation mode ---
      else {
        double angle_to_goal = atan2(dy, dx);
        double heading_error = atan2(sin(angle_to_goal - current_yaw_),
                                     cos(angle_to_goal - current_yaw_));

        if (distance > 0.75) {
          cmd.linear.x = 0.4 * distance;
          cmd.angular.z = 0.6 * heading_error;
        } else {
          position_done = true;
        }
      }

      // Cap speeds
      cmd.linear.x = std::clamp(cmd.linear.x, -1.0, 1.0);
      cmd.angular.z = std::clamp(cmd.angular.z, -1.0, 1.0);

      cmd_pub_->publish(cmd);

      // Completion check
      if ((position_done || (!goal->x && !goal->y)) &&
          (!has_yaw_goal || fabs(yaw_error) < 0.25)) {
        stop_robot();
        result->success = true;
        result->message = "Goal reached successfully.";
        goal_handle->succeed(result);
        return;
      }

      rate.sleep();
    }
  }

  void stop_robot() {
    geometry_msgs::msg::Twist stop;
    cmd_pub_->publish(stop);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LinearVelo>());
  rclcpp::shutdown();
  return 0;
}
