#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>  // for std::clamp
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>

#include "amiga_interfaces/action/navigate_to_pose_in_frame.hpp"

class LinearVelo : public rclcpp::Node {
 public:
  using NavigateToPoseInFrame = amiga_interfaces::action::NavigateToPoseInFrame;
  using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPoseInFrame>;

  LinearVelo() : Node("navigate_to_pose_in_frame_server") {
    cmd_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered/local", 10,
        std::bind(&LinearVelo::odom_callback, this, std::placeholders::_1));

    // Parameters
    this->declare_parameter<double>("forward_speed", 1.0);
    this->declare_parameter<double>("angular_speed", 0.6);
    this->get_parameter("forward_speed", forward_speed_cmd_);
    this->get_parameter("angular_speed", angular_speed_cmd_);

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
  double current_linear_speed_ = 0.0;   // m/s from odometry
  double current_angular_speed_ = 0.0;  // rad/s from odometry
  double forward_speed_cmd_ = 1.0;      // m/s, configurable via parameter
  double angular_speed_cmd_ = 0.6;      // rad/s, configurable via parameter

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

    // Estimate planar linear speed from odometry twist
    const double vx = msg->twist.twist.linear.x;
    const double vy = msg->twist.twist.linear.y;
    current_linear_speed_ = std::sqrt(vx * vx + vy * vy);
    current_angular_speed_ = msg->twist.twist.angular.z;
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

    // Capture start pose
    const double start_x = current_x_;
    const double start_y = current_y_;
    const double start_yaw = current_yaw_;

    double target_yaw = goal->yaw;
    if (!goal->absolute)
      target_yaw = start_yaw + goal->yaw;  // relative rotation

    bool has_yaw_goal = std::fabs(goal->yaw) > 1e-3;
    bool position_done = false;
    bool yaw_done = false;

    // Compute translation target as a vector from start to target
    double target_x = goal->x;
    double target_y = goal->y;
    if (!goal->absolute) {
      target_x = start_x + goal->x;
      target_y = start_y + goal->y;
    }
    const double goal_vec_x = target_x - start_x;
    const double goal_vec_y = target_y - start_y;
    const double target_distance =
        std::sqrt(goal_vec_x * goal_vec_x + goal_vec_y * goal_vec_y);
    const double angle_to_goal = std::atan2(goal_vec_y, goal_vec_x);

    // Integrate actual traveled distance using odometry speed
    double traveled_distance = 0.0;
    rclcpp::Time last_time = this->get_clock()->now();
    // Use configured forward speed command (m/s)
    const double forward_speed_cmd = forward_speed_cmd_;

    // Setup rotation tracking based on time/odometry
    auto normalize_angle = [](double a) {
      while (a > M_PI) a -= 2.0 * M_PI;
      while (a < -M_PI) a += 2.0 * M_PI;
      return a;
    };
    double desired_signed_rotation = 0.0;
    if (has_yaw_goal) {
      if (goal->absolute) {
        desired_signed_rotation = normalize_angle(target_yaw - start_yaw);
      } else {
        desired_signed_rotation = goal->yaw;
      }
    }
    const double desired_rotation_mag = std::fabs(desired_signed_rotation);
    const double rotation_dir = (desired_signed_rotation >= 0.0) ? 1.0 : -1.0;
    double rotated_angle = 0.0;  // integrate |omega| over time

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        stop_robot();
        result->success = false;
        result->message = "Goal canceled.";
        goal_handle->canceled(result);
        return;
      }

      geometry_msgs::msg::Twist cmd;

      // Update traveled distance from odometry speed and time delta
      rclcpp::Time now = this->get_clock()->now();
      double dt = (now - last_time).seconds();
      if (dt > 0.0) {
        traveled_distance += current_linear_speed_ * dt;
        rotated_angle += std::fabs(current_angular_speed_) * dt;
        last_time = now;
      }

      // Compute yaw error
      double yaw_error =
          atan2(sin(target_yaw - current_yaw_), cos(target_yaw - current_yaw_));

      // Feedback based on time/velocity integrated distance
      double distance_remaining =
          std::max(0.0, target_distance - traveled_distance);
      feedback->distance_remaining = distance_remaining;
      double yaw_remaining =
          rotation_dir * std::max(0.0, desired_rotation_mag - rotated_angle);
      feedback->yaw_remaining = has_yaw_goal ? yaw_remaining : 0.0f;
      goal_handle->publish_feedback(feedback);

      // --- Rotation-only mode ---
      if (has_yaw_goal && (fabs(goal->x) < 1e-3 && fabs(goal->y) < 1e-3)) {
        if (rotated_angle < desired_rotation_mag) {
          cmd.angular.z = angular_speed_cmd_ * rotation_dir;
        } else {
          yaw_done = true;
        }
      }
      // --- Translation mode ---
      else {
        // Use a fixed target heading based on start->goal vector
        double heading_error = atan2(sin(angle_to_goal - current_yaw_),
                                     cos(angle_to_goal - current_yaw_));

        if (traveled_distance < target_distance) {
          // Forward speed command; reduce if heading error is large
          double speed_scale = std::clamp(
              1.0 - std::min(std::fabs(heading_error), 1.0), 0.2, 1.0);
          cmd.linear.x = forward_speed_cmd * speed_scale;
          cmd.angular.z = 0.6 * heading_error;
        } else {
          // Distance reached; stop translation, optionally finish yaw
          // (time-based)
          position_done = true;
          cmd.linear.x = 0.0;
          if (has_yaw_goal && rotated_angle < desired_rotation_mag) {
            cmd.angular.z = angular_speed_cmd_ * rotation_dir;
          }
        }
      }

      // Cap speeds
      cmd.linear.x = std::clamp(cmd.linear.x, -1.0, 1.0);
      cmd.angular.z = std::clamp(cmd.angular.z, -1.0, 1.0);

      cmd_pub_->publish(cmd);

      // Completion check
      if ((position_done || (!goal->x && !goal->y)) &&
          (!has_yaw_goal || yaw_done ||
           rotated_angle >= desired_rotation_mag)) {
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
