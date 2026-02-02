#include "amiga_navigation/linear_velo.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>  // for std::clamp
#include <cmath>
#include <thread>

#include "amiga_navigation_interfaces/action/navigate_to_pose_in_frame.hpp"

namespace amiga_navigation {

LinearVelo::LinearVelo(const rclcpp::NodeOptions &options)
    : Node("navigate_to_pose_in_frame", options) {
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_raw", 10);
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered/local", 1,
    std::bind(&LinearVelo::odom_callback, this, std::placeholders::_1));

  // Parameters
  this->declare_parameter<double>("forward_speed", MAX_LINEAR_VELOCITY);
  this->declare_parameter<double>("angular_speed", MAX_ANGULAR_VELOCITY);
  this->get_parameter("forward_speed", forward_speed_cmd_);
  this->get_parameter("angular_speed", angular_speed_cmd_);

  action_server_ = rclcpp_action::create_server<NavigateToPoseInFrameAction>(
      this, "navigate_to_pose_in_frame",
      std::bind(&LinearVelo::handle_goal, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&LinearVelo::handle_cancel, this, std::placeholders::_1),
      std::bind(&LinearVelo::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Linear Velocity Control Action Server ready.");
}

void LinearVelo::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;

  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  current_yaw_ = yaw;
}

rclcpp_action::GoalResponse LinearVelo::handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const NavigateToPoseInFrameAction::Goal> goal) {
  RCLCPP_INFO(get_logger(),
              "Received goal: x=%.2f, y=%.2f, yaw=%.2f, absolute=%d", goal->x,
              goal->y, goal->yaw, goal->absolute);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LinearVelo::handle_cancel(
    const std::shared_ptr<GoalHandleNavigateToPoseInFrame> goal_handle) {
  RCLCPP_WARN(get_logger(), "Goal canceled.");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void LinearVelo::handle_accepted(
    const std::shared_ptr<GoalHandleNavigateToPoseInFrame> goal_handle) {
  std::thread{std::bind(&LinearVelo::execute, this, goal_handle)}.detach();
}

void LinearVelo::execute(
    const std::shared_ptr<GoalHandleNavigateToPoseInFrame> goal_handle) {
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<NavigateToPoseInFrameAction::Feedback>();
  auto result = std::make_shared<NavigateToPoseInFrameAction::Result>();

  rclcpp::Rate rate(2);
  RCLCPP_INFO(get_logger(),
              "Executing linear velocity goal: x=%.2f, y=%.2f, yaw=%.2f",
              goal->x, goal->y, goal->yaw);

  // Capture start pose
  const double start_x = current_x_;
  const double start_y = current_y_;
  const double start_yaw = current_yaw_;

  double target_yaw = goal->yaw;
  if (!goal->absolute) target_yaw = start_yaw + goal->yaw;

  bool has_yaw_goal = std::fabs(goal->yaw) > 1e-3;
  bool position_done = false;
  bool yaw_done = false;
  bool heading_done = false;
  double target_x = goal->x;
  double target_y = goal->y;

  if (!goal->absolute) {
    const double c = std::cos(start_yaw);
    const double s = std::sin(start_yaw);
    const double dx_w = c * goal->x - s * goal->y;
    const double dy_w = s * goal->x + c * goal->y;
    target_x = dx_w;
    target_y = dy_w;
  }
  const double goal_vec_x = target_x;
  const double goal_vec_y = target_y;
  const double target_distance =
      std::sqrt(goal_vec_x * goal_vec_x + goal_vec_y * goal_vec_y);
  const double angle_to_goal = std::atan2(goal_vec_y, goal_vec_x);

  double traveled_distance = 0.0;
  const double forward_speed_cmd = forward_speed_cmd_;

  auto normalize_angle = [](double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  };

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      stop_robot();
      result->success = false;
      result->message = "Goal canceled.";
      goal_handle->canceled(result);
      return;
    }

    geometry_msgs::msg::Twist cmd;

    const double dx = current_x_ - start_x;
    const double dy = current_y_ - start_y;
    traveled_distance = std::sqrt(dx * dx + dy * dy);

    double distance_remaining =
        std::max(0.0, target_distance - traveled_distance);
    feedback->distance_remaining = distance_remaining;
    const double yaw_error =
      has_yaw_goal ? normalize_angle(target_yaw - current_yaw_) : 0.0;
    feedback->yaw_remaining = has_yaw_goal ? yaw_error : 0.0f;
    goal_handle->publish_feedback(feedback);

    // --- Rotate-in-place to face goal ---
    if (!position_done && !heading_done) {
      const double heading_error = atan2(sin(angle_to_goal - current_yaw_),
                                         cos(angle_to_goal - current_yaw_));
      if (std::fabs(heading_error) > HEADING_TOL) {
        const double scaled = std::clamp(heading_error / YAW_SLOWDOWN, -1.0, 1.0);
        cmd.angular.z = angular_speed_cmd_ * scaled;
      } else {
        heading_done = true;
      }
    }
    // --- Translation mode ---
    else if (!position_done) {
      if (traveled_distance < target_distance) {
        // Scale velocity based on distance remaining - slow down as we approach
        const double decel_distance = 2.0;  // Distance at which to start slowing down (meters)
        double speed_scale;

        if (distance_remaining > decel_distance) {
          speed_scale = 1.0;  // Full speed while far away
        } else {
          // Linearly interpolate from MIN to MAX velocity as we approach
          speed_scale = std::clamp(distance_remaining / decel_distance,
                                   MIN_LINEAR_VELOCITY / MAX_LINEAR_VELOCITY,
                                   1.0);
        }

        cmd.linear.x = forward_speed_cmd * speed_scale;
      } else {
        position_done = true;
        cmd.linear.x = 0.0;
      }
    }
    // --- Final yaw alignment ---
    else if (has_yaw_goal) {
      if (std::fabs(yaw_error) > YAW_TOL) {
        const double scaled = std::clamp(yaw_error / YAW_SLOWDOWN, -1.0, 1.0);
        cmd.angular.z = angular_speed_cmd_ * scaled;
      } else {
        yaw_done = true;
      }
    }

    cmd.linear.x =
        std::clamp(cmd.linear.x, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    cmd.angular.z =
        std::clamp(cmd.angular.z, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

    cmd_pub_->publish(cmd);

    if ((position_done || (!goal->x && !goal->y)) &&
      (!has_yaw_goal || yaw_done)) {
      stop_robot();
      result->success = true;
      result->message = "Goal reached successfully.";
      goal_handle->succeed(result);
      return;
    }

    rate.sleep();
  }
}

void LinearVelo::stop_robot() {
  geometry_msgs::msg::Twist stop;
  cmd_pub_->publish(stop);
}

}  // namespace amiga_navigation

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amiga_navigation::LinearVelo>());
  rclcpp::shutdown();
  return 0;
}
