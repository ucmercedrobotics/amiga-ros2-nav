#include "amiga_navigation/lidar_object_navigator.hpp"

#include <cmath>
#include <limits>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using namespace std::placeholders;

namespace amiga_navigation {

LidarObjectNavigator::LidarObjectNavigator(const rclcpp::NodeOptions& options)
    : Node("lidar_object_navigator", options) {
  this->declare_parameter<std::string>("lidar_topic", "/ouster/points");
  std::string lidar_topic = this->get_parameter("lidar_topic").as_string();

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic, qos,
    std::bind(&LidarObjectNavigator::lidar_callback, this, _1));

  navigate_to_pose_in_frame_client_ =
      rclcpp_action::create_client<NavigateToPoseInFrameAction>(
          this, "navigate_to_pose_in_frame");

  action_server_ = rclcpp_action::create_server<NavigateViaLidar>(
      this, "navigate_via_lidar",
      std::bind(&LidarObjectNavigator::handle_goal, this, _1, _2),
      std::bind(&LidarObjectNavigator::handle_cancel, this, _1),
      std::bind(&LidarObjectNavigator::handle_accepted, this, _1));

  RCLCPP_INFO(this->get_logger(), "LidarObjectNavigator initialized");
}

void LidarObjectNavigator::lidar_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  size_t point_count = static_cast<size_t>(msg->width) * msg->height;
  RCLCPP_DEBUG(this->get_logger(), "Received PointCloud2 with %zu points",
               point_count);
  latest_scan_ = msg;
}

rclcpp_action::GoalResponse LidarObjectNavigator::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const NavigateViaLidar::Goal> goal) {
  (void)uuid;
  RCLCPP_INFO(this->get_logger(),
              "Received goal to navigate to object angle %f",
              goal->object_angle);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LidarObjectNavigator::handle_cancel(
    const std::shared_ptr<GoalHandleNavigateViaLidar> goal_handle) {
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void LidarObjectNavigator::handle_accepted(
    const std::shared_ptr<GoalHandleNavigateViaLidar> goal_handle) {
  std::thread{std::bind(&LidarObjectNavigator::execute, this, _1), goal_handle}
      .detach();
}

void LidarObjectNavigator::execute(
    const std::shared_ptr<GoalHandleNavigateViaLidar> goal_handle) {
  if (!latest_scan_) {
    RCLCPP_WARN(this->get_logger(), "No LiDAR data available yet.");
    return;
  }
  active_goal_handle_ = goal_handle;

  auto result = std::make_shared<NavigateViaLidar::Result>();
  double theta_target = goal_handle->get_goal()->object_angle;
  double azimuth_tolerance = AZIMUTH_TOLERANCE; 

  const auto &pc = *latest_scan_;
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pc, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pc, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pc, "z");

  std::vector<Eigen::Vector3f> selected_points;
  float x, y, z;

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    x = *iter_x;
    y = *iter_y;
    z = *iter_z;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
      continue;

    double azimuth = std::atan2(y, x);
    double diff = std::atan2(std::sin(azimuth - theta_target),
                            std::cos(azimuth - theta_target));

    if (std::fabs(diff) < azimuth_tolerance) {
      if (z > MIN_OBJECT_HEIGHT && z < MAX_OBJECT_HEIGHT) {
        selected_points.emplace_back(x, y, z);
      }
    }
  }

  if (selected_points.empty()) {
    RCLCPP_WARN(this->get_logger(), "No points found at orientation %.2f rad", theta_target);
    return;
  }

  RCLCPP_INFO(this->get_logger(),
              "Found %zu points near %.2f rad. Height range: [%.2f, %.2f] m",
              selected_points.size(), theta_target, MIN_OBJECT_HEIGHT, MAX_OBJECT_HEIGHT);

  float min_distance = std::numeric_limits<float>::max();
  size_t closest_idx = 0;
  for (size_t i = 0; i < selected_points.size(); ++i) {
    float dist = std::sqrt(selected_points[i](0) * selected_points[i](0) +
                          selected_points[i](1) * selected_points[i](1));
    if (dist < MAX_OBJECT_DISTANCE && dist < min_distance) {
      min_distance = dist;
      closest_idx = i;
    }
  }
  
  if (min_distance >= std::numeric_limits<float>::max()) {
    RCLCPP_WARN(this->get_logger(), 
                "No points found within maximum distance of %.2f m", MAX_OBJECT_DISTANCE);
    return;
  }
  
  x = selected_points[closest_idx](0);
  y = selected_points[closest_idx](1);
  z = selected_points[closest_idx](2);

  // TODO: we need to convert this into base frame coordinates, not sensor
  float ground_distance = std::sqrt(x * x + y * y);
  float object_distance = ground_distance;
  float object_angle = std::atan2(y, x);

  RCLCPP_INFO(this->get_logger(),
              "Object at: (x=%.3f,y=%.3f,z=%.3f) ground_distance=%.2f m,"
              " angle=%.2f rad",
              x, y, z, ground_distance, object_angle);

  if (!navigate_to_pose_in_frame_client_->wait_for_action_server(
          std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(),
                 "NavigateToPoseInFrame action server not available!");
    result->success = false;
    result->message = "NavigateToPoseInFrame action server not available";
    goal_handle->abort(result);
    return;
  }

  float safety_distance = SAFETY_DISTANCE;
  float target_distance = std::max(0.0f, object_distance - safety_distance);
  float target_x = target_distance * std::cos(object_angle);
  float target_y = target_distance * std::sin(object_angle);

  RCLCPP_INFO(this->get_logger(), "Sending relative move goal: (%.2f, %.2f)",
              target_x, target_y);

  auto goal_msg = NavigateToPoseInFrameAction::Goal();
  goal_msg.x = target_x;
  goal_msg.y = target_y;
  goal_msg.yaw = object_angle;
  goal_msg.absolute = false;

  auto send_goal_options =
      rclcpp_action::Client<NavigateToPoseInFrameAction>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&LidarObjectNavigator::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&LidarObjectNavigator::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&LidarObjectNavigator::result_callback, this, _1);

  navigate_to_pose_in_frame_client_->async_send_goal(goal_msg,
                                                     send_goal_options);
}

void LidarObjectNavigator::goal_response_callback(
    const GoalHandleNavigateToPoseInFrame::SharedPtr& goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(),
                 "Goal was rejected by NavigateToPoseInFrameAction server");
    if (active_goal_handle_) {
      auto result = std::make_shared<NavigateViaLidar::Result>();
      result->success = false;
      result->message = "NavigateToPoseInFrameAction goal rejected";
      active_goal_handle_->abort(result);
    }
  } else {
    RCLCPP_INFO(
        this->get_logger(),
        "Goal accepted by NavigateToPoseInFrameAction server, navigating...");
  }
}

void LidarObjectNavigator::feedback_callback(
    GoalHandleNavigateToPoseInFrame::SharedPtr,
    const std::shared_ptr<const NavigateToPoseInFrameAction::Feedback>
        feedback) {
  RCLCPP_INFO(this->get_logger(), "Distance remaining: %.2f m",
              feedback->distance_remaining);
  if (active_goal_handle_) {
    auto fb = std::make_shared<NavigateViaLidar::Feedback>();
    fb->distance_remaining = feedback->distance_remaining;
    active_goal_handle_->publish_feedback(fb);
  }
}

void LidarObjectNavigator::result_callback(
    const GoalHandleNavigateToPoseInFrame::WrappedResult& result) {
  auto action_result = std::make_shared<NavigateViaLidar::Result>();
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

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<amiga_navigation::LidarObjectNavigator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
