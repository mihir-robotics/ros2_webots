#include "lidar_sim/webot.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cstdio>
#include <functional>
#include <webots/motor.h>
#include <webots/robot.h>

#define HALF_DISTANCE_BETWEEN_WHEELS 0.045
#define WHEEL_RADIUS 0.025

namespace bot {
void webot::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters) {

  right_motor = wb_robot_get_device("right wheel motor");
  left_motor = wb_robot_get_device("left wheel motor");

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);

  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(right_motor, 0.0);

  cmd_vel_subscription_ = node->create_subscription<lidar_sim::msg::Vel>(
      "/bot/velocity", rclcpp::SensorDataQoS().reliable(),
      std::bind(&webot::cmdVelCallback, this, std::placeholders::_1));

    // Lidar subscription
  laser_subscription_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
      "/bot/scan", rclcpp::SensorDataQoS().best_effort(),
      std::bind(&webot::laserCallback, this, std::placeholders::_1));

    // Initialize ArUco Camera Node
    // Create a dedicated node for the camera:
    camera_ros_node_ = rclcpp::Node::make_shared("camera_node");
    aruco_camera_ = std::make_unique<cameraNode>(camera_ros_node_.get());
    camera_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    camera_executor_->add_node(camera_ros_node_);

}

void webot::cmdVelCallback(
    const lidar_sim::msg::Vel::SharedPtr msg) {
  cmd_vel_msg.linear = msg->linear;
  cmd_vel_msg.angular = msg->angular;
}

void webot::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // Store latest scan for obstacle detection
  latest_scan_ = *msg;
  float front_min = std::numeric_limits<float>::infinity();
  float left_min  = std::numeric_limits<float>::infinity();
  float right_min = std::numeric_limits<float>::infinity();

  for (size_t i = 0; i < msg->ranges.size(); i++) {

    float range = msg->ranges[i];

    if (!std::isfinite(range))
      continue;

    float angle =
      msg->angle_min + i * msg->angle_increment;

    // Front sector ±20°
    if (fabs(angle) < 20.0 * M_PI / 180.0)
      front_min = std::min(front_min, range);

    // Left sector
    else if (angle > 20.0 * M_PI / 180.0)
      left_min = std::min(left_min, range);

    // Right sector
    else
      right_min = std::min(right_min, range);
  }

  front_distance_ = front_min;
  left_distance_ = left_min;
  right_distance_ = right_min;
}

void webot::step() {
  // Call cameraNode executor
  camera_executor_->spin_some(std::chrono::nanoseconds(0));

  auto forward_speed = cmd_vel_msg.linear;
  auto angular_speed = cmd_vel_msg.angular;
  
  auto command_motor_left =
  (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) /
  WHEEL_RADIUS;
  auto command_motor_right =
  (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) /
  WHEEL_RADIUS;
  
double safe_distance = 0.25; // meters
float turn_ratio = 0.015;

if (front_distance_ < safe_distance) {

    // Obstacle ahead

    if (left_distance_ > right_distance_) {
        // More space on left
        wb_motor_set_velocity(left_motor, -command_motor_left*turn_ratio);
        wb_motor_set_velocity(right_motor, command_motor_right);
    } else {
        // More space on right
        wb_motor_set_velocity(left_motor, command_motor_left);
        wb_motor_set_velocity(right_motor, -command_motor_right*turn_ratio);
    }

} else {

    wb_motor_set_velocity(left_motor, command_motor_left);
    wb_motor_set_velocity(right_motor, command_motor_right);
}
}


} // namespace bot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(bot::webot,
                       webots_ros2_driver::PluginInterface)