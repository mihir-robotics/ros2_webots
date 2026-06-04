#include "lidar_sim/webot.hpp"

#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <functional>
#include <webots/motor.h>
#include <webots/robot.h>

#define HALF_DISTANCE_BETWEEN_WHEELS 0.045
#define WHEEL_RADIUS 0.025

namespace my_robot_driver {
void webot::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters) {

  right_motor = wb_robot_get_device("right wheel motor");
  left_motor = wb_robot_get_device("left wheel motor");

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);

  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(right_motor, 0.0);

  ds0 = wb_robot_get_device("ds0");
  ds1 = wb_robot_get_device("ds1");
  wb_distance_sensor_enable(ds0, TIME_STEP);
  wb_distance_sensor_enable(ds1, TIME_STEP);

  cmd_vel_subscription_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::SensorDataQoS().reliable(),
      std::bind(&webot::cmdVelCallback, this, std::placeholders::_1));

    // Lidar subscription
  laser_subscription_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS().best_effort(),
      std::bind(&webot::laserCallback, this, std::placeholders::_1));

}

void webot::cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  cmd_vel_msg.linear = msg->linear;
  cmd_vel_msg.angular = msg->angular;
}

void webot::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // Store latest scan for obstacle detection
  latest_scan_ = *msg;
}

void webot::step() {
  double ds0_value = wb_distance_sensor_get_value(ds0);
  double ds1_value = wb_distance_sensor_get_value(ds1);

  // Sensor lookupTable: 0-1020 at 0-0.05m, then 0 at 0.15m
  // Higher values = closer object
  double threshold = 300; // Adjust based on your needs

  auto forward_speed = cmd_vel_msg.linear.x;
  auto angular_speed = cmd_vel_msg.angular.z;
  
  auto command_motor_left =
  (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) /
  WHEEL_RADIUS;
  auto command_motor_right =
  (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) /
  WHEEL_RADIUS;
  
  // Obstacle avoidance logic
  if (ds0_value > threshold || ds1_value > threshold) {
    // Obstacle detected - stop and turn
    if (ds0_value > threshold) {
      // Left sensor sees obstacle - turn right
        wb_motor_set_velocity(left_motor, command_motor_left);
        wb_motor_set_velocity(right_motor, -command_motor_right*0.50);
    } else {
      // Right sensor sees obstacle - turn left
        wb_motor_set_velocity(left_motor, -command_motor_left*0.50);
        wb_motor_set_velocity(right_motor, command_motor_right);
    }
  }
  else {
    wb_motor_set_velocity(left_motor, command_motor_left);
    wb_motor_set_velocity(right_motor, command_motor_right);
}
}


} // namespace my_robot_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_robot_driver::webot,
                       webots_ros2_driver::PluginInterface)