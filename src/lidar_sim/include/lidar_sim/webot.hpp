#ifndef WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP
#define WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP

#include "rclcpp/macros.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"
#include <webots/distance_sensor.h>

#define TIME_STEP 32

namespace my_robot_driver {
class webot : public webots_ros2_driver::PluginInterface {
public:
  void step() override;
  void init(webots_ros2_driver::WebotsNode *node,
            std::unordered_map<std::string, std::string> &parameters) override;

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
  
  geometry_msgs::msg::Twist cmd_vel_msg;
  sensor_msgs::msg::LaserScan latest_scan_;
  
  WbDeviceTag lidar;
  WbDeviceTag right_motor;
  WbDeviceTag left_motor;
  WbDeviceTag ds0;
  WbDeviceTag ds1;
};
} // namespace my_robot_driver
#endif