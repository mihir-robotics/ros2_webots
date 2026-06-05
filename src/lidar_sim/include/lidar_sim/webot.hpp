#ifndef WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP
#define WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP

#include "rclcpp/macros.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"


#include "lidar_sim/msg/vel.hpp"
#include "lidar_sim/camera_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_robot_driver {
class webot : public webots_ros2_driver::PluginInterface {
public:
  void step() override;
  void init(webots_ros2_driver::WebotsNode *node,
            std::unordered_map<std::string, std::string> &parameters) override;

private:
  void cmdVelCallback(const lidar_sim::msg::Vel::SharedPtr msg);
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  rclcpp::Subscription<lidar_sim::msg::Vel>::SharedPtr cmd_vel_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;

  // ArUco Camera Node
  std::unique_ptr<cameraNode> aruco_camera_;
  
  lidar_sim::msg::Vel cmd_vel_msg;
  sensor_msgs::msg::LaserScan latest_scan_;

  float front_distance_;
  float left_distance_;
  float right_distance_;
  
  WbDeviceTag lidar;
  WbDeviceTag right_motor;
  WbDeviceTag left_motor;
};
} // namespace my_robot_driver
#endif