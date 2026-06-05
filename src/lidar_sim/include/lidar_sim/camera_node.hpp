#ifndef WEBOTS_ROS2_CAMERA
#define WEBOTS_ROS2_CAMERA

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32.hpp"
 
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
 
#include <memory>
#include <string>

namespace bot {
class cameraNode {
    public: 
        cameraNode(rclcpp::Node *node);

    private:
    // ROS2 subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr aruco_id_publisher_;
    
    // OpenCV ArUco detector
    cv::Ptr<cv::aruco::Dictionary>dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters>detector_params_;
    
    // Image callback
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    // ArUco detection parameters
    int detected_marker_id_;
    bool marker_detected_;
    
    // Logger
    rclcpp::Logger logger_;
 
}; 
}
#endif