#include "lidar_sim/camera_node.hpp"
 
#include <iostream>
 
namespace bot {
 
cameraNode::cameraNode(rclcpp::Node *node)
    : detected_marker_id_(-1), marker_detected_(false),
      logger_(node->get_logger()) {
  // Initialize ArUco detector with default dictionary (DICT_6X6_250)
  dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
  detector_params_ = cv::aruco::DetectorParameters::create();
  detector_params_->minMarkerPerimeterRate = 0.80; // ignore until close
 
  // Create subscription to camera image topic
  image_subscription_ = node->create_subscription<sensor_msgs::msg::Image>(
      "camera/image_raw/image_color", rclcpp::SensorDataQoS().best_effort(),
      std::bind(&cameraNode::imageCallback, this, std::placeholders::_1));
 
  // Create publisher for detected ArUco marker IDs
  aruco_id_publisher_ = node->create_publisher<std_msgs::msg::Int32>(
      "/aruco/marker_id", rclcpp::SensorDataQoS().best_effort());
 
  RCLCPP_INFO(logger_, "cameraNode initialized. Listening on "
                       "camera/image_raw/image_color");
}
 
void cameraNode::imageCallback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  try {
    // Convert ROS image message to OpenCV Mat
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(
        msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat frame = cv_image->image;
 
    // Detect ArUco markers
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::aruco::detectMarkers(frame, dictionary_, marker_corners, marker_ids, detector_params_);
 
    // If markers detected
    if (!marker_ids.empty()) {
      // Get the first detected marker
      detected_marker_id_ = marker_ids[0];
      marker_detected_ = true;
 
      // Publish the marker ID
      auto msg_id = std_msgs::msg::Int32();
      msg_id.data = detected_marker_id_;
      aruco_id_publisher_->publish(msg_id);
 
      RCLCPP_DEBUG(logger_, "ArUco marker detected with ID: %d",
                   detected_marker_id_);
 
      // Optional: Draw markers on frame for visualization
      cv::aruco::drawDetectedMarkers(frame, marker_corners, marker_ids);
    } else {
      marker_detected_ = false;
      // Optionally publish -1 to indicate no marker detected
      // auto msg_id = std_msgs::msg::Int32();
      // msg_id.data = -1;
      // aruco_id_publisher_->publish(msg_id);
    }
 
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(logger_, "cv_bridge exception: %s", e.what());
  }
}
 
} // namespace bot