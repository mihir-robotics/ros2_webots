cmake_minimum_required(VERSION 3.5)
project(lidar_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Besides the package specific dependencies we also need the `pluginlib` and `webots_ros2_driver`
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(webots_ros2_driver REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/Vel.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

# Export the plugin configuration file
pluginlib_export_plugin_description_file(webots_ros2_driver my_robot_driver.xml)

# webot library
add_library(
  ${PROJECT_NAME}_lib
  SHARED
  src/webot.cpp
)
target_include_directories(
  ${PROJECT_NAME}_lib
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}_lib
  pluginlib
  rclcpp
  webots_ros2_driver
)

# Ensure the message interfaces are generated before building the library
add_dependencies(${PROJECT_NAME}_lib ${PROJECT_NAME})

# Link the generated typesupport library
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_lib "${cpp_typesupport_target}")

install(TARGETS
  ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# Install additional directories.
install(DIRECTORY
  launch
  resource
  worlds
  DESTINATION share/${PROJECT_NAME}/
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}_lib
)
ament_export_dependencies(
  rosidl_default_runtime_cpp
  rosidl_runtime_cpp
)
ament_package()