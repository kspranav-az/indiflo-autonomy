cmake_minimum_required(VERSION 3.8)
project(stereo_depth_ros2)

# Set C++ standard and compile options
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Stereo depth node
add_executable(stereo_depth_node
  src/stereo_depth_node.cpp
)

target_include_directories(stereo_depth_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

set(stereo_depth_dependencies
  "rclcpp"
  "sensor_msgs"
  "cv_bridge"
  "ament_index_cpp"
)

ament_target_dependencies(stereo_depth_node ${stereo_depth_dependencies})
target_link_libraries(stereo_depth_node ${OpenCV_LIBS})

# Fake odometry node
add_executable(fake_odom_node
  src/fake_odom_node.cpp
)

set(fake_odom_dependencies
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
)

ament_target_dependencies(fake_odom_node ${fake_odom_dependencies})

# Install executables
install(TARGETS stereo_depth_node fake_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config
install(
  DIRECTORY cfg/
  DESTINATION share/${PROJECT_NAME}/cfg
)

# Install openvins configs
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_dependencies(${stereo_depth_dependencies} ${fake_odom_dependencies})
ament_package()
