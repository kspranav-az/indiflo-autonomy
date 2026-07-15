cmake_minimum_required(VERSION 3.8)
project(perception_viz)

find_package(ament_cmake REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/depth_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install RViz configs
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_package()
