#!/bin/bash
# Source this script on the Jetson before running the Gazebo simulator stack.
# It selects CycloneDDS to match the Mac simulator and sets the ROS_DOMAIN_ID.

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Optional: uncomment and set CYCLONEDDS_URI if multicast discovery is flaky.
# export CYCLONEDDS_URI=file:///workspaces/ros2_ws/cyclonedds.xml

echo "Jetson sim environment ready."
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "  RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo ""
echo "Run the sim stack with:"
echo "  ros2 launch stereo_depth_ros2 stereo_vio_navigation_sim.launch.py"
