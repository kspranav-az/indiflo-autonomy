#!/bin/bash
# Source this script on the Jetson before running the Gazebo simulator stack.
# It sets the ROS_DOMAIN_ID to match the Mac simulator.
#
# RMW note: the Mac simulator was built with CycloneDDS only. If
# ros-humble-rmw-cyclonedds-cpp is installed on the Jetson, uncomment the
# RMW_IMPLEMENTATION line below for best compatibility. If the install fails
# with a 404, leave it commented and the Jetson will use its default RMW
# (Fast-DDS). Cross-vendor DDS discovery may work for topic listing but can be
# unreliable under heavy image traffic.

export ROS_DOMAIN_ID=42
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Optional: uncomment and set CYCLONEDDS_URI if multicast discovery is flaky.
# export CYCLONEDDS_URI=file:///workspaces/ros2_ws/cyclonedds.xml

echo "Jetson sim environment ready."
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
if [ -n "$RMW_IMPLEMENTATION" ]; then
    echo "  RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
else
    echo "  RMW_IMPLEMENTATION=(default - CycloneDDS install pending)"
fi
echo ""
echo "Run the sim stack with:"
echo "  ros2 launch stereo_depth_ros2 stereo_vio_navigation_sim.launch.py"
