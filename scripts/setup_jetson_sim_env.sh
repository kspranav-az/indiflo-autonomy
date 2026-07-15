#!/bin/bash
# Source this script on the Jetson before running the Gazebo simulator stack.
# It sets the ROS_DOMAIN_ID to match the Mac simulator and selects CycloneDDS
# so both machines use the same DDS vendor.

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# CycloneDDS config: unicast peer discovery on domain 42 for the wired USB
# Ethernet link to the Mac simulator. The Jetson side binds to its wired
# interface (192.168.55.7) so it does not accidentally use Wi-Fi, l4tbr0, or
# docker0. Peers include loopback, the Mac (192.168.55.14), and the Jetson
# itself so local nodes can discover each other. Multicast SPDP is disabled.
export CYCLONEDDS_URI=file:///workspaces/ros2_ws/cyclonedds.xml

# Source local rmw_cyclonedds_cpp build if available (apt package is 404).
if [ -f "$HOME/rmw_ws/install/setup.bash" ]; then
    source "$HOME/rmw_ws/install/setup.bash"
fi

echo "Jetson sim environment ready."
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
if [ -n "$RMW_IMPLEMENTATION" ]; then
    echo "  RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
else
    echo "  RMW_IMPLEMENTATION=(Jetson default)"
fi
echo ""
echo "Run the sim stack with:"
echo "  ros2 launch stereo_depth_ros2 stereo_vio_navigation_sim.launch.py"
