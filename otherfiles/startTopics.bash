#!/bin/bash
# orin2_launch.sh

echo "Setting up ROS2 networking environment..."
source ~/2025-terrence-bot/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

echo "Launching main robot control..."
ros2 launch control_topic topics_launch.py