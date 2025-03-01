#!/bin/bash

cd ~/Documents/2024-tobias-bot/catkin_ws 
source ~/Documents/2024-tobias-bot/catkin_ws/devel/setup.bash

sudo chmod 777 /dev/ttyACM0

echo "Waiting 10 seconds for ROS core"
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311

gnome-terminal -e "bash -c \"roscore; exec bash\""
sleep 10
#roslaunch tobias wiredControl.launch
# gnome-terminal -e "roslaunch realsense2_camera rs_camera.launch camera:=cam_1 serial_no:=033322071026"
# gnome-terminal -e "roslaunch realsense2_camera rs_camera.launch camera:=cam_2 serial_no:=048522075108"

roslaunch tobias wiredControl.launch
