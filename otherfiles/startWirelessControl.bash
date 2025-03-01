#!/bin/bash

cd ~/Documents/2024-tobias-bot/catkin_ws 
source ./devel/setup.bash

sudo chmod 777 /dev/ttyACM0

# takes a few seconds for ROS core to start - if the wait isn't long enough 
# the following commands will error
echo "Waiting 10 seconds for ROS core"
# export ROS_HOSTNAME=rmccontrolstation
export ROS_MASTER_URI=http://rmccontrolstation:11311
gnome-terminal -e "bash -c \"roscore; exec bash\""
sleep 10

# start up both cams
gnome-terminal -e "roslaunch realsense2_camera rs_camera.launch camera:=cam_1 serial_no:=033322071026"
gnome-terminal -e "roslaunch realsense2_camera rs_camera.launch camera:=cam_2 serial_no:=048522075108"


roslaunch tobias wirelessControl.launch
