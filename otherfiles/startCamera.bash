#!/bin/bash

cd ~/Documents/camera/catkin_ws 
source ./devel/setup.bash

# ROS_MASTER_URI tells nodes where they can find the master node
export ROS_MASTER_URI=http://rmccontrolstation:11311
gnome-terminal -e "bash -c \"roscore; exec bash\""

gnome-terminal -e "source ~/Documents/fbispycam/catkin_ws/devel/setup.bash; \
                    roslaunch realsense2_camera tabitha_cam.launch camera:=cam_1 serial_no:=728312070349"
gnome-terminal -e "source ~/Documents/fbispycam/catkin_ws/devel/setup.bash; \
                    roslaunch realsense2_camera tabitha_cam.launch camera:=cam_2 serial_no:=728312070349" 
