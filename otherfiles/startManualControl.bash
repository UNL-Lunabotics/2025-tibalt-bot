!/bin/bash

source ~/Documents/2022-terrence/catkin_ws/devel/setup.bash

sudo chmod 777 /dev/ttyACM0

echo "Waiting 10 seconds for ROS core"
gnome-terminal -e "bash -c \"roscore; exec bash\""
sleep 10
roslaunch terrence wiredControl.launch
