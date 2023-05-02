#!/bin/bash
source /opt/ros/melodic/setup.bash
source /home/$USER/MobileRobotSimulator/catkin_ws/devel/setup.bash
echo "Exporting ROS MASTER since->$1" 
export ROS_MASTER_URI=http://$1:11311
roslaunch simulator minibot.launch
