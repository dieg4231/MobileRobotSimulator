#!/bin/bash
source /opt/ros/melodic/setup.bash
python3 /home/$USER/MobileRobotSimulator/message.py & PIDIOS=$!
python3 /home/$USER/MobileRobotSimulator/robot_scanner.py & PIDMIX=$!
wait $PIDIOS
wait $PIDMIX
