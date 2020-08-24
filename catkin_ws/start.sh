#!/bin/bash
#Script to start the ros simulations 
xterm -hold -e ". devel/setup.bash && roscore" &
xterm -hold -e ". devel/setup.bash && roscd simulator/src/gui/ && python simulator_node.py" &
sleep 3
xterm -hold -e ". devel/setup.bash  && rosrun simulator light_node" &
xterm -hold -e ". devel/setup.bash  && rosrun simulator laser_node" &
xterm -hold -e ". devel/setup.bash  && rosrun simulator base_node" &
xterm -hold -e ". devel/setup.bash  && rosrun clips_ros ros_pyclips_node.py" & 
xterm -hold -e ". devel/setup.bash  && rosrun simulator find_obj_node" & 
xterm -hold -e ". devel/setup.bash  && rosrun simulator manipulator_node" & 
sleep 3
xterm -hold -e ". devel/setup.bash && rosrun simulator motion_planner_node" 
#xterm -hold -e ". devel/setup.bash && rosrun simulator move_turtlebot_node" 
