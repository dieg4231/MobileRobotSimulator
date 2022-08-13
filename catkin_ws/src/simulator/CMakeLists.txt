cmake_minimum_required(VERSION 2.8.3)
project(simulator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf
  message_generation
  clips_ros
  joy
)


find_package(CGAL REQUIRED COMPONENTS Core)
set(CGAL_DONT_OVERRIDE_CMAKE_FLAGS TRUE CACHE BOOL "Don't override flags")
message(CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS})
include(${CGAL_USE_FILE})
message(CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS})

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   poseCustom.msg
   PosesArray.msg
   Parameters.msg
   Laser_values.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   simulator_robot_step.srv
   simulator_parameters.srv
   simulator_base.srv
   simulator_laser.srv
   simulator_light.srv
   simulator_robot_laser_values.srv
   simulator_algorithm_result.srv
   simulator_stop.srv
   simulator_set_light_position.srv
   simulator_MoveRealRobot.srv
   simulator_object_interaction.srv
   simulator_find_obj.srv
   simulator_manipulator.srv
   auto_charge.srv
   return_home.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simulator
#  CATKIN_DEPENDS roscpp rospy std_msgs clips
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(motion_planner_node src/motion_planner/motion_planner_node.cpp)
target_link_libraries(motion_planner_node ${catkin_LIBRARIES})
add_dependencies(motion_planner_node simulator_generate_messages_cpp)

add_executable(base_node src/simulator_physics/base_node.cpp)
target_link_libraries(base_node ${catkin_LIBRARIES})
add_dependencies(base_node simulator_generate_messages_cpp)

add_executable(laser_node src/simulator_physics/laser_node.cpp)
target_link_libraries(laser_node ${catkin_LIBRARIES})
add_dependencies(laser_node simulator_generate_messages_cpp)

add_executable(light_server src/simulator_physics/light_server.cpp)
target_link_libraries(light_server ${catkin_LIBRARIES})
add_dependencies(light_server simulator_generate_messages_cpp)

add_executable(light_node src/simulator_physics/light_node.cpp)
target_link_libraries(light_node ${catkin_LIBRARIES})
add_dependencies(light_node simulator_generate_messages_cpp)

add_executable(move_turtlebot_node src/turtlebot/move_turtlebot_node.cpp)
target_link_libraries(move_turtlebot_node ${catkin_LIBRARIES})
add_dependencies(move_turtlebot_node simulator_generate_messages_cpp)

add_executable(move_minibot_node src/turtlebot/move_minibot_node.cpp)
target_link_libraries(move_minibot_node ${catkin_LIBRARIES})
add_dependencies(move_minibot_node simulator_generate_messages_cpp)

add_executable(map_rviz_node src/visualization/map_rviz_node.cpp)
target_link_libraries(map_rviz_node ${catkin_LIBRARIES})
add_dependencies(map_rviz_node simulator_generate_messages_cpp)

add_executable(objs_viz_node src/visualization/objs_viz_node.cpp)
target_link_libraries(objs_viz_node ${catkin_LIBRARIES})
add_dependencies(objs_viz_node simulator_generate_messages_cpp)

add_executable(find_obj_node src/simulator_physics/find_obj_node.cpp)
target_link_libraries(find_obj_node ${catkin_LIBRARIES})
add_dependencies(find_obj_node simulator_generate_messages_cpp)

add_executable(manipulator_node src/simulator_physics/manipulator_node.cpp)
target_link_libraries(manipulator_node ${catkin_LIBRARIES})
add_dependencies(manipulator_node simulator_generate_messages_cpp)

add_executable(action_planner_node src/motion_planner/action_planner_node.cpp)
target_link_libraries(action_planner_node ${catkin_LIBRARIES})
add_dependencies(action_planner_node simulator_generate_messages_cpp)

add_executable(auto_charge_server src/motion_planner/auto_charge_server.cpp)
target_link_libraries(auto_charge_server ${catkin_LIBRARIES})
add_dependencies(auto_charge_server simulator_generate_messages_cpp)

add_executable(return_home_server src/motion_planner/return_home_server.cpp)
target_link_libraries(return_home_server ${catkin_LIBRARIES})
add_dependencies(return_home_server simulator_generate_messages_cpp)

add_executable(joy_teleop_node src/joy_teleop/joy_teleop_node.cpp)
target_link_libraries(joy_teleop_node ${catkin_LIBRARIES})
