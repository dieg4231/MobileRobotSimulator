cmake_minimum_required(VERSION 2.8.3)
project(clips_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib 
  std_msgs
  std_srvs
  message_generation
  geometry_msgs 
)


## Generate messages in the 'msg' folder
file(GLOB MESSAGE_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/msg ${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg)
add_message_files(FILES ${MESSAGE_FILES})

## Generate services in the 'srv' folder
file(GLOB SERVICE_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/srv ${CMAKE_CURRENT_SOURCE_DIR}/srv/*.srv)
add_service_files(FILES ${SERVICE_FILES})

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

catkin_package(
  INCLUDE_DIRS src
  LIBRARIES clips_ros
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs std_srvs
#  DEPENDS system_lib
)

include_directories(
  src 
  ${catkin_INCLUDE_DIRS}
)

add_library(clips_ros
  src/SimuladorRepresentation.cpp
)

add_dependencies(clips_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(clips_ros
  ${catkin_LIBRARIES}

)
