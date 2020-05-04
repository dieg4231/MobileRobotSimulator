#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "ros/ros.h"
#include <ros/package.h>
#include "simulator/Parameters.h"
#include "../utilities/simulator_structures.h"
#include "simulator/simulator_find_obj.h"
#include "clips_ros/SimuladorRepresentation.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

bool find_object(simulator::simulator_find_obj::Request  &req ,simulator::simulator_find_obj::Response &res)
{

    char str[300];
    sprintf(str,"(assert (answer %s command %s %s %f %f %f %s 1))",req.ROS_System.c_str(),req.action.c_str(),req.object.c_str(),req.x,req.y,req.z,req.arm.c_str());
    printf("\nSend fact %s\n",str);
    res.answer = str;

    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "simulator_find_obj");
    ros::NodeHandle n;
    SimuladorRepresentation::setNodeHandle(&n);
    ros::ServiceServer service = n.advertiseService("simulator_find_obj", find_object);
    ros::spin();
    return 0;
}

