#include "ros/ros.h"
#include <ros/package.h>
#include "simulator/Parameters.h"
#include "../utilities/simulator_structures.h"
#include "simulator/simulator_manipulator.h"
#include "simulator/simulator_object_interaction.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#define GRASP 1
#define RELEASE 0
parameters params;

bool object_interaction(int action, const char name[50])
{
    ros::NodeHandle n;
    ros::ServiceClient client;
    simulator::simulator_object_interaction srv;
    client = n.serviceClient<simulator::simulator_object_interaction>("simulator_object_interaction");
    std::string s;
    s=name;
    srv.request.name = s;
    srv.request.grasp = action;

    if( !client.call(srv) )
    {
        ROS_ERROR("Failed to call service simulator_object_interaction");
    }
    printf("%d\n",srv.response.done );
    return srv.response.done;
}

bool manipulator(simulator::simulator_manipulator::Request  &req ,simulator::simulator_manipulator::Response &res)
{
    char str[300];
    //sscanf(result.c_str(),"%s %s %s",ROS_System,action,object);
    //printf("%s object %s\n",action,object);

    if( strcmp(req.action.c_str(),"grab") == 0 )
    {
        if(object_interaction(GRASP,req.object.c_str()))
        {
            printf("object %s grasped\n",req.object.c_str());
        }
        else
        {
            printf("object %s not grasped\n",req.object.c_str());
        }

        sprintf(str,"(assert (answer %s command %s %s 1))",req.ROS_System.c_str(),req.action.c_str(),req.object.c_str());
        res.answer = str;
    }
    else if(strcmp(req.action.c_str(),"drop")==0)
    {

        if(object_interaction(RELEASE,req.object.c_str()))
        {
            printf("object %s released\n",req.object.c_str());
        }
        else
        {
            printf("object %s not released\n",req.object.c_str());
        }

        sprintf(str,"(assert (answer %s command %s %s %f %f %f 1))",req.ROS_System.c_str(),req.action.c_str(),req.object.c_str(),params.robot_x ,params.robot_y ,params.robot_theta);
        res.answer = str;
    } 
    else
    {
        ROS_ERROR("Operation unknow");
        res.answer = "FAIL";
    }


    return true;
}


void paramsCallback(const simulator::Parameters::ConstPtr& paramss)
{
  params.robot_x             = paramss->robot_x   ;
  params.robot_y             = paramss->robot_y   ;
  params.robot_theta         = paramss->robot_theta   ;    
  params.robot_radio         = paramss->robot_radio   ;    
  params.robot_max_advance   = paramss->robot_max_advance   ;          
  params.robot_turn_angle    = paramss->robot_turn_angle   ;         
  params.laser_num_sensors   = paramss->laser_num_sensors   ;          
  params.laser_origin        = paramss->laser_origin         ;     
  params.laser_range         = paramss->laser_range   ;    
  params.laser_value         = paramss->laser_value   ;    
  strcpy(params.world_name ,paramss -> world_name.c_str());       
  params.noise               = paramss->noise   ;   
  params.run                 = paramss->run   ; 
  params.light_x             = paramss->light_x;
  params.light_y             = paramss->light_y;
  params.behavior            = paramss->behavior; 
}

int main(int argc, char *argv[])
{	
   ros::init(argc, argv, "simulator_manipulator");
   ros::NodeHandle n;
   ros::ServiceServer service = n.advertiseService("simulator_manipulator", manipulator);
   ros::Subscriber params_sub = n.subscribe("simulator_parameters_pub", 0, paramsCallback);
   ros::spin();
   return 0;
}

