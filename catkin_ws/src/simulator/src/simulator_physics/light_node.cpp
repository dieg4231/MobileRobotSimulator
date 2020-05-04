
/***********************************************
*                                              *
*      light_node.cpp                          *
*                                              *
*      Jesus Savage                            *
*      Diego Cordero                           *
*                                              *
*              Bio-Robotics Laboratory         *
*              UNAM, 2019                      *
*                                              *
*                                              *
************************************************/



#include "ros/ros.h"
#include "simulator/Parameters.h"
#include "../utilities/simulator_structures.h"
#include "simulator/simulator_light.h"

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

parameters params;

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



bool get_intensities(simulator::simulator_light::Request  &req ,simulator::simulator_light::Response &res)
{
 
	float x,y;
	int sensor;
	float step = 3.1415/4; 
	float values[8];
	int i;

	x = params.robot_radio * cos(params.robot_theta) + params.robot_x;
    y = params.robot_radio * sin( params.robot_theta ) + params.robot_y;
    res.values[0] = values[0] =  1 / sqrt( pow(x - params.light_x ,2) + pow(y - params.light_y,2));

	for(int i = 1; i < 8; i++)
	{
		x = params.robot_radio * cos( params.robot_theta + i * step) + params.robot_x;
		y = params.robot_radio * sin( params.robot_theta + i * step) + params.robot_y;
		res.values[i] = values[i] =  1 / sqrt( pow(x - params.light_x ,2) + pow(y - params.light_y,2));
	}
	sensor = 0;

	for(int i = 1; i < 8; i++)
	{
		if( values[i] > values[sensor])
			sensor = i;
	}
	return true;
}




int main(int argc, char *argv[])
{	
	ros::init(argc, argv, "simulator_light_node");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("simulator_light",get_intensities);
	ros::Subscriber params_sub = n.subscribe("simulator_parameters_pub", 0, paramsCallback);
	ros::spin();

	return 0;
}
