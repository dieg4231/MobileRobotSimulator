#include "simulator/simulator_robot_step.h"
#include "simulator/simulator_parameters.h"


parameters get_params()
{
  parameters params;

  ros::NodeHandle n;
  ros::ServiceClient client;
  simulator::simulator_parameters srv;
  client = n.serviceClient<simulator::simulator_parameters>("simulator_get_parameters"); //create the client
  
  srv.request.request=1;
  params.run=0;

    if (client.call(srv))
    {
      params.robot_x             = srv.response.robot_x   ;
      params.robot_y             = srv.response.robot_y   ;
      params.robot_theta         = srv.response.robot_theta   ;    
      params.robot_radio         = srv.response.robot_radio   ;    
      params.robot_max_advance   = srv.response.robot_max_advance   ;          
      params.robot_turn_angle    = srv.response.robot_turn_angle   ;         
      params.laser_num_sensors   = srv.response.laser_num_sensors   ;          
      params.laser_origin        = srv.response.laser_origin         ;     
      params.laser_range         = srv.response.laser_range   ;    
      params.laser_value         = srv.response.laser_value   ;    
      strcpy(params.world_name,srv.response.world_name.c_str());       
      params.noise               = srv.response.noise   ;   
      params.run                 = srv.response.run   ; 
      params.light_x             = srv.response.light_x;
      params.light_y             = srv.response.light_y;
      params.behavior            = srv.response.behavior; 
      params.steps               = srv.response.steps;
    }
    else
    {
      ROS_ERROR("Failed to call service get_parameters");
      
    }
  

  printf("Inicial parameters:\n" );
  printf("robot_x %f \n",params.robot_x )       ;   
  printf("robot_y %f \n",params.robot_y )      ;
  printf("robot_theta %f \n",params.robot_theta             );
  printf("robot_radio %f \n",params.robot_radio             );
  printf("robot_max_advance %f \n",params.robot_max_advance  );           
  printf("robot_turn_angle %f \n",params.robot_turn_angle  );           
  printf("laser_num_sensors %d \n",params.laser_num_sensors);             
  printf("laser_origin %f \n",params.laser_origin        )        ;     
  printf("laser_range %f \n",params.laser_range        );    
  printf("laser_value %f \n",params.laser_value        );    
  printf("world_name %s \n",params.world_name);       
  printf("noise %d \n",params.noise  );   
  printf("run %d \n",params.run    ) ;     
  printf("light_x %f \n",params.light_x    );
  printf("light_y %f \n",params.light_y   );
  printf("behavior %d \n",params.behavior ); 

  return params;
}


int move_gui(float angle ,float distance)
{
  
  ros::NodeHandle n;
  ros::ServiceClient client;
  simulator::simulator_robot_step srv;
  client = n.serviceClient<simulator::simulator_robot_step>("simulator_robot_step"); //create the client
  
  srv.request.theta=angle;
  srv.request.distance=distance;
  
  if (client.call(srv))
  {
      printf("%s\n","Echo" );
  }
  else
  {
    ROS_ERROR("Failed to call service get_parameters");
    
  }
  

  return 1;
}