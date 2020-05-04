/***********************************************
*                                              *
*      action_planner.h                        *
*                                              *
*      Jesus Savage                            *
*      Julio Cruz                              *
*                                              *
*              Bio-Robotics Laboratory         *
*              UNAM, 2020                      *
*                                              *
*                                              *
************************************************/


#include "ros/ros.h"
#include "simulator/simulator_find_obj.h"
#include "simulator/simulator_manipulator.h"
#include <iostream>
#include <vector>
#include <ctime>
#include <map>
#include <sstream>
#include <string>

#ifndef PI
#define PI 3.1415926535f
#endif


// It starts the communication with the CLIPS node
int start_clips_node_action_planner(){

 bool init_kdb = false;
 std::string file;
 std::string result;

 std::cout << "Starting CLIPS" << std::endl;


 //This functions loads initial facts and rules from a file
 // The first parameter is a file that contains the names of CLIPS files *.clp
 // The second parameter indicates with false do not start executing the loaded files
 // The third parameter is a timeout
 //file = "/src/expert_system/oracle.dat";
 file = "/src/action_planner/ViRBot_Cubes_ROS/ROS_cubes.dat";
 init_kdb = SimuladorRepresentation::initKDB(file, false, 2000);
 if(!init_kdb){
                std::cout << "CLIPS error file not found: " << file  << std::endl;
                return 0;
 }

 //Function to RESET CLIPS
 SimuladorRepresentation::resetCLIPS(true);

 //Function to print facts 
 SimuladorRepresentation::factCLIPS(true);

 //Function to print the loaded rules' names
 SimuladorRepresentation::ruleCLIPS(true);

 //Function to start running Clips
 SimuladorRepresentation::runCLIPS(true);

 //Function to asserting a fact to the clips node to check if Clips is alive
 SimuladorRepresentation::strQueryKDB("(assert (alive clips))", result, 10000);

 std::cout << "CLIPS answer: " << result << std::endl;

}



/*
This function is used to calculate the rotation angle for the Mvto command
*/
float get_angle(float ang,float c,float d,float X,float Y){
        float x,y;
        x=c-X;
        y=d-Y;
        if((x == 0) && (y == 0)) return(0);
        if(fabs(x)<0.0001)      return((float) ((y<0.0f)? 3*PI/2  : PI/2) - ang );
        else{
                if(x>=0.0f&&y>=0.0f) return( (float)(atan(y/x)-ang) );
                else if(x< 0.0f&&y>=0.0f) return( (float)(atan(y/x)+PI-ang) );
                else if(x< 0.0f&&y<0.0f) return( (float)(atan(y/x)+PI-ang) );
                else return( (float)(atan(y/x)+2*PI-ang));
        }
}



void get_distance_theta(float x,float y,float angle,float x1,float y1,float *distance,float *theta){

 // it calculates the distance
 *distance=(float)sqrt((x-x1)*(x-x1)+(y-y1)*(y-y1));
 printf("Distance: %f\n",*distance);

 // it calculates the rotation angle
 *theta=get_angle(angle,x,y,x1,y1);
 printf("rotation angle: %f\n",*theta);

}



void action_planner(float px, float py, float theta, movement *movements){

       static int mini_sm=1;
       static char object_name[200];
       static int init_flg=1;
       std::string result;
       static int i=0,j=0,k=0;
       char str[300];
       char action[30];
       char ROS_System[30];
       char room[30];
       char zone[30];
       static float x,y,z=0.0;
       static char arm[30];
       static char object[100];
       static float distance=1.0;
       static float angle=0.0;

       movements->twist = 0.0;
       movements->advance = 0.0;

       if(init_flg==1){

       		// It starts the communication with the Clips node
       		start_clips_node_action_planner();
       		init_flg=0;
       		strcpy(arm,"manipulator");

        	SimuladorRepresentation::strQueryKDB("(assert (start action-planning))", result, 10000);
          std::cout << "CLIPS answer: " << result << std::endl;
       }

       i++;
       sprintf(str,"(assert (step %d ))",i);
       printf("\nSend fact %s\n",str);
       SimuladorRepresentation::strQueryKDB(str, result, 10000);
       printf("\nCLIPS answer: %d %s\n",i,result.c_str());

       sscanf(result.c_str(),"%s %s",ROS_System,action);
       printf("ROS_System %s action %s\n",ROS_System,action);

       if(strcmp(action,"goto")==0){

       		// ACT-PLN goto bedroom deposit 0.505 0.45 30000 4 
       		sscanf(result.c_str(),"%s %s %s %s %f %f",ROS_System,action,room,zone,&x,&y);
       		printf("Room %s zone %s x %f y %f\n",room,zone,x,y);
       		printf("Pose x %f y %f theta %f\n",px,py,theta);

		get_distance_theta(x,y,theta,px,py,&distance,&angle);
		printf("goto angle %f distance %f\n",angle,distance);

		movements->twist = angle;
                movements->advance = distance;


		//answer ?sender command goto ?room ?zone ?x ?y ?flg
		sprintf(str,"(assert (answer %s command %s %s %s %f %f 1))",ROS_System,action,room,zone,x,y);
       		printf("\nSend fact %s\n",str);
       		SimuladorRepresentation::strQueryKDB(str, result, 10000);
       		printf("\nCLIPS answer: %d %s\n",i,result.c_str());


       }

       
    if(strcmp(action,"find_object")==0){

		//ACT-PLN find_object blockB 30000 4 
        sscanf(result.c_str(),"%s %s %s %f %f %f",ROS_System,action,object,&x,&y,&z);
        printf("%s object %s x %f y %f z %f\n",action,object,x,y,z);

		    //(answer ?sender command find_object ?block1 ?x ?y ?z ?arm 1)
        //sprintf(str,"(assert (answer %s command %s %s %f %f %f %s 1))",ROS_System,action,object,x,y,z,arm);
        //printf("\nSend fact %s\n",str);
        

        ros::NodeHandle n;
        ros::ServiceClient client;
        simulator::simulator_find_obj srv;
        client = n.serviceClient<simulator::simulator_find_obj>("simulator_find_obj"); //create the client

        srv.request.ROS_System = ROS_System;
        srv.request.action = action;
        srv.request.object = object;
        srv.request.x = x;
        srv.request.y = y;
        srv.request.z = z;
        srv.request.arm = arm;

        if (client.call(srv))
        {
          strcpy(str,srv.response.answer.c_str());
        }
        else
        {
          ROS_ERROR("Failed to call service simulator_find_obj");
        }
        SimuladorRepresentation::strQueryKDB(str, result, 10000);
        printf("\nCLIPS answer: %d %s\n",i,result.c_str());

       }

      

       if(strcmp(action,"mv")==0){

	        //  ACT-PLN mv 0.505 0.45 30000 4
                sscanf(result.c_str(),"%s %s %f %f",ROS_System,action,&x,&y);

                printf("%s %f %f\n",action,x,y);
		            get_distance_theta(x,y,theta,px,py,&distance,&angle);
                printf("mv angle %f distance %f\n",angle,distance);

                movements->twist = angle;
                movements->advance = distance;

		//(answer ?sender command mv ?distance ?angle ?flg)
                sprintf(str,"(assert (answer %s command %s %f %f 1))",ROS_System,action,distance,angle);
                printf("\nSend fact %s\n",str);
                SimuladorRepresentation::strQueryKDB(str, result, 10000);
                printf("\nCLIPS answer: %d %s\n",i,result.c_str());


       }



  if(strcmp(action,"grab")==0 || strcmp(action,"drop")==0)
  {
    sscanf(result.c_str(),"%s %s %s",ROS_System,action,object);
    printf("%s object %s\n",action,object);

    ros::NodeHandle n;
    ros::ServiceClient client;
    simulator::simulator_manipulator srv;
    client = n.serviceClient<simulator::simulator_manipulator>("simulator_manipulator"); //create the client

    srv.request.ROS_System = ROS_System;
    srv.request.action = action;
    srv.request.object = object;

    if (client.call(srv))
    {
      strcpy(str,srv.response.answer.c_str());
    }
    else
    {
      ROS_ERROR("Failed to call service simulator_manipulator");
    }

    printf("\nSend fact %s\n",str);
    SimuladorRepresentation::strQueryKDB(str, result, 10000);
    printf("\nCLIPS answer: %d %s\n",i,result.c_str());
  }



    if(strcmp(action,"go")==0){

    // ACT-PLN go any 0.505 0.45 0.0 30000 4
		//ACT-PLN go storage 0.506491 0.875 1.541639 30000 4 
       		sscanf(result.c_str(),"%s %s %s %f %f %f",ROS_System,action,room,&x,&y,&z);
                printf("%s %s %f %f %f\n",action,room,x,y,z);

		if(strcmp(room,"any")==0){
			j++;
			//if(j==3)j=1;
			movements->twist = - 0.3*j;
                	movements->advance = 0.04*j;
                	printf("go j %d any angle %f distance %f\n",j,angle,distance);
		}
		else{
			k++;
			if(k==3)k=1;
			get_distance_theta(x,y,theta,px,py,&distance,&angle);
			movements->twist = angle;
			movements->advance = distance - 0.045*k;
                	printf("go k %d angle %f distance %f\n",k,angle,distance);
		}



		//(answer ?sender command go ?x ?y ?z ?flg)
                sprintf(str,"(assert (answer %s command %s %f %f %f 1))",ROS_System,action,x,y,z);
                printf("\nSend fact %s\n",str);
                SimuladorRepresentation::strQueryKDB(str, result, 10000);
                printf("\nCLIPS answer: %d %s\n",i,result.c_str());


      }

printf("movements->twist %f movements->advance %f \n",movements->twist,movements->advance);

}


