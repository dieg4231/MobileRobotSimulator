
/***********************************************
*                                              *
*      oracle.h	                               *
*                                              *
*      Jesus Savage                            *
*      Julio Cruz                              *
*                                              *
*              Bio-Robotics Laboratory         *
*              UNAM, 2019                      *
*                                              *
*                                              *
************************************************/



#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <ctime>
#include <map>
#include <sstream>
#include <string>


// It starts the communication with the CLIPS node
int start_clips(){

 bool init_kdb = false;
 std::string file;
 std::string result;

 std::cout << "Starting CLIPS" << std::endl;


 //This functions loads initial facts and rules from a file
 // The first parameter is a file that contains the names of CLIPS files *.clp
 // The second parameter indicates with false do not start executing the loaded files
 // The third parameter is a timeout
 file = "/src/expert_system/oracle.dat";
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


float oracle_clips(float intensity, int  dest,int obs ,movement *movements,float Max_Advance ,float Max_Twist){

 static int j=1;
 char str[300];
 static int init_flg=1;
 std::string result;
 char accion[20];
 float rotation,advance;
 int num;
 float status = 0.0;



 if(init_flg==1){

       // It starts the communication with the Clips node
       start_clips();

       // It loads the maximum rotation and advance of the robot
       strcpy(str,"(assert (max-advance ");
       sprintf(str,"%s %f max-rotation %f",str,Max_Advance,Max_Twist);
       strcat(str,"))");
       printf("Send fact %s\n",str);
       SimuladorRepresentation::strQueryKDB(str, result, 10000);
       printf("CLIPS answer: %d %s\n",j,result.c_str());
       rotation = 0.0;
       advance = 0.0;
       init_flg=0;
  }
  else{

       // It loads to the Clips node as a fact the quantized sensory data
       strcpy(str,"(assert (step ");
       sprintf(str,"%s %d intensity %f obs %d dest %d",str,j,intensity,obs,dest);
       strcat(str,"))");
       printf("Send fact %s\n",str);
       SimuladorRepresentation::strQueryKDB(str, result, 10000);
       printf("CLIPS answer: %d %s\n",j,result.c_str());
       j++;

      sscanf(result.c_str(),"%s%d%f%f%f",accion,&num,&rotation,&advance,&status);
      //printf("rotation %f advance %f\n",rotation,advance);
      
      if (status == 1.0){
                        printf("\n **************** Reached light source ******************************\n");
      }

  }
	
  movements->advance=advance;
  movements->twist=rotation;

  return status;

}


int ros_clips(int  dest,int obs ,movement *movements  ,int *next_state ,float Mag_Advance ,float max_twist){

        static int j=1;
        char str[300];
        char accion[20],rotation[20],advance[20];
        int num;
        float status;
	static int init_flg=1;


	if(init_flg==1){

       		// It starts the communication with the Clips node
       		start_clips();
		strcpy(str,"(assert (init))");
        	printf("Send fact %s\n",str);
        	SimuladorRepresentation::sendAndRunCLIPS(str);

       		init_flg=0;
  	}
  	else{

        	strcpy(str,"(assert (ros step ");
        	sprintf(str,"%s %d obs %d dest %d advance %f twist %f",str,j,obs,dest, Mag_Advance, max_twist);
        	strcat(str,"))");
        	printf("Send fact %s\n",str);
        	SimuladorRepresentation::sendAndRunCLIPS(str);
		//set busy clips flag and wait for a CLIPS response
       	 	SimuladorRepresentation::set_busy_clips(true);
	
        	j++;

       }


}


