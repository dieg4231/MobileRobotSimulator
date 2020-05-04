/***********************************************
*                                              *
*      light_follower.h	                       *
*                                              *
*      Diego Cordero                           *
*      Jesus Savage			       *
*					       *
*                                              *
*              Bio-Robotics Laboratory         *
*              UNAM, 2019                      *
*                                              *
*                                              *
************************************************/


#define THRESHOLD_FOLLOWER 30
 

int light_follower(float intensity,float *light_values,movement *movements,float max_advance, float max_turn_angle)
{

 int sensor = 0;
 int i;
 int result = 0;


 if(intensity > THRESHOLD_FOLLOWER)
 {

	movements->twist = 0.0;
 	movements->advance = 0.0;
	result = 1;
	printf("\n **************** Reached light source ******************************\n");
 }
 else
 {
 	for(i = 1; i < 8; i++) 
 	{
	    if( light_values[i] > light_values[sensor])
		sensor = i;
 	}
 	
 	if(sensor > 4)
	   sensor = -(8 - sensor);	


	movements->twist = sensor * 3.1315 / 16;
 	movements->advance = max_advance/2;
 }

 return result;

}
