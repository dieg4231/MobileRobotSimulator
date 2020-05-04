/********************************************************
 *                                                      *
 *                                                      *
 *      user_sm.h			          	*
 *                                                      *
 *							*
 *		FI-UNAM					*
 *		17-2-2019                               *
 *                                                      *
 ********************************************************/



// State Machine 
void user_sm(float intensity, float *light_values, float *observations, int size, float laser_value, int  dest, int obs ,
					movement *movements  ,int *next_state ,float Mag_Advance ,float max_twist)
{

 int state = *next_state;
 int i;

 printf("intensity %f\n",intensity);
 printf("quantized destination %d\n",dest);
 printf("quantized obs %d\n",obs);

 for(int i = 0; i < 8; i++)
        printf("light_values[%d] %f\n",i,light_values[i]);
 for (int i = 0; i < size ; i++ ) 
         printf("laser observations[%d] %f\n",i,observations[i]);


 switch ( state ) {

        case 0:

		*movements=generate_output(FORWARD,Mag_Advance,max_twist);
                *next_state = 1;

               	break;

        case 1:
               *movements=generate_output(LEFT,Mag_Advance,max_twist);
               *next_state = 0;
                break;


	default:
		//printf("State %d not defined used ", state);
                *movements=generate_output(STOP,Mag_Advance,max_twist);
                *next_state = 0;
                break;

                
 }

 printf("Next State: %d\n", *next_state);


}



                 
