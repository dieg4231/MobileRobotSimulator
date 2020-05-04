/********************************************************
 *                                                      *
 *                                                      *
 *      state_machine_avoidance_destination.h          	*
 *                                                      *
 *		Jesus Savage				*
 *		Diego Cordero				*
 *		FI-UNAM					*
 *		17-2-2019                                *
 *                                                      *
 ********************************************************/



#define THRESHOLD 20

// State Machine 
int sm_avoidance_destination(float intensity, int  dest,int obs ,movement *movements  ,int *next_state ,float Mag_Advance ,float max_twist)
{

 int state = *next_state;
 int result=0;


 printf("Present State: %d \n", state);
 printf("intensity %f obstacles %d dest %d\n",intensity,obs,dest);

 switch ( state ) {

        case 0:

		if (intensity > THRESHOLD){
                        *movements=generate_output(STOP,Mag_Advance,max_twist);
                        //printf("Present State: %d STOP\n", state);
                        printf("\n **************** Reached light source ******************************\n");
                        *next_state = 0;
			result = 1;
                }
                else{
                        *movements=generate_output(FORWARD,Mag_Advance,max_twist);
                        //printf("Present State: %d forward\n", state);
                        *next_state = 1;
                }

                break;


        case 1:
                if (obs == 0){
			// there is not obstacle
                        *movements=generate_output(FORWARD,Mag_Advance,max_twist);
                        //printf("Present State: %d FORWARD\n", state);
                        *next_state = 13;
                }
                else{
                        *movements=generate_output(STOP,Mag_Advance,max_twist);
                        //printf("Present State: %d STOP\n", state);

                        if (obs == 1){
                                // obtacle on the right
                                *next_state = 4;
                        }
                        else if (obs == 2){
                                // obtacle on the left
                                *next_state = 2;
                        }
                        else if (obs == 3){
				// obstacle on the front
                                *next_state = 6;
                        }
                }

                break;

        case 2: // Backward, obstacle in the left
                *movements=generate_output(BACKWARD,Mag_Advance,max_twist);
		//printf("Present State: %d BACKWARD, obstacle LEFT\n", state);
                *next_state = 3;
                break;

        case 3: // right turn
                *movements=generate_output(RIGHT,Mag_Advance,max_twist);
		//printf("Present State: %d TURN RIGHT\n", state);
                *next_state = 0;
                break;

        case 4: // Backward, obstacle in the right
                *movements=generate_output(BACKWARD,Mag_Advance,max_twist);
		//printf("Present State: %d BACKWARD, obstacle RIGHT\n", state);
                *next_state = 5;
                break;

        case 5: // left turn
                *movements=generate_output(LEFT,Mag_Advance,max_twist);
		//printf("Present State: %d TURN LEFT\n", state);
                *next_state = 0;
                break;

        case 6: // Backward, obstacle in front
                *movements=generate_output(BACKWARD,Mag_Advance,max_twist);
		//printf("Present State: %d BACKWARD, obstacle FRONT\n", state);
                *next_state = 7;
                break;

	case 7: /// Left turn
                *movements=generate_output(LEFT,Mag_Advance,max_twist);
		//printf("Present State: %d TURN 1 LEFT\n", state);
                *next_state = 8;
                break;

        case 8:// Left turn
                *movements=generate_output(LEFT,Mag_Advance,max_twist);
		//printf("Present State: %d TURN 2 LEFT\n", state);
                *next_state = 9;
                break;

        case 9: // Forward
                *movements=generate_output(FORWARD,Mag_Advance,max_twist);
                //printf("Present State: %d 1 FORWARD\n", state);
                *next_state = 10;
                break;

        case 10: // Forward
                *movements=generate_output(FORWARD,Mag_Advance,max_twist);
                //printf("Present State: %d 2 FORWARD\n", state);
                *next_state = 11;
                break;

	case 11: // Right turn
                *movements=generate_output(RIGHT,Mag_Advance,max_twist);
                //printf("Present State: %d turn 1 RIGHT\n", state);
                *next_state = 12;
                break;

        case 12: // Right turn
                *movements=generate_output(RIGHT,Mag_Advance,max_twist);
                //printf("Present State: %d turn 2 RIGHT\n", state);
                *next_state = 0;
                break;

        case 13: // // check destination
		if (dest == 0){
                                // go right
                                *movements=generate_output(RIGHT,Mag_Advance,max_twist);
                                //printf("Present State: %d RIGHT \n", state);
                                *next_state = 3;
                 }
                 else if (dest == 1){
                                // go left
                                *movements=generate_output(LEFT,Mag_Advance,max_twist);
                                //printf("Present State: %d LEFT\n", state);
                                *next_state = 5;
                 }
                 else if (dest == 2){
                                // go forward 
                                *movements=generate_output(FORWARD,Mag_Advance,max_twist);
                                //printf("Present State: %d FORWARD\n", state);
                                *next_state = 3;
                 }
                 else if (dest == 3){
                                // go forward 
                                *movements=generate_output(FORWARD,Mag_Advance,max_twist);
                                //printf("Present State: %d FORWARD\n", state);
                                *next_state = 5;
                 }
                break;

	default:
		//printf("State %d not defined used ", state);
                *movements=generate_output(STOP,Mag_Advance,max_twist);
                *next_state = 0;
                break;

                
 }

 printf("Next State: %d \n", *next_state);
 return result;

}



                 
