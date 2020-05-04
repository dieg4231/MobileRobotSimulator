/********************************************************
 *                                                      *
 *                                                      *
 *      state_machine_destination.h           		*
 *                                                      *
 *              Jesus Savage                            *
 *              Diego Cordero                           *
 *              FI-UNAM                                 *
 *              13-2-2019                               *
 *                                                      *
 ********************************************************/

#define THRESHOLD_DEST 25.0


// State Machine 
int sm_destination(float intensity, int  dest,movement *movements  ,int *next_state ,float Mag_Advance ,float max_twist)
{

 int state = *next_state;
 int result=0;


 //printf("Present State: %d \n", state);
 //printf("intensity %f dest %d\n",intensity,dest);

 switch ( state ) {

        case 1:

                if (intensity > THRESHOLD_DEST){
                        *movements=generate_output(STOP,Mag_Advance,max_twist);
                        //printf("Present State: %d STOP\n", state);
                        printf("\n **************** Reached light source ******************************\n");
                        *next_state = 1;
                        result = 1;
                }
                else{
                        *movements=generate_output(FORWARD,Mag_Advance,max_twist);
                        //printf("Present State: %d FORWARD\n", state);
                        *next_state = 2;
                }

                break;


	case 2: // // check destination
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
                                *next_state = 4;
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
                                *next_state = 4;
                 }
                 else if (dest == 4){
                                // go forward 
                                *movements=generate_output(FORWARD,Mag_Advance,max_twist);
                                //printf("Present State: %d FORWARD\n", state);
                                *next_state = 1;
                 }
                break;


        case 3: // right turn
                *movements=generate_output(RIGHT,Mag_Advance,max_twist);
                //printf("Present State: %d TURN RIGHT\n", state);
                *next_state = 1;
                break;


        case 4: // left turn
                *movements=generate_output(LEFT,Mag_Advance,max_twist);
                //printf("Present State: %d TURN LEFT\n", state);
                *next_state = 1;
                break;

        default:
                //printf("State %d not defined used ", state);
                *movements=generate_output(STOP,Mag_Advance,max_twist);
                *next_state = 1;
                break;


 }

 return result;

}




