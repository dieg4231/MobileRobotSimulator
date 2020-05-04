/********************************************************
 *                                                      *
 *                                                      *
 *      state_machine_avoidance.h	                *
 *                                                      *
 *              Jesus Savage                            *
 *              Diego Cordero                           *
 *              FI-UNAM                                 *
 *              5-2-2019                                *
 *                                                      *
 ********************************************************/



// Function to get next state
void sm_avoid_obstacles(int obs ,movement *movements  ,int *next_state ,float Mag_Advance ,float max_twist)
{


 int state = *next_state;

 //printf("Next state %d \n",state );

 switch ( state ) 
 {

        case 0:
                if (obs == 0){
                        // there is not obstacle
                        *movements=generate_output(FORWARD,Mag_Advance,max_twist);
                        //printf("Present State: %d FORWARD\n", state);
                        *next_state = 0;
                }
                else{
                        *movements=generate_output(STOP,Mag_Advance,max_twist);
                        //printf("Present State: %d STOP\n", state);

                        if (obs == 1){
                                // obtacle in the right
                                *next_state = 1;
                        }
                        else if (obs == 2){
                                // obstacle in the left
                                *next_state = 3;
                        }
                        else if (obs == 3){
                                // obstacle in the front
                                *next_state = 5;
                        }
                }

                break;

        case 1: // Backward, obstacle in the right
                *movements=generate_output(BACKWARD,Mag_Advance,max_twist);
                //printf("Present State: %d BACKWARD, obstacle RIGHT\n", state);
                *next_state = 2;
                break;

        case 2: // left turn
                *movements=generate_output(LEFT,Mag_Advance,max_twist);
                //printf("Present State: %d TURN LEFT\n", state);
                *next_state = 0;
                break;

        case 3: // Backward, obstacle in the left
                *movements=generate_output(BACKWARD,Mag_Advance,max_twist);
                //printf("Present State: %d BACKWARD, obstacle LEFT\n", state);
                *next_state = 4;
                break;

        case 4: // right turn
                *movements=generate_output(RIGHT,Mag_Advance,max_twist);
                //printf("Present State: %d TURN RIGHT\n", state);
                *next_state = 0;
                break;
        case 5: // Backward, obstacle in front

                *movements=generate_output(BACKWARD,Mag_Advance,max_twist);
                //printf("Present State: %d BACKWARD, obstacle FRONT\n", state);
                *next_state = 6;
                break;

        case 6: /// Left turn
                *movements=generate_output(LEFT,Mag_Advance,max_twist);
                //printf("Present State: %d TURN 1 LEFT\n", state);
                *next_state = 7;
                break;

        case 7:// Left turn
                *movements=generate_output(LEFT,Mag_Advance,max_twist);
                //printf("Present State: %d TURN 2 LEFT\n", state);
                *next_state = 8;
                break;

        case 8: // Forward
                *movements=generate_output(FORWARD,Mag_Advance,max_twist);
                //printf("Present State: %d 1 FORWARD\n", state);
                *next_state = 9;
                break;

        case 9: // Forward
                *movements=generate_output(FORWARD,Mag_Advance,max_twist);
                //printf("Present State: %d 2 FORWARD\n", state);
                *next_state = 10;
                break;

        case 10: // Right turn
                *movements=generate_output(RIGHT,Mag_Advance,max_twist);
                //printf("Present State: %d TURN 1 RIGHT\n", state);
                *next_state = 11;
                break;

        case 11: // Right turn
                *movements=generate_output(RIGHT,Mag_Advance,max_twist);
                //printf("Present State: %d TURN 2 RIGHT\n", state);
                *next_state = 0;
                break;
 }


}


