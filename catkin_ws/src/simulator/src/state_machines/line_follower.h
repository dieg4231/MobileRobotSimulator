/********************************************************
 *                                                      *
 *                                                      *
 *      line_follower.h	                                *
 *                                                      * 
 *              Miguel Sanchez                          *
 *              FI-UNAM                                 *
 *              Agosto-2022                             *
 *                                                      *
 ********************************************************/
#include "ros/ros.h"
#include "dijkstralib.h"
#include<geometry_msgs/Twist.h>
#include <math.h>
#include<unistd.h>

#define THRESHOLD 35
float OBSTACLE_THRESHOLD=0.128;
const float dist_tolerancy = 0.1;
const float ang_tolerancy=0.2;
const float sleep_time = 5;
int Kprop = 0;
int node_dest=-1;
int obstacles=-1;
int lenght=0;
int frec=4;

ros::Publisher pubCmdVel;


int lines[3];
float rx=0;
float ry=0;
float rtheta=0;
bool battery_charging = false;



enum State { MOVE_FOWARD, MOVE_BACKWARD, TURN_RIGHT, TURN_LEFT, ROBOT_STOP, TURN_RIGHT_FOWARD,TURN_LEFT_FOWARD };

geometry_msgs::Twist move_robot(int move){

    float lin_vel = 0.09 ;
    float ang_vel = 1.0 ;
    geometry_msgs::Twist speed;

    switch(move)
    {
        case ROBOT_STOP:
            speed.linear.x = 0.0;
            speed.linear.y = 0.0;
            speed.angular.z = 0.0;  
            break;
        case MOVE_FOWARD:
            speed.linear.x = lin_vel;
            speed.linear.y = 0.0;
            speed.angular.z = 0.0;  
            break;
        case MOVE_BACKWARD:
            speed.linear.x = -lin_vel;
            speed.linear.y = 0.0;
            speed.angular.z = 0.0;  
            break;
        case TURN_LEFT:
            speed.linear.x = 0.0;
            speed.linear.y = 0.0;
            speed.angular.z = ang_vel;  
            break;
        case TURN_RIGHT:
            speed.linear.x = 0.0;
            speed.linear.y = 0.0;
            speed.angular.z = -ang_vel;  
            break;
        case TURN_RIGHT_FOWARD:
            //speed.linear.x = -(lin_vel*(1+(Kprop/1000000)));
            speed.linear.x = (lin_vel/15); 
            speed.linear.y = 0.0;
            //speed.angular.z= -ang_vel;
            speed.angular.z = -(ang_vel*(1-(Kprop/10000)));  
            //printf("lx: %f\n",-(lin_vel*(1+(Kprop/100000))));
            //printf("lx: %f\n",-(ang_vel*(1+(Kprop/100000))));
            break;
        case TURN_LEFT_FOWARD:
            speed.linear.x = -lin_vel;
            speed.linear.y = 0.0;
            speed.angular.z = ang_vel/2;  
            break;
        
        default:
            printf("Error\n");
    }

    return speed;
}

void ParamsCallback(const simulator::Parameters::ConstPtr& paramss)
{
    rx     = paramss->robot_x;
    ry     = paramss->robot_y;
    rtheta = paramss->robot_theta;
}

void lineSensCallback(const std_msgs::Int16MultiArray::ConstPtr& msg){
    lines[0] = msg->data[0];
    lines[1] = msg->data[1];
    lines[2] = msg->data[2];
}


int orient_to_node(int node_dest,ros::Rate lat)
{
    float ang=0;
    ang=atan2f(nodes[node_dest].y-ry,nodes[node_dest].x-rx);
    printf("node: %d\n",node_dest);
    printf("Orientation goal: %f\n",ang*(180/3.141593));
    printf("Robot orientation %f\n",rtheta*(180/3.141593));
    
    if( abs(rtheta-ang) <= ang_tolerancy)
    {
        pubCmdVel.publish(move_robot(ROBOT_STOP));
        lat.sleep();
        printf("\n\t\tRobot oriented\n\n");
        return 1;
    }
    else if( (ang-rtheta) < 0)
    {
        printf("TURN_RIGHT\n");
        pubCmdVel.publish(move_robot(TURN_RIGHT));
        lat.sleep();

    }
    else if( (ang-rtheta) > 0)
    {
        printf("TURN_LEFT\n");
        pubCmdVel.publish(move_robot(TURN_LEFT));
        lat.sleep();
    }
    return 0;

}

int avoid_obst(ros::Rate lat)
{
    
    obstacles=quantize_laser(lasers,lenght,OBSTACLE_THRESHOLD);
    
    //Mostrar mediciones laser 
    /*for(int i=0; i<3;i++)
    {
        printf("l[%d]: %f\n",i,lasers[i]);   
        
    }
    */
    if(lasers[1]<OBSTACLE_THRESHOLD)
        obstacles=3;
    if(!obstacles)
    {   
        //printf("no obstacle\n");    
    }
    else
    {
        pubCmdVel.publish(move_robot(ROBOT_STOP));
        lat.sleep();
        if(obstacles==1)
            printf("obstacle in right\n");    
            
        if(obstacles==2)
            printf("obstacle in left\n");
            
        printf("obstacle detected, sorting obstacle\n");
        
        for(int i=0;i<10;i++)
        {
            pubCmdVel.publish(move_robot(MOVE_BACKWARD));
            lat.sleep();
        }
              

        pubCmdVel.publish(move_robot(ROBOT_STOP));
        lat.sleep();
        /*while(!orient_to_node(node_dest,lat))
        {
            printf("orienting to node to continue path\n");
            ros::spinOnce();
            lat.sleep();
        }*/
    }
    ros::spinOnce();
}



void findline(ros::Rate lat)
{
    
    int rotation=-1;
    int sensor=-1;
    printf("RIGHT_FOWARD\n");
    int lineflag=0;
    do
    {
        //printf("l0 l1 l2\n");
        //printf("%d %d %d\n",lines[0],lines[1],lines[2]);
        pubCmdVel.publish(move_robot(TURN_RIGHT_FOWARD));
        lat.sleep();
        ros::spinOnce();
        avoid_obst(lat);
        
        //printf("l0 %d l1 %d  l2 %d \n",lines[0],lines[1],lines[2]);
        if(lines[0] || lines[1] || lines[2])
        {
            printf("lines[0]: %d lines[1]: %d Lines [2]:: %d \n",lines[0],lines[1],lines[2]);    
            pubCmdVel.publish(move_robot(ROBOT_STOP));
            lat.sleep();
            if(lines[0] && !lines[1])
            {
                rotation=2;//RIGHT
                sensor=0;
            }
            else if(!lines[0] && lines[1])
            {
                rotation=3;//LEFT
                sensor=1;
            }
            else if(lines[2])
            {
                rotation=4; //STOP
                sensor=2;
            }
                      
            
            /*ros::spinOnce();
            printf("lines[0]: %d lines[1]: %d Lines [2]:: %d \n",lines[0],lines[1],lines[2]);       
            if(lines[0] || lines[1] || lines[2])
                {   
                    lineflag=1;
                    //scanf("%d",&lineflag);
                }
                */
            lineflag=1;
        }
        Kprop++;
        
    }while(!lineflag);
    
    Kprop=0;
    
    printf("FOWARD\n");
    while(!lines[2])
    {
        pubCmdVel.publish(move_robot(MOVE_FOWARD));
        lat.sleep();
        
        ros::spinOnce();
        
        avoid_obst(lat);
    }
    pubCmdVel.publish(move_robot(ROBOT_STOP));
    lat.sleep();
    
    printf("TURN\n");
    printf("rotation: %d\n",rotation);
    printf("lines[%d]: %d\n",sensor,lines[sensor]);
    if(sensor != 2 && sensor !=-1)
    {
        while(lines[sensor])
        {
            pubCmdVel.publish(move_robot(rotation));
            //printf("move: %d\n",rotation);
            //printf("lines[%d]: %d\n",sensor,lines[sensor]);
            lat.sleep();
            ros::spinOnce();
        }
    }
    
    /*while(!orient_to_node(node_dest,lat))
        {
            printf("orienting to node find line\n");
            ros::spinOnce();
            lat.sleep();
        }
         
    */printf("\n \tline found\n");


}


int follow_line(ros::Rate lat){
    //printf("l0 %d l1 %d l2 %d \n",lines[0],lines[1],lines[2]);

    if(lines[2])
    {
        if (!lines[0] && !lines[1])
        {
            //*movements=generate_output(FORWARD,Mag_Advance,max_twist);
            pubCmdVel.publish(move_robot(MOVE_FOWARD));
            lat.sleep();
            //printf("FOWARD\t");
        }
        else if(lines[0] && !lines[1])
        {    
            //*movements=generate_output(RIGHT,Mag_Advance,max_twist);
            pubCmdVel.publish(move_robot(TURN_RIGHT));
            lat.sleep();
            //printf("RIGHT\t");
        }
        else if(!lines[0] && lines[1])
        {
            //*movements=generate_output(LEFT,Mag_Advance,max_twist);
            pubCmdVel.publish(move_robot(TURN_LEFT));
            lat.sleep();
            //printf("LEFT\t");
        }
        else if(lines[0] && lines[1])
        {
        pubCmdVel.publish(move_robot(ROBOT_STOP));
        lat.sleep();
        //printf("STOP\t");
        return 1;    
        }
    }
    
    else
    {
        findline(lat);
        /*while(!orient_to_node(node_dest,lat))
        {
            printf("orienting to node to find line\n");
            ros::spinOnce();
            avoid_obst(lat);
            lat.sleep();
        }
        while(!lines[2])
        {
            pubCmdVel.publish(move_robot(MOVE_FOWARD));
            printf("\tmove foward\n");
            ros::spinOnce();
            avoid_obst(lat);
            lat.sleep();
        }
        */

    }
    
    return 0;
}

float calculateDistance(float x1,float y1, float x2, float y2){
return sqrt(pow( x1 - x2 ,2) + pow( y1 - y2 ,2)); 
}



//Behavior 
int line_follower(
                float *observations
                ,int obs
                ,int size
                ,float x_dest
                ,float y_dest
                ,float Mag_Advance 
                ,float max_twist
                ,float robotx 
                ,float roboty
                ,char *world_name
                )
{
    ros::NodeHandle nh;
    ros::Rate lat(frec);
    pubCmdVel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber subLineSensor = nh.subscribe("/line_sensors", 1000, lineSensCallback);
    ros::Subscriber subPoseRobot = nh.subscribe("/simulator_parameters_pub",0,ParamsCallback);
    
    int result=0,current_node=-1;
    obstacles=obs;
    lenght=size;
    char archivo[150];
    int i;   
    int start = 0;
    std::string paths = ros::package::getPath("simulator");
    strcpy(archivo,paths.c_str());
    strcat(archivo,"/src/data/");
    strcat(archivo,world_name);
    strcat(archivo,"/");
    strcat(archivo,world_name);
    strcat(archivo,".top");
    for(i = 0; i < MUM_NODES; i++)
    {
        nodes[i].flag='N';
        nodes[i].num_conections = 0;
        nodes[i].parent = -1;
        nodes[i].acumulado = 0;
        nodes[i].x=0.0;
        nodes[i].y=0.0;
    }

    num_nodes=read_nodes(archivo); // Se lee el arcivo .top

    for(i=0; i<num_nodes; i++)
    {
        if (calculateDistance(x_dest,y_dest,nodes[i].x,nodes[i].y) < dist_tolerancy)
            {
                node_dest=nodes[i].num_node;
            }
    }

    printf("\nDestine Node: %d \n",node_dest);
    printf("Current Node: %d \n",current_node);
    printf("X_dest: %f \n",nodes[node_dest].x);
    printf("Y_dest: %f \n",nodes[node_dest].y);
    printf("Obstacle :%i\n",obs);
    printf("following line\n");
    while (current_node != node_dest) 
    {
        //printf("l0 %d l1 %d l2 %d\n",lines[0],lines[1],lines[02]);
        
        if(rx!=0 && ry!=0)
        {
            result=follow_line(lat);
            //printf("debugging obst avoider");
            //avoid_obst(lat);
            
        }
        //printf("RESULT %d\n",result);     
        if(result)
        {
            for(i=0;i< num_nodes;i++)
                {
                    if ((calculateDistance(rx,ry,nodes[i].x,nodes[i].y)) <= dist_tolerancy)
                    {
                        current_node=nodes[i].num_node;
                    }
                }
            printf("\nNode %d reached.\n\n",current_node);
            if (current_node !=node_dest)
            {
                //*movements=generate_output(FORWARD,Mag_Advance,max_twist);
                while(lines[0] && lines[1] )
                {
                    pubCmdVel.publish(move_robot(MOVE_FOWARD));
                    ros::spinOnce();
                    lat.sleep();
                }
                
                pubCmdVel.publish(move_robot(ROBOT_STOP));
                lat.sleep();

                while(orient_to_node(node_dest,lat)!=1)
                {
                    printf("orienting to next node\n");
                    ros::spinOnce();
                    lat.sleep();
                }
                

                //printf("MOVE FORWARD\n");
            }
                    
                                           
        }
        ros::spinOnce();
        avoid_obst(lat);
        lat.sleep();
    }

    pubCmdVel.publish(move_robot(ROBOT_STOP));
    lat.sleep();
    printf("Reached destine node\n\n");
    return result;
}




