#include "ros/ros.h"
#include "simulator/Parameters.h"
#include "simulator/Laser_values.h"
#include "sensor_msgs/LaserScan.h"
#include "simulator/simulator_set_light_position.h"
#include "simulator/simulator_stop.h"
#include "simulator/simulator_robot_step.h"
#include "simulator/simulator_parameters.h"
#include "simulator/simulator_robot_laser_values.h"
#include "simulator/simulator_base.h"
#include "simulator/simulator_laser.h"
#include "simulator/simulator_light.h"
#include "simulator/simulator_algorithm_result.h"
#include "simulator/simulator_turtlebot.h"
#include "simulator/simulator_object_interaction.h"
#include <string.h>

#define GRASP 1
#define RELEASE 0

#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4

next_position next;
parameters params;
int new_simulation = 1;
float lasers[512];

movement generate_output(int out ,float advance ,float twist)
{

    movement output;

    switch(out) {

    case 0: // Stop
        output.advance = 0.0f;
        output.twist = 0.0f;
        //printf("STOP\n");
        break;

    case 1: // Forward
        output.advance = advance;
        output.twist = 0.0f;
        //printf("FORWARD\n");
        break;

    case 2: // backward
        output.advance = -advance;
        output.twist = 0.0f;
        //printf("BACKWARD\n");
        break;

    case 3:// Turn left
        output.advance = 0.0f;
        output.twist = twist;
        //printf("LEFT\n");
        break;

    case 4: // Turn right
        output.advance = 0.0f;
        output.twist = -twist;
        printf("RIGHT %f\n",output.twist);
        break;

    default:
        printf("Output %d not defined used ", out);
        output.advance = 0.0f;
        output.twist = 0.0f;
        break;
    }

    return(output);

}


void parametersCallback(const simulator::Parameters::ConstPtr& paramss)
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
    params.steps               = paramss->steps;
    params.turtle              = paramss->turtle;

}


/// GUI interaction


int stop()
{
    ros::NodeHandle n;
    ros::ServiceClient client;
    simulator::simulator_stop srv;
    client = n.serviceClient<simulator::simulator_stop>("simulator_stop");
    srv.request.stop = true;

    if ( !client.call(srv) )
        ROS_ERROR("Failed to call service simulator_stop");

    return 1;
}

bool object_interaction(int action, char name[50])
{
    ros::NodeHandle n;
    ros::ServiceClient client;
    simulator::simulator_object_interaction srv;
    client = n.serviceClient<simulator::simulator_object_interaction>("simulator_object_interaction");
    std::string s;
    s=name;
    srv.request.name = s;
    srv.request.grasp = action;

    if( !client.call(srv) )
    {
        ROS_ERROR("Failed to call service simulator_object_interaction");
    }
    printf("%d\n",srv.response.done );
    return srv.response.done;
}


int set_light_position(float x, float y)
{
    ros::NodeHandle n;
    ros::ServiceClient client;
    simulator::simulator_set_light_position srv;
    client = n.serviceClient<simulator::simulator_set_light_position>("simulator_set_light_position");
    srv.request.light_x = x;
    srv.request.light_y = y;

    if( !client.call(srv) )
    {
        ROS_ERROR("Failed to call service simulator_set_light_position");
    }
    return 1;
}


int print_algorithm_graph (step *steps )
{
    ros::NodeHandle n;
    ros::ServiceClient client;
    simulator::simulator_algorithm_result srv;
    client = n.serviceClient<simulator::simulator_algorithm_result>("simulator_print_graph"); //create the client

    for(int i = 0; i < 200; i++)
        srv.request.nodes_algorithm[i] = steps[i].node;

    if( !client.call(srv) )
        ROS_ERROR("Failed to call service simulator_print_graph");

    return 1;
}

//// Light Functions

int get_light_values(float *intensity, float *values)
{
    int sensor;
    ros::NodeHandle n;
    ros::ServiceClient client;
    simulator::simulator_light srv;
    client = n.serviceClient<simulator::simulator_light>("simulator_light"); //create the client
    srv.request.req = 1;

    if ( client.call(srv) )
    {
        for(int i = 0; i < 8; i++)
            values[i] = srv.response.values[i];

        sensor = 0;

        for(int i = 1; i < 8; i++)
        {
            if( values[i] > values[sensor])
                sensor = i;
        }
        *intensity = values[sensor];
    }
    else
    {
        ROS_ERROR("Failed to call service  simulator_light");
    }
}

int get_light_values_turtle(float *intensity, float *values)
{
    int sensor;
    ros::NodeHandle n;
    ros::ServiceClient client;
    simulator::simulator_light srv;
    client = n.serviceClient<simulator::simulator_light>("simulator_light_turtle"); //create the client
    srv.request.req = 1;

    if ( client.call(srv) )
    {
        for(int i = 0; i < 8; i++)
            values[i] = srv.response.values[i];

        sensor = 0;

        for(int i = 1; i < 8; i++)
        {
            if( values[i] > values[sensor])
                sensor = i;
        }
        *intensity = values[sensor];
    }
    else
    {
        ROS_ERROR("Failed to call service  simulator_light_turtle");
    }
}

int quantize_light(float *light_values)
{
    int sensor = 0;

    for(int i = 1; i < 8; i+=2 )
    {
        if( light_values[i] > light_values[sensor] )
            sensor = i;
    }
    //printf("biggest value sensor %d %f\n",sensor,light_values[sensor]);
    if(sensor == 0)
        return 2;
    else if(sensor == 1)
        return 3;
    else if(sensor == 3)
        return 1;
    else if(sensor == 5)
        return 0;
    else if(sensor == 7)
        return 2;
    else
        return 0;
}


//////LASER Functions

int quantize_laser_noise(float *observations, int size, float laser_value  )
{
    /*
      It quantizes the inputs
    */
    int a,b,cta;
    int iz,de,salida;
    int j;

    iz = de = salida = 0;
    if( size % 2 != 0)
    {
        a = ( size - 1 ) / 2;
        b = a + 1;
    }
    else
    {
        a = b = size / 2;
    }

    cta = 0;
    for (int i = b; i < size ; i++ ) //izquierda
    {
        if( observations[i] < laser_value  )
            cta++;
        if( cta >=  size*.4  )
        {
            iz = 2;
            break;
        }
    }

    cta = 0;
    for (int i = 0; i < a ; i++ ) //derecha
    {
        if( observations[i] < laser_value  )
            cta++;
        if( cta >=  size*.4  )
        {
            de = 1;
            break;
        }
    }

    return iz + de ;
}

int quantize_laser(float *observations, int size, float laser_value  )
{
    /*
      It quantizes the inputs
    */
    int a,b;
    int iz,de,salida;
    int j;

    iz = de = salida = 0;
    if( size % 2 != 0)
    {
        a = ( size - 1 ) / 2;
        b = a + 1;
    }
    else
    {
        a = b = size / 2;
    }

    for (int i = b; i < size ; i++ ) //izquierda
    {
        if( observations[i] < laser_value  )
        {
            iz = 2;
            break;
        }
    }

    for (int i = 0; i < a ; i++ ) //derecha
    {
        if( observations[i] < laser_value  )
        {
            de = 1;
            break;
        }
    }

    return iz + de ;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    /*
       This functions returns the lidars value acoording to gui settings
    */
    double PI;
    double K1;
    double theta;
    double ranges;
    double inc_angle;
    double init_angle;
    double sensors[512];
    double complete_range;
    int index;
    int num_points;
    int range_laser;
    int num_sensors;

    num_points = 512;
    PI = 3.1415926535;
    range_laser = 360;//240;
    complete_range = range_laser * PI / 180;
    K1 = complete_range / num_points;
    num_sensors = params.laser_num_sensors;
    ranges = params.laser_range;
    init_angle = params.laser_origin;
    inc_angle = ranges / num_sensors;
    theta = init_angle;

    for(int j = 0, k = 1 ; j < num_sensors; j++, k++)
    {
        index = int ( ( theta * 256 ) / 1.5707 ) + 256;
        lasers[j] = float( msg->ranges[index] );
        theta = k * inc_angle + init_angle;
    }

}


int get_lidar_values(float *lasers, float robot_x ,float robot_y, float robot_theta, bool turtle)
{
    ros::NodeHandle n;
    ros::ServiceClient client;

    simulator::simulator_laser srv;
    client = n.serviceClient<simulator::simulator_laser>("simulator_laser_serv"); //create the client

    srv.request.robot_x = robot_x;
    srv.request.robot_y = robot_y;
    srv.request.robot_theta = robot_theta;

    if (client.call(srv))
    {
        for(int i = 0; i < 512; i++)
            lasers[i] = srv.response.sensors[i];
    }
    else
    {
        ROS_ERROR("Failed to call service simulator_robot_laser_values");
    }

    return 1;
}


/// BASE

int move_gui(float angle ,float distance ,next_position *next,float lidar_readings[512] )
{
    ros::NodeHandle n;
    ros::ServiceClient client;
    simulator::simulator_robot_step srv;
    client = n.serviceClient<simulator::simulator_robot_step>("simulator_robot_step"); //create the client

    srv.request.theta = angle;
    srv.request.distance = distance;

    for(int i = 0; i < 512; i++ )
        srv.request.sensors[i] = lidar_readings[i];

    if (client.call(srv))
    {
        next->robot_x = srv.response.robot_x;
        next->robot_y = srv.response.robot_y;
        next->robot_theta =srv.response.theta;
    }
    else
    {
        ROS_ERROR("Failed to call service simulator_robot_step");

    }

    return 1;
}


int move_turtle(float theta,float distance)
{
    ros::NodeHandle n;
    ros::ServiceClient client;
    simulator::simulator_turtlebot srv;
    client = n.serviceClient<simulator::simulator_turtlebot>("simulator_move_turtle");
    srv.request.theta = theta;
    srv.request.distance = distance;

    if (client.call(srv))
    {
        if(srv.response.done)
            printf("Turtlebot move done \n");
        else
            printf("Turtlebot move fail \n");
    }
    else
    {
        ROS_ERROR("Failed to call service simulator_move_turtle");

    }

    return 1;
}



void check_collision(float theta ,float distance ,int new_simulation,float *final_theta,float *final_distance )
{
    ros::NodeHandle n;
    ros::ServiceClient client;
    simulator::simulator_base srv;
    client = n.serviceClient<simulator::simulator_base>("simulator_base"); //create the client

    srv.request.x1 = next.robot_x;
    srv.request.y1 = next.robot_y;
    srv.request.orientation = next.robot_theta;
    srv.request.theta = theta;
    srv.request.distance = distance;
    srv.request.new_simulation =new_simulation;

    if (client.call(srv))
    {
        *final_distance = srv.response.distance;
        *final_theta = srv.response.theta;

        //printf("TTTTTdistance: %f   , req  %f \n",srv.response.distance ,distance );
    }
    else
    {
        *final_distance = 0;
        *final_theta = 0;
        ROS_ERROR("Failed to call service simulator_base");
    }

  
}


int move_robot(float theta,float advance,float lidar_readings[512] )
{
    float final_distance,final_theta;
    check_collision(theta ,advance ,new_simulation,&final_theta,&final_distance);

    move_gui(final_theta ,final_distance ,&next,lidar_readings);
    if(params.turtle)
        move_turtle(theta,advance);
    ros::spinOnce();
    return 1;
}