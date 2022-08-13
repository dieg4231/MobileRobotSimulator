#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<simulator/return_home.h>
#include<sensor_msgs/LaserScan.h>
#include<std_msgs/Int16MultiArray.h>

using namespace std;

ros::Publisher pubCmdVel;

int line_sensor[2];
float obstacle_distance = 1, obstacle_threshold = 0.1300;

enum State { FOWARD, BACKWARD, TURN_RIGHT, TURN_LEFT, STOP };

geometry_msgs::Twist move_robot(int move){

    geometry_msgs::Twist speed;

    switch(move)
    {
        case STOP:
            speed.linear.x = 0.0;
            speed.linear.y = 0.0;
            speed.angular.z = 0.0;  
            break;
        case FOWARD:
            speed.linear.x = 0.08;
            speed.linear.y = 0.0;
            speed.angular.z = 0.0;  
            break;
        case BACKWARD:
            speed.linear.x = -0.08;
            speed.linear.y = 0.0;
            speed.angular.z = 0.0;  
            break;
        case TURN_LEFT:
            speed.linear.x = 0.0;
            speed.linear.y = 0.0;
            speed.angular.z = 1.2;  
            break;
        case TURN_RIGHT:
            speed.linear.x = 0.0;
            speed.linear.y = 0.0;
            speed.angular.z = -1.2;  
            break;
        default:
            cout << "An unexpected error has occurred :(" << endl;
    }

    return speed;
}

void followLine(bool *goal_point){
    //cout << "{ " << line_sensor[0] << ", " << line_sensor[1] << " }" << endl;
    if(line_sensor[0] == 0 && line_sensor[1] == 0)    
        pubCmdVel.publish(move_robot(FOWARD));
    else if(line_sensor[0] == 1 && line_sensor[1] == 0)    
        pubCmdVel.publish(move_robot(TURN_RIGHT));
    else if(line_sensor[0] == 0 && line_sensor[1] == 1)    
        pubCmdVel.publish(move_robot(TURN_LEFT));
    else{
        pubCmdVel.publish(move_robot(STOP));
        *goal_point = true;	
    }
    
    if(obstacle_distance < obstacle_threshold){
        pubCmdVel.publish(move_robot(BACKWARD));
	cout << "Obstacle" << endl;
    }
}

bool returnHome(simulator::return_home::Request &req, simulator::return_home::Response &res){
    ros::Rate rate(50);

    bool goal_point = false;

    while(ros::ok()){
        followLine(&goal_point);
        
	if(goal_point) break;

        ros::spinOnce();
        rate.sleep();
    }

    res.res = "OK";

    return true;
}

void lineSensorsCallback(const std_msgs::Int16MultiArray::ConstPtr& msg){
    line_sensor[0] = msg->data[0];
    line_sensor[1] = msg->data[1];
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    obstacle_distance = msg->ranges[1]; 
    //cout << msg->ranges[1] << endl;
}

int main(int argc, char **argv){
    cout << "Starting return_home_server by Luis Nava" << endl;
    ros::init(argc, argv, "retunr_home_server");
    ros::NodeHandle nh;
    ros::Rate loop(20);

    pubCmdVel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::ServiceServer service = nh.advertiseService("return_home", returnHome);
    ros::Subscriber subScan = nh.subscribe("/scan",10, scanCallback);
    ros::Subscriber subLineSensor = nh.subscribe("/line_sensors", 1000, lineSensorsCallback);
    
   
    pubCmdVel.publish(move_robot(STOP));
    
    ros::spin();
    return 0;
}
