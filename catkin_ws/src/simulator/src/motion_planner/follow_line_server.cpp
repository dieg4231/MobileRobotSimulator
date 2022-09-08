#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<simulator/line_follower.h>
#include<sensor_msgs/LaserScan.h>
#include<std_msgs/Int16MultiArray.h>

using namespace std;

ros::Publisher pubCmdVel;

int line_sensor[2];
bool finished = false;
bool flagOnce = false;

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

void follow_line(){
    cout << "{ " << line_sensor[0] << ", " << line_sensor[1] << ", " << line_sensor[2] << " }" << endl;
    if(line_sensor[0] == 1 && line_sensor[1] == 1 && !flagOnce)
        pubCmdVel.publish(move_robot(FOWARD));
    else if(line_sensor[0] == 0 && line_sensor[1] == 0) {
        pubCmdVel.publish(move_robot(FOWARD));
        flagOnce = true;
    }
    else if(line_sensor[0] == 1 && line_sensor[1] == 0) {
        pubCmdVel.publish(move_robot(TURN_RIGHT));
        flagOnce = true;
    }
    else if(line_sensor[0] == 0 && line_sensor[1] == 1) {
        pubCmdVel.publish(move_robot(TURN_LEFT));
        flagOnce = true;
    }
    else {
        pubCmdVel.publish(move_robot(STOP));
        finished = true;
    }
    
}

bool startFollow(simulator::line_follower::Request &req, simulator::line_follower::Response &res){
    ros::Rate rate(50);

    /*while(ros::ok()){
        follow_line();
        if(finished) break;
        
        ros::spinOnce();
        rate.sleep();
    }//*/

    res.response = "Ok";

    return true;
}

void lineSensorsCallback(const std_msgs::Int16MultiArray::ConstPtr& msg){
    line_sensor[0] = msg->data[0];
    line_sensor[1] = msg->data[1];
}


int main(int argc, char **argv){
    cout << "Starting follow_line_server by Luis Nava" << endl;
    ros::init(argc, argv, "follow_line_server");
    ros::NodeHandle nh;
    ros::Rate loop(20);

    pubCmdVel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::ServiceServer service = nh.advertiseService("follow_line", startFollow);
    ros::Subscriber subLineSensor = nh.subscribe("/line_sensors", 1000, lineSensorsCallback);
    
   
    pubCmdVel.publish(move_robot(STOP));
    
    while(ros::ok()){
	//follow_line();
	ros::spinOnce();
    }

    return 0;
}