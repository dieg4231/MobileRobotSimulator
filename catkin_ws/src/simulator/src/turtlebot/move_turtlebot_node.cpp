#include <stdio.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include "simulator/simulator_turtlebot.h"
#include <math.h>

geometry_msgs::Pose2D current_pose;
ros::Publisher movement_pub;
int once;

void odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    // linear position
    current_pose.x = msg->pose.pose.position.x;
    current_pose.y = msg->pose.pose.position.y;
    
    // quaternion to RPY conversion
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_pose.theta = yaw;
    //printf("YAW: %f ,X %f  ,Y %f \n", yaw,current_pose.x,current_pose.y) ; 
    // angular position
    once =1;
}


bool move_turtle(simulator::simulator_turtlebot::Request &req, simulator::simulator_turtlebot::Response &res)
{
    double dist = req.distance;
    double angle = req.theta;//M_PI;//1.5707;
    double x,y,x_goal,y_goal,angle_goal;
    double q = M_PI/2;
    double quadrant;
    geometry_msgs::Twist move;

    ros::Rate rate(100);

    ROS_INFO("--NEW GOAL--");
    ROS_INFO("  Angular Move:");
    ros::Time start = ros::Time::now();
    while(!once)
    {
        ros::spinOnce();
        rate.sleep();
    };

    angle_goal = current_pose.theta + angle;
    angle_goal = fmod( angle_goal , M_PI*2 );
    if( angle_goal > M_PI )
    {
       angle_goal = -(M_PI - ( angle_goal - M_PI ) );
    }
    else if (angle_goal < -M_PI)
    {
       angle_goal = (M_PI + ( angle_goal + M_PI ) );
    }

    printf("        Theta Goal: %f Theta Current: %f  Error: %f \n",angle_goal,current_pose.theta,  fabs(fabs(angle_goal) - fabs(current_pose.theta) )  );

    while(ros::ok() && ( fabs( fabs(angle_goal) - fabs(current_pose.theta) )  > .05 )  )
    {
        //geometry_msgs::Twist move;
        move.linear.x = 0.0; //speed value m/s

        if( angle > 0 )
            move.angular.z = 1;
        else
            move.angular.z = -1;

        movement_pub.publish(move);
        ros::spinOnce();
        rate.sleep();
    }
    printf("        Theta Goal: %f Theta Current: %f  Error: %f \n",angle_goal,current_pose.theta,  fabs(fabs(angle_goal) - fabs(current_pose.theta) )  );

    
    move.angular.z = 0;
    movement_pub.publish(move);

    
    ros::spinOnce();
    rate.sleep();

    ROS_INFO("  Move Linear:");
    x = dist*cos(current_pose.theta);
    y = dist*sin(current_pose.theta);
    //printf("-----x %f  y %f \n",x,y );
    x_goal = current_pose.x  + x ;
    y_goal = current_pose.y  + y ; 


    double err1 =  sqrt(pow(x_goal - current_pose.x,2 )+pow(y_goal - current_pose.y,2 )) ;
    double err2 = err1;
    double err2_aux =1000;

    while(ros::ok() &&  err2 > .01 && err2 <= err2_aux+.002 ) //  (  fabs( fabs(x_goal) - fabs(current_pose.x) )  > .01   || fabs( fabs( y_goal) - fabs(current_pose.y))  > .01 ) ) 
    {
        printf("        XGoal: %f XCurrent: %f  Err: %f \n",x_goal, current_pose.x,sqrt(pow(x_goal - current_pose.x,2 )+pow(y_goal - current_pose.y,2 )));

        printf("        YGoal: %f YCurrent: %f  Err: %f \n",y_goal, current_pose.y,sqrt(pow(x_goal - current_pose.x,2 )+pow(y_goal - current_pose.y,2 )));

        //geometry_msgs::Twist move;
        //velocity controls
        if( dist>0 )
            move.linear.x = 0.2; //speed value m/s
        else
            move.linear.x = -0.2; //speed value m/s
        move.angular.z = 0;
        movement_pub.publish(move);

    
        ros::spinOnce();
        rate.sleep();

        err2_aux = err2;
        err2= sqrt(pow(x_goal - current_pose.x,2 )+pow(y_goal - current_pose.y,2 ));
    }
    printf("        XGoal: %f XCurrent: %f  Err: %f \n",x_goal, current_pose.x,sqrt(pow(x_goal - current_pose.x,2 )+pow(y_goal - current_pose.y,2 )));
    printf("        YGoal: %f YCurrent: %f  Err: %f \n",y_goal, current_pose.y,sqrt(pow(x_goal - current_pose.x,2 )+pow(y_goal - current_pose.y,2 )));

    printf("%f > .01 && %f <= %f\n", err2  ,err2,err2_aux );
    printf("ERR1: %f ERR2: %f \n",err1, sqrt(pow(x_goal - current_pose.x,2 )+pow(y_goal - current_pose.y,2 )) );

    //geometry_msgs::Twist move;
    move.linear.x = 0;
    move.angular.z = 0;
    movement_pub.publish(move);
    
    res.done =1;
    ros::spinOnce();
    rate.sleep();
  
}


int main(int argc, char **argv)
{
    once=0;
    ros::init(argc, argv, "move_turtle_node");
    ros::NodeHandle n;
    ros::Subscriber sub_odometry = n.subscribe("odom", 1, odomCallback);
    movement_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi",1); //for sensors the value after , should be higher to get a more accurate result (queued)
    ros::ServiceServer service = n.advertiseService("simulator_move_turtle", move_turtle);
    ROS_INFO("start move_turtle_node ");
    ros::spin();
    return 0;
}
