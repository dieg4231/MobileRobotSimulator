#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <simulator/simulator_MoveRealRobot.h>

ros::Publisher pubCmdVel;
tf::StampedTransform transform;
tf::Quaternion q;
tf::TransformListener * transformListener;

enum State{
	SM_INIT, 
	SM_GOAL_POSE_ACCEL, 
	SM_GOAL_POSE_CRUISE, 
	SM_GOAL_POSE_DECCEL, 
	SM_GOAL_POSE_CORRECT_ANGLE,
	SM_GOAL_POSE_FINISH 
};

geometry_msgs::Twist calculate_speeds(float robot_x, float robot_y, float robot_t, float goal_x, float goal_y,
		float cruise_speed, bool backwards){
	//Control constants
	float alpha = 0.6548;
	float beta = 0.2;
	float max_angular = 3.5;

	//Error calculation
	float angle_error = 0;
	if(backwards) angle_error = atan2(robot_y - goal_y, robot_x - goal_x) - robot_t;
	else angle_error = atan2(goal_y - robot_y, goal_x - robot_x) - robot_t;
	if(angle_error >   M_PI) angle_error -= 2*M_PI;
	if(angle_error <= -M_PI) angle_error += 2*M_PI;

	if(backwards) cruise_speed *= -1;
	geometry_msgs::Twist result;
	result.linear.x  = cruise_speed * exp(-(angle_error * angle_error) / alpha);
	result.linear.y  = 0;
	result.angular.z = max_angular * (2 / (1 + exp(-angle_error / beta)) - 1);
	return result;
}

geometry_msgs::Twist calculate_speeds(float robot_angle, float goal_angle){
	//Control constants
	float beta = 0.12;
	float max_angular = 1.4;

	float angle_error = goal_angle - robot_angle;
	if(angle_error >   M_PI) angle_error -= 2*M_PI;
	if(angle_error <= -M_PI) angle_error += 2*M_PI;

	geometry_msgs::Twist result;
	result.linear.x  = 0;
	result.linear.y  = 0;
	result.angular.z = max_angular * (2 / (1 + exp(-angle_error / beta)) - 1);
	return result;
}

void get_robot_position_wrt_odom(tf::TransformListener * transformListener, float& robot_x, float& robot_y, float& robot_t){
	tf::StampedTransform transform;
	transformListener->lookupTransform("odom", "base_link", ros::Time(0), transform);
	robot_x = transform.getOrigin().x();
	robot_y = transform.getOrigin().y();
	tf::Quaternion q = transform.getRotation();
	robot_t = atan2((float)q.z(), (float)q.w()) * 2;
	if(robot_t >   M_PI) robot_t -= 2*M_PI;
	if(robot_t <= -M_PI) robot_t += 2*M_PI;
}

void get_goal_position_wrt_odom(float goal_distance, float goal_angle, tf::TransformListener* transformListener,
		float& goal_x, float& goal_y, float& goal_t){
	tf::StampedTransform transform;
	transformListener->lookupTransform("odom", "base_link", ros::Time(0), transform);
	float robot_x = transform.getOrigin().x();
	float robot_y = transform.getOrigin().y();
	tf::Quaternion q = transform.getRotation();
	float robot_t = atan2((float)q.z(), (float)q.w()) * 2;

	goal_x = robot_x + goal_distance * cos(robot_t + goal_angle);
	goal_y = robot_y + goal_distance * sin(robot_t + goal_angle);
	goal_t = robot_t + goal_angle;
	if(goal_t >   M_PI) goal_t -= 2*M_PI;
	if(goal_t <= -M_PI) goal_t += 2*M_PI;
}

bool simpleMoveCallback(simulator::simulator_MoveRealRobot::Request &req, simulator::simulator_MoveRealRobot::Response &res){
	State state = SM_INIT;
	ros::Rate rate(30);

	float cruise_speed = 0;
	int attempts = 0;
	bool withDistance;

	float goal_x, goal_y, goal_t;
	float robot_x, robot_y, robot_t;

	bool success = false;

	float error;

	geometry_msgs::Twist twist;
	geometry_msgs::Twist zero_twist;
	zero_twist.linear.x  = 0;
	zero_twist.linear.y  = 0;
	zero_twist.angular.z = 0;

	while(ros::ok() && !success){

		switch(state){
			case SM_INIT:
				cruise_speed = 0.0;
				withDistance = false;
				get_goal_position_wrt_odom(req.distance, req.theta, transformListener, goal_x, goal_y, goal_t);
				state = State::SM_GOAL_POSE_ACCEL;
				attempts = (int)((fabs(req.distance)+0.1)/0.4*60 + fabs(req.theta)/0.5*60);
				break;

			case SM_GOAL_POSE_ACCEL:
				cruise_speed += 0.007;
				get_robot_position_wrt_odom(transformListener, robot_x, robot_y, robot_t);
				error = sqrt((goal_x - robot_x)*(goal_x - robot_x) + (goal_y - robot_y)*(goal_y - robot_y));
				std::cout << "error:" << error << std::endl;
				if(error < 0.035 && !withDistance)
					state = State::SM_GOAL_POSE_CORRECT_ANGLE;
				else{
					if(fabs(req.theta) >= 0.045 && !withDistance){
						state = State::SM_GOAL_POSE_CORRECT_ANGLE;
						withDistance = true;
					}else{
						if(error < cruise_speed)
							state = State::SM_GOAL_POSE_DECCEL;
						else if(cruise_speed >= 0.4)
							state = State::SM_GOAL_POSE_CRUISE;
						else
							state = State::SM_GOAL_POSE_ACCEL;
					}

					twist = calculate_speeds(robot_x, robot_y, robot_t, goal_x, goal_y, cruise_speed, req.distance < 0);
					pubCmdVel.publish(twist);
				}
				if(--attempts <= 0)
					state = State::SM_GOAL_POSE_FINISH;
				break;
			case SM_GOAL_POSE_CRUISE:
				get_robot_position_wrt_odom(transformListener, robot_x, robot_y, robot_t);
				error = sqrt((goal_x - robot_x)*(goal_x - robot_x) + (goal_y - robot_y)*(goal_y - robot_y));
				std::cout << "errorCruise:" << error << std::endl;
				//std::cout << "cruise_speed:" << cruise_speed << std::endl;
				if(error < cruise_speed)
					state = State::SM_GOAL_POSE_DECCEL;

				twist = calculate_speeds(robot_x, robot_y, robot_t, goal_x, goal_y, cruise_speed, req.distance < 0);
				pubCmdVel.publish(twist);
				if(--attempts <= 0)
					state = State::SM_GOAL_POSE_FINISH;
				break;

			case SM_GOAL_POSE_DECCEL:
				withDistance = false;
				cruise_speed -= 0.007;
				get_robot_position_wrt_odom(transformListener, robot_x, robot_y, robot_t);
				error = sqrt((goal_x - robot_x)*(goal_x - robot_x) + (goal_y - robot_y)*(goal_y - robot_y));
				std::cout << "errorDes:" << error << std::endl;
				if(error < 0.035 || cruise_speed <= 0)
					state = State::SM_GOAL_POSE_CORRECT_ANGLE;

				twist = calculate_speeds(robot_x, robot_y, robot_t, goal_x, goal_y, cruise_speed, req.distance < 0);
				pubCmdVel.publish(twist);
				if(--attempts <= 0)
					state = State::SM_GOAL_POSE_FINISH;
				break;

			case SM_GOAL_POSE_CORRECT_ANGLE:
				get_robot_position_wrt_odom(transformListener, robot_x, robot_y, robot_t);
				error = fabs(goal_t - robot_t);
				std::cout << "Angle error:" << error << std::endl;
				if(error < 0.045){
					if(!withDistance)
						state = State::SM_GOAL_POSE_FINISH;
					else{
						state = State::SM_GOAL_POSE_ACCEL;
					}

				}else{
					twist = calculate_speeds(robot_t, goal_t);
					pubCmdVel.publish(twist);
				}
				if(--attempts <= 0)
					state = State::SM_GOAL_POSE_FINISH;
				break;
			case SM_GOAL_POSE_FINISH:
				std::cout << "SimpleMove.->Successful move with dist=" << req.distance << " angle=" << req.theta << std::endl;
				success = true;
				pubCmdVel.publish(zero_twist);
				break;
		}

		ros::spinOnce();
		rate.sleep();
	}

	res.done = 1;
	return true;
}

int main(int argc, char ** argv){
	ros::init(argc, argv, "simple_move_node");
	ros::NodeHandle nh;
	ros::Rate rate(30);

	ros::ServiceServer simpleMoveService = nh.advertiseService("simulator_move_RealRobot", simpleMoveCallback);
	pubCmdVel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	transformListener = new tf::TransformListener();
	try{
		transformListener->waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(10.0));
		transformListener->lookupTransform("odom", "base_link", ros::Time(0), transform);
	}
	catch(...){
		std::cout << "SimpleMove.->Cannot get tranforms for robot's pose calculation... :'(" << std::endl;
		return -1;
	}

	while(ros::ok()){
		ros::spinOnce();
		rate.sleep();
	}
}
