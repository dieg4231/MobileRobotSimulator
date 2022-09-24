#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <simulator/simulator_MoveRealRobot.h>

ros::Publisher pubCmdVel;
tf::Quaternion q;
tf::StampedTransform transform;
tf::TransformListener* transformListener;


float a, b, c, x, v_top;

//PARAMETERS FOR ROBOT'S MOVEMENTS
float max_speed = 0.12;
float initial_speed = 0.04;
float angle_tolerancy = 0.02;
float distance_tolerancy = 0.004;

//PARAMETERS FOR LINEAR MOVEMENTS
float linear_alpha = 1.4;
float linear_beta = 12.5;
float linear_max_angular = 2.5;

//PARAMETERS FOR ANGULAR MOVEMENTS
float angular_alpha = 0.45;
float angular_max_angular = 2.5;

enum State{
    SM_INIT,
    SM_ROBOT_ACCEL,
    SM_ROBOT_DECCEL,
    SM_ROBOT_CORRECT_ANGLE,
    SM_ROBOT_FINISH
};

void load_parameters()
{
    if(!ros::param::get("/max_speed", max_speed)) {
        ROS_ERROR("/max_speed param did not loaded, check file: ~/MobileRobotSimulator/catkin_ws/src/simulator/src/data/control/control.yaml"); 
        std::cout << "Loading default value for max_speed->" << max_speed << std::endl;
    } else { std::cout << "\tmax_speed->" << max_speed << std::endl; }
    
    if(!ros::param::get("/initial_speed", initial_speed)){
        ROS_ERROR("/initial_speed param did not loaded, check file: ~/MobileRobotSimulator/catkin_ws/src/simulator/src/data/control/control.yaml"); 
        std::cout << "Loading default value for initial_speed->" << initial_speed << std::endl;
    } else { std::cout << "\tinitial_speed->" << initial_speed << std::endl; }
   
    if(!ros::param::get("/angle_tolerancy", angle_tolerancy)) {
        ROS_ERROR("/angle_tolerancy param did not loaded, check file: ~/MobileRobotSimulator/catkin_ws/src/simulator/src/data/control/control.yaml"); 
        std::cout << "Loading default value for angle_tolerancy->" << angle_tolerancy << std::endl;
    } else { std::cout << "\tangle_tolerancy->" << angle_tolerancy << std::endl; }
    
    if(!ros::param::get("/distance_tolerancy", distance_tolerancy)) {
        ROS_ERROR("/distance_tolerancy param did not loaded, check file: ~/MobileRobotSimulator/catkin_ws/src/simulator/src/data/control/control.yaml"); 
        std::cout << "Loading default value for distance_tolerancy->" << distance_tolerancy << std::endl;
    } else { std::cout << "\tdistance_tolerancy->" << distance_tolerancy << std::endl; }
  
    if(!ros::param::get("/linear_alpha", linear_alpha)) {
        ROS_ERROR("/linear_alpha param did not loaded, check file: ~/MobileRobotSimulator/catkin_ws/src/simulator/src/data/control/control.yaml"); 
        std::cout << "Loading default value for linear_alpha->" << linear_alpha << std::endl;
    } else { std::cout << "\tlinear_alpha->" << linear_alpha << std::endl; }
    
    if(!ros::param::get("/linear_beta", linear_beta)) {
        ROS_ERROR("/linear_beta param did not loaded, check file: ~/MobileRobotSimulator/catkin_ws/src/simulator/src/data/control/control.yaml"); 
        std::cout << "Loading default value for linear_beta->" << linear_beta << std::endl;
    } else { std::cout << "\tlinear_beta->" << linear_beta << std::endl; }

    if(!ros::param::get("/linear_max_angular", linear_max_angular)) {
        ROS_ERROR("/linear_max_angular param did not loaded, check file: ~/MobileRobotSimulator/catkin_ws/src/simulator/src/data/control/control.yaml"); 
        std::cout << "Loading default value for /linear_max_angular->" << linear_max_angular << std::endl;
    } else { std::cout << "\tlinear_max_angular->" <<linear_max_angular << std::endl; }
   
    if(!ros::param::get("/angular_alpha", angular_alpha)) {
        ROS_ERROR("/angular_alpha param did not loaded, check file: ~/MobileRobotSimulator/catkin_ws/src/simulator/src/data/control/control.yaml"); 
        std::cout << "Loading default value for angular_alpha->" << angular_alpha << std::endl;
    } else { std::cout << "\tangular_alpha->" << angular_alpha << std::endl; }
    
    if(!ros::param::get("/angular_max_angular", angular_max_angular)) {
        ROS_ERROR("/angular_max_angular param did not loaded, check file: ~/MobileRobotSimulator/catkin_ws/src/simulator/src/data/control/control.yaml"); 
        std::cout << "Loading default value for /angular_max_angular->" << angular_max_angular << std::endl;
    } else { std::cout << "\tangular_max_angular->" <<angular_max_angular << std::endl; }
}

geometry_msgs::Twist compute_speed(float robot_x, float robot_y, float robot_t, float goal_x, float goal_y,float robot_advance, bool backwards)
{
    float speed = 3*a*pow(robot_advance, 2) + 2*b*robot_advance + c;
    speed = max_speed * speed / v_top;
    if(speed < 0) speed = 0;

    //PARAMETERS FOR LINEAR MOVEMENTS
	float alpha = linear_alpha;
	float beta  =  linear_beta;
	float max_angular = linear_max_angular;

    //Error calculation
	float angle_error = 0;
	if(backwards) angle_error = atan2(robot_y - goal_y, robot_x - goal_x) - robot_t;
	else angle_error = atan2(goal_y - robot_y, goal_x - robot_x) - robot_t;
	if(angle_error >   M_PI) angle_error -= 2*M_PI;
	if(angle_error <= -M_PI) angle_error += 2*M_PI;
    
    if(backwards) speed *= -1;
    
    geometry_msgs::Twist _speed;
    _speed.linear.x = speed * exp(-(angle_error * angle_error) / alpha);
    _speed.linear.y = 0;
    _speed.angular.z = max_angular * (2 / (1 + exp(-angle_error / beta)) - 1);
    return _speed; 

}

geometry_msgs::Twist compute_speed(float goal_angle, float robot_angle)
{
	float angle_error = goal_angle - robot_angle;
    if(angle_error >   M_PI) angle_error -= 2*M_PI;
    if(angle_error <= -M_PI) angle_error += 2*M_PI;
    
    //PARAMETER FOR ANGULAR MOVEMENTS
    float alpha = angular_alpha;
    float max_angular = angular_max_angular;

    geometry_msgs::Twist speed;
	speed.linear.x  = 0;
	speed.linear.y  = 0;
    speed.angular.z = max_angular * (2 / (1 + exp(-angle_error / alpha)) - 1);
	return speed;
}

void get_robot_position(tf::TransformListener* transformListener, float& robot_x, float& robot_y, float& robot_t)
{
    double roll, pitch, yaw;
	tf::StampedTransform transform;
	transformListener->lookupTransform("odom", "base_link", ros::Time(0), transform);
	robot_x = transform.getOrigin().x();
	robot_y = transform.getOrigin().y();
    transform.getBasis().getRPY(roll, pitch, yaw);
    robot_t = yaw;

	if(robot_t >   M_PI) robot_t -= 2*M_PI;
	if(robot_t <= -M_PI) robot_t += 2*M_PI;
}
void get_goal_position(tf::TransformListener* transformListener, float& robot_x, float& robot_y, float& robot_t, 
                        float goal_distance, float goal_angle, float& goal_x, float& goal_y, float& goal_t)
{
    get_robot_position(transformListener, robot_x, robot_y, robot_t);

    goal_x = robot_x + goal_distance * cos(robot_t + goal_angle);
	goal_y = robot_y + goal_distance * sin(robot_t + goal_angle);
	goal_t = robot_t + goal_angle;
	if(goal_t >   M_PI) goal_t -= 2*M_PI;
	if(goal_t <= -M_PI) goal_t += 2*M_PI;
}

bool simpleMoveCallback(simulator::simulator_MoveRealRobot::Request &req, simulator::simulator_MoveRealRobot::Response &res)
{
    State state = SM_INIT;
    ros::Rate rate(50); 
    
    float goal_x, goal_y, goal_t;
    float start_robot_x, start_robot_y;
    float goal_distance, distance_error, angle_error, last_error;
    float robot_x, robot_y, robot_t, robot_advance, robot_traveled;

    bool goal_reached = false;
    bool with_distance = false;
    int attempts = 0;
    
    geometry_msgs::Twist twist, stop_robot;
	stop_robot.linear.x  = 0;
	stop_robot.linear.y  = 0;
	stop_robot.angular.z = 0;
    
    while(ros::ok() && !goal_reached)
    {
        switch(state)
        {
            case SM_INIT:
                //std::cout<<"ON INIT"<<std::endl;
                get_goal_position(transformListener, robot_x, robot_y, robot_t, req.distance, req.theta, goal_x, goal_y, goal_t);
                start_robot_x = robot_x;
                start_robot_y = robot_y;
                goal_distance = fabs(req.distance);
                last_error = goal_distance + distance_tolerancy;
                
                x = goal_distance;
                if(goal_distance > distance_tolerancy){
                    float xp_0 = 6*initial_speed / (max_speed +3*initial_speed);
                    a = (xp_0 - 2)/pow(x, 2);
                    b = (3 - 2*xp_0) / (x); 
                    c = xp_0;
                    float x_inf = - b /(3*a);
                    v_top = 3*a*pow(x_inf, 2) + 2*b*x_inf + c;
                }
                attempts = (int)((fabs(req.distance)+0.1)/0.5*400 + fabs(req.theta)/0.5*60);
                std::cout<<std::endl;
                std::cout << "goal_distance.->" << goal_distance << std::endl;
                std::cout<<"-------------------------------"<<std::endl;
                //std::cout<<std::setprecision(3)<<"Start position: x-> "<<robot_x<<"\ty-> "<<robot_y<<"\t\033[1;33mt->\033[0m"<<robot_t<<std::endl;
                state = State::SM_ROBOT_ACCEL;
            break;

            case SM_ROBOT_ACCEL:
                //std::cout << "SM_ROBOT_ACCEL" << std::endl;
                get_robot_position(transformListener, robot_x, robot_y, robot_t);
                distance_error =  sqrt(pow(goal_x - robot_x, 2) + pow(goal_y - robot_y, 2));
                robot_traveled = sqrt(pow(robot_x - start_robot_x, 2) + pow(robot_y - start_robot_y, 2));

                angle_error = fabs(goal_t - robot_t);
                if(distance_error < distance_tolerancy && !with_distance)
                    state = State::SM_ROBOT_CORRECT_ANGLE;
                else
                {
                    if(fabs(req.theta) >= angle_tolerancy && !with_distance)
                    {
                        state = State::SM_ROBOT_CORRECT_ANGLE;
                        with_distance = true;
                    }
                    else
                    {
                        if(distance_error > distance_tolerancy)
                        {
                            with_distance = true;
                            
                            robot_advance = goal_distance - distance_error;


                            if(last_error < distance_error && distance_error < goal_distance / 2) {
                                //std::cout <<"\t\033[1;33mUps! I've exceeded the goal...\033[0m"<< std::endl;
                                state = State::SM_ROBOT_FINISH;
                            }
                            //std::cout<< "\trobot_traveled.->"<<robot_traveled<<"\trobot_advanced.->"<<robot_advance<<"\tdistance_error.->"<<distance_error<<std::endl;
                            twist = compute_speed(robot_x, robot_y, robot_t, goal_x, goal_y, robot_advance, req.distance < 0);
                            pubCmdVel.publish(twist);
                            last_error = distance_error;
                        }
                        else
                            state = State::SM_ROBOT_FINISH;
                    }
                }
				if(--attempts <= 0)
                {
					state = State::SM_ROBOT_FINISH;
                    std::cout << "\t\033[1;33mUps! I've exceeded the attempts...\033[0m" << std::endl;
                }//*/
            break;

            case SM_ROBOT_CORRECT_ANGLE:
                //std::cout << "SM_ROBOT_CORRECT_ANGLE" << std::endl;
                get_robot_position(transformListener, robot_x, robot_y, robot_t);
                angle_error = goal_t - robot_t;
                if(angle_error >   M_PI) angle_error -= 2*M_PI;
                if(angle_error <= -M_PI) angle_error += 2*M_PI;

                if(fabs(angle_error) < angle_tolerancy) 
                {
                    if(!with_distance) state = State::SM_ROBOT_FINISH;
                    else state = State::SM_ROBOT_ACCEL;
                }
                else {
                    twist = compute_speed(goal_t, robot_t);
                    pubCmdVel.publish(twist);    //_speed.angular.z = 0;

                }
            break;

            case SM_ROBOT_FINISH:
                //std::cout << "SM_ROBOT_FINISH" << std::endl;
                std::cout << "SimpleMove.->Successful moved with dist=" << req.distance << " angle=" << req.theta << std::endl;
                goal_reached = true;
                pubCmdVel.publish(stop_robot);
            break;

            default:
                std::cout<<"An unexpected error has occurred :O"<<std::endl;
            break;
        }//FROM switch(state)
        
        ros::spinOnce();
        rate.sleep();
    }//FROM while(ros::ok())
    pubCmdVel.publish(stop_robot);
    get_robot_position(transformListener, robot_x, robot_y, robot_t);
    std::cout<<"goal_t->"<<goal_t<<"\trobot_t->"<<robot_t<<"\tangle_error->"<<angle_error<<"\t\033[1;33mdistance_error\033[0m.->"<<distance_error<<std::endl;
    res.done = 1;
    return true;
}//FROM bool simpleMoveCallback()

int main(int argc, char ** argv){
    std::cout<<"Starting move_minibot_node..."<<std::endl;
	ros::init(argc, argv, "move_minibot_node");
	ros::NodeHandle nh;
	ros::Rate rate(30);

	load_parameters();
	ros::ServiceServer simpleMoveService = nh.advertiseService("simulator_move_RealRobot", simpleMoveCallback);
    pubCmdVel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	
    transformListener = new tf::TransformListener();
    try{
		transformListener->waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(10.0));
		transformListener->lookupTransform("odom", "base_link", ros::Time(0), transform);
    }
    catch(...){
        std::cout << "SimpleMove.->Cannot get transforms for robot's pose calculation... :'(" << std::endl;
		return -1;
    }
    
	while(ros::ok()){
    
		ros::spinOnce();
		rate.sleep();
	}
}
