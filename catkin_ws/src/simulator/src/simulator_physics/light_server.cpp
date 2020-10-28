#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <simulator/simulator_light.h>

using namespace std;

float light_sensor[8];

void light_callback(const std_msgs::Int16MultiArrayConstPtr& msg) {

    cout << "[" << msg->data[0] << ", " <<
                   msg->data[1] << ", " <<
                   msg->data[2] << ", " <<
                   msg->data[3] << ", " <<
                   msg->data[4] << ", " <<
                   msg->data[5] << ", " <<
                   msg->data[6] << ", " <<
                   msg->data[7] << ", " <<
                    "] " << endl;

    for(int i=0; i<8; i++) light_sensor[i] = ( msg->data[i]*40 ) / 1024;

}

bool get_intensities(simulator::simulator_light::Request &req, simulator::simulator_light::Response &res) {

    for(int i=0; i<8; i++) res.values[i] = light_sensor[i];

    return true;
}

int main(int argc, char **argv) {
    
    cout << "Starting light_server by Luis Näva..." << endl;
    ros::init(argc, argv, "light_server");
    ros::NodeHandle nh;

    ros::Subscriber sub_light_sensors = nh.subscribe("/light_sensors", 10, light_callback);
    ros::ServiceServer service = nh.advertiseService("simulator_light_RealRobot", get_intensities);

    ros::spin();

    return 0;
}
