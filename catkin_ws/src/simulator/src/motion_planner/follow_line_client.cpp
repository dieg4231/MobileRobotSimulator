#include<ros/ros.h>
#include<simulator/line_follower.h>

using namespace std;

int main(int argc, char **argv){
    cout << "Starting follow_line_client by Luis Nava" << endl;
    ros::init(argc, argv, "follow_line_client");
    ros::NodeHandle nh;
    ros::Rate loop(20);

    ros::ServiceClient client = nh.serviceClient<simulator::line_follower>("follow_line");

    simulator::line_follower srv;
    

    if(client.call(srv)) {
        cout << "Ok" << endl;
    }
    else {
        ROS_ERROR("Failed to call service line_follow_server");
        return 1;
    }

    return 0;
}