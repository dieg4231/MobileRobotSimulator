// ROS
#include <geometry_msgs/Point.h>
#include <ros/package.h>
#include "simulator/Parameters.h"
#include "simulator/simulator_parameters.h"
#include "simulator/PosesArray.h"
#include "simulator/poseCustom.h"
#include "../utilities/simulator_structures.h"
#include <ros/ros.h>

// For visualizing things in rviz
#include <functional>

//#include <rviz_visual_tools/rviz_visual_tools.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// C++
#include <string>
#include <vector>


visualization_msgs::Marker marker;
visualization_msgs::MarkerArray titulos;
geometry_msgs::Point point;



void paramsCallback(const simulator::PosesArray::ConstPtr& objs)
{
  titulos.markers.clear();
  marker.points.clear();
  visualization_msgs::Marker aux;
  for(int i = 0 ; i < objs->posesArray.size(); i++)
  {
    point.x = objs->posesArray[i].x ;
    point.y = objs->posesArray[i].y;
    point.z = .05;
    marker.points.push_back(point);

    


  
  
  visualization_msgs::Marker aux;
  aux.header.frame_id = "map";
  aux.header.stamp = ros::Time();
  aux.ns = "my_namespace";
  aux.id = i+100;
  aux.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  aux.action = visualization_msgs::Marker::ADD;
  aux.lifetime = ros::Duration(.1);
  aux.pose.position.x = objs->posesArray[i].x ;
  aux.pose.position.y = objs->posesArray[i].y;
  aux.pose.position.z = 0.10;
  aux.pose.orientation.x = 0.0;
  aux.pose.orientation.y = 0.0;
  aux.pose.orientation.z = 0.0;
  aux.pose.orientation.w = 1.0;
  aux.scale.x = .05;
  aux.scale.y = .05;
  aux.scale.z = .03;
  aux.color.a = 1.0; // Don't forget to set the alpha!
  aux.color.r = 0.0;
  aux.color.g = 0.0;
  aux.color.b = 0.0;
  aux.text= objs->posesArray[i].name;

  titulos.markers.push_back(aux);
  }


}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_viz_node");
  ros::NodeHandle n;
  ros::Subscriber params_sub = n.subscribe("objectsPose", 0, paramsCallback);


  ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "obj_marker", 0 );
  ros::Publisher vis_pub_titles = n.advertise<visualization_msgs::MarkerArray>( "obj_marker_titles", 0 );






  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 2;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(.1);
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = .05;
  marker.scale.y = .05;
  marker.scale.z = .05;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.1;
  marker.color.g = 0.5;
  marker.color.b = 1.0;

  std_msgs::ColorRGBA color;

  color.r = 0;
  color.g = 0;
  color.b = 50;
  color.a = 1;




  ros::Rate r(10.0);

  while(n.ok())
  {

   
    vis_pub.publish( marker );
    vis_pub_titles.publish(titulos);

    ros::spinOnce(); 
    r.sleep();

  }

  ROS_INFO_STREAM("Shutting down.");

  return 0;
}



