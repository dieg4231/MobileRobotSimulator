// ROS
#include <geometry_msgs/Point.h>
#include <ros/package.h>
#include "simulator/Parameters.h"
#include "simulator/simulator_parameters.h"
#include "../utilities/simulator_structures.h"
#include <ros/ros.h>

// For visualizing things in rviz
#include <functional>

//#include <rviz_visual_tools/rviz_visual_tools.h>
#include <visualization_msgs/Marker.h>
// C++
#include <string>
#include <vector>

//CGAL for triangulation
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/partition_2.h>
#include <cassert>
#include <list>


typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Partition_traits_2<K>                         Traits;
typedef Traits::Point_2                                     Point_2;
typedef std::vector<Point_2>                                  Point_list;
typedef Traits::Polygon_2                                   Polygon_2;
typedef std::list<Polygon_2>                                Polygon_list;

Point_2 aux,aux2;
Point_list point_list;


#define MAX_NUM_POLYGONS 100
#define NUM_MAX_VERTEX 10
#define STRSIZ 300
#define SIZE_LINE 10000



typedef struct Vertex_ {
        float x;
        float y;
} Vertex;

typedef struct Polygon_ {
        char    name[ STRSIZ ];
        char    type[ STRSIZ ];
        int     num_vertex;
        Vertex  vertex[NUM_MAX_VERTEX];
        Vertex  min,max;
} Polygon;

Polygon polygons_wrl[100];
int num_polygons_wrl = 0;
parameters params;
char actual_world[50];
float dimensions_room_x,dimensions_room_y;


// it reads the file that conteins the environment description
int ReadPolygons(char *file,Polygon *polygons){

  FILE *fp;
  char data[ STRSIZ ];
  int i;
  int num_poly = 0;
  int flg = 0;
  float tmp;
  

  fp = fopen(file,"r"); 
   
  if( fp == NULL )
  {
    sprintf(data, "File %s does not exist\n", file);
    printf("File %s does not exist\n", file);
    return(0);
  }
  //printf("World environment %s \n",file);

  while( fscanf(fp, "%s" ,data) != EOF)
  {
    if( strcmp(";(", data ) == 0 )
    {
      flg = 1;
      while(flg)
      {
        if(  0 < fscanf(fp, "%s", data));
        sscanf(data, "%f", &tmp);
        if(strcmp(")", data) == 0) flg = 0;
      }
    }
    else if((strcmp("polygon", data ) == 0) && ( flg == 0 ) )
    {
      if(  0 < fscanf(fp, "%s", data));
      strcpy(polygons[num_poly].type, data);
      if(  0 < fscanf(fp, "%s", data));
      strcpy(polygons[num_poly].name, data);
      i = 0;
      flg = 1;

      polygons[num_poly].max.x = 0;
      polygons[num_poly].max.y = 0;
      polygons[num_poly].min.x = 9999;
      polygons[num_poly].min.y = 9999;

      while(flg)
      {
        if(  0 < fscanf(fp, "%s", data));
        if(strcmp(")",data) == 0) 
        {
          polygons[num_poly].num_vertex = i - 1;
          polygons[num_poly].vertex[i].x = polygons[num_poly].vertex[0].x; // to calculate intersecction range
          polygons[num_poly].vertex[i].y = polygons[num_poly].vertex[0].y; // the first vertex its repeated on the last
          num_poly++;
          flg = 0;
        }
        else
        {
          sscanf(data, "%f", &tmp);
          polygons[num_poly].vertex[i].x = tmp;
          if(  0 < fscanf(fp, "%s", data));
          sscanf(data, "%f", &tmp);
          polygons[num_poly].vertex[i].y = tmp;
          
          if(polygons[num_poly].vertex[i].x > polygons[num_poly].max.x)  polygons[num_poly].max.x = polygons[num_poly].vertex[i].x;
          if(polygons[num_poly].vertex[i].y > polygons[num_poly].max.y)  polygons[num_poly].max.y = polygons[num_poly].vertex[i].y;
          if(polygons[num_poly].vertex[i].x < polygons[num_poly].min.x)  polygons[num_poly].min.x = polygons[num_poly].vertex[i].x;
          if(polygons[num_poly].vertex[i].y < polygons[num_poly].min.y)  polygons[num_poly].min.y = polygons[num_poly].vertex[i].y;
  
          //printf("polygon vertex %d x %f y %f\n",i,polygons[num_poly].vertex[i].x,polygons[num_poly].vertex[i].y);
          i++;
        }
      }
    }
    else if(strcmp("dimensions", data) == 0  && (flg == 0) )
    {
      if(  0 < fscanf(fp, "%s", data));
      if(  0 < fscanf(fp, "%s", data));
      sscanf(data, "%f", &dimensions_room_x);
      if(  0 < fscanf(fp, "%s", data));
      sscanf(data, "%f", &dimensions_room_y);
      //printf("dimensions x %f y %f\n",dimensions_room_x,dimensions_room_y);
    }
  }
  fclose(fp);
  return num_poly;
}

void read_environment(char *file, int debug)
{

  int i;                                                                            
  int j;
  // it reads the polygons 
  strcpy(polygons_wrl[0].name, "NULL");
  if(debug == 1) printf("\nEnvironment file: %s\n", file);
  num_polygons_wrl = ReadPolygons(file, polygons_wrl);
  
  if(num_polygons_wrl == 0)
    printf("File doesnt exist %s \n",file);
  else  
    printf("Load: %s \n",file);                                                                                                                                                     
  // it prints the polygons
  if(debug == 1)
  for(i = 0; i < num_polygons_wrl; i++)
  {
    printf("\npolygon[%d].name=%s\n",i,polygons_wrl[i].name);
    printf("polygon[%d].type=%s\n",i,polygons_wrl[i].type);
    printf("Num vertex  polygon[%d].num_vertex=%d\n",i,polygons_wrl[i].num_vertex);
      printf("max x,y = (%f, %f)  min x,y = (%f, %f) \n", polygons_wrl[i].max.x, polygons_wrl[i].max.y, polygons_wrl[i].min.x, polygons_wrl[i].min.y);
      //printf("self.w.create_rectangle(%f* self.canvasX/2, (self.canvasY-( %f* self.canvasY )/2) ,  (%f* self.canvasX)/2, (self.canvasY-(%f* self.canvasX)/2), outline='#000000', width=1)\n", polygons_wrl[i].max.x, polygons_wrl[i].max.y, polygons_wrl[i].min.x, polygons_wrl[i].min.y);
    for(j = 0; j <= polygons_wrl[i].num_vertex+1 ; j++)
    {
      printf("polygon[%d].vertex[%d] x=%f y=%f\n", i, j, polygons_wrl[i].vertex[j].x, polygons_wrl[i].vertex[j].y);
      //printf("polygon[%d].line[%d] m=%f b=%f\n", i, j, polygons_wrl[i].line[j].m, polygons_wrl[i].line[j].b);
    }
  }
}



void paramsCallback(const simulator::Parameters::ConstPtr& paramss)
{
  std::string paths = ros::package::getPath("simulator");
  char path[500];
  int k;
  Polygon_2    polygon;
  Polygon_list partition_polys;


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

  if(  strcmp( paramss->world_name.c_str(),actual_world) ) 
  {
    strcpy(path,paths.c_str());
    strcat(path,"/src/data/");
    strcat(path,paramss->world_name.c_str());
    strcat(path,"/");
    strcat(path,paramss->world_name.c_str());
    strcat(path,".wrl");
    read_environment(path,0);
    strcat(actual_world,paramss->world_name.c_str());
    strcpy(actual_world,paramss->world_name.c_str());

    point_list.clear();

    for(int i = 0; i < num_polygons_wrl; i++)
    {
      polygon.clear();
      partition_polys.clear();
      for(int j = 0; j <= polygons_wrl[i].num_vertex; j++ )
          polygon.push_back(Point_2(polygons_wrl[i].vertex[j].x, polygons_wrl[i].vertex[j].y));

      CGAL::approx_convex_partition_2(polygon.vertices_begin(),
                                   polygon.vertices_end(),
                                   std::back_inserter(partition_polys));
      for(  Polygon_list::iterator  vi = partition_polys.begin(); vi != partition_polys.end(); ++vi) 
      { k = 0;  
        for( Polygon_2::Vertex_iterator   vert = vi->vertices_begin(); vert != vi->vertices_end(); ++vert,k++) 
          {
            if(k == 0)
              aux = *vert;
            else if(k == 1)
            {
              aux2 = *vert;
            }
            else
            {
              point_list.push_back(aux); 
              point_list.push_back(aux2); 
              point_list.push_back(*vert); 
              aux2 = *vert;

            }
          }
        }  
    }

  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_tools_demo");
  ros::NodeHandle n;
  ros::Subscriber params_sub = n.subscribe("simulator_parameters_pub", 0, paramsCallback);


  ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "map_marker", 0 );
  ros::Publisher goal_pub = n.advertise<visualization_msgs::Marker>( "goal_marker", 0 );
  ros::Publisher goal_base_pub = n.advertise<visualization_msgs::Marker>( "goal_marker_base", 0 );


visualization_msgs::Marker goal;
goal.header.frame_id = "map";
goal.header.stamp = ros::Time();
goal.ns = "my_namespace2";
goal.id = 0;
goal.type = visualization_msgs::Marker::SPHERE;
goal.action = visualization_msgs::Marker::ADD;
goal.lifetime = ros::Duration(1);
goal.pose.position.x = 0;
goal.pose.position.y = 0;
goal.pose.position.z = 0.1;
goal.pose.orientation.x = 0.0;
goal.pose.orientation.y = 0.0;
goal.pose.orientation.z = 0.0;
goal.pose.orientation.w = 1.0;
goal.scale.x = .05;
goal.scale.y = .05;
goal.scale.z = .05;
goal.color.a = 1.0; // Don't forget to set the alpha!
goal.color.r = 1.0;
goal.color.g = 0.8;
goal.color.b = 0.0;


visualization_msgs::Marker goal_base;
goal_base.header.frame_id = "map";
goal_base.header.stamp = ros::Time();
goal_base.ns = "my_namespace3";
goal_base.id = 1;
goal_base.type = visualization_msgs::Marker::CYLINDER;
goal_base.action = visualization_msgs::Marker::ADD;
goal_base.lifetime = ros::Duration(1);
goal_base.pose.position.x = 0;
goal_base.pose.position.y = 0;
goal_base.pose.position.z = 0.05;
goal_base.pose.orientation.x = 0.0;
goal_base.pose.orientation.y = 0.0;
goal_base.pose.orientation.z = 0.0;
goal_base.pose.orientation.w = 1.0;
goal_base.scale.x = .01;
goal_base.scale.y = .01;
goal_base.scale.z = .1;
goal_base.color.a = 1.0; // Don't forget to set the alpha!
goal_base.color.r = 1.0;
goal_base.color.g = 0.8;
goal_base.color.b = 0.0;


geometry_msgs::Point pt;

visualization_msgs::Marker marker;
marker.header.frame_id = "map";
marker.header.stamp = ros::Time();
marker.ns = "my_namespace";
marker.id = 2;
marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
marker.action = visualization_msgs::Marker::ADD;
marker.lifetime = ros::Duration(1);
marker.pose.position.x = 0;
marker.pose.position.y = 0;
marker.pose.position.z = 0;
marker.pose.orientation.x = 0.0;
marker.pose.orientation.y = 0.0;
marker.pose.orientation.z = 0.0;
marker.pose.orientation.w = 1.0;
marker.scale.x = 1;
marker.scale.y = 1;
marker.scale.z = 1;
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
    
    marker.points.clear();
    marker.colors.clear();

    for(int i = 0; i < num_polygons_wrl; i++)
    {
      for(int j = 1; j <= polygons_wrl[i].num_vertex; j++ )
      {



          pt.x = polygons_wrl[i].vertex[j-1].x;
          pt.y = polygons_wrl[i].vertex[j-1].y;
          pt.z = 0;
          marker.points.push_back(pt);

          pt.x = polygons_wrl[i].vertex[j].x;
          pt.y = polygons_wrl[i].vertex[j].y;
          pt.z = 0;
          marker.points.push_back(pt);

          pt.x = polygons_wrl[i].vertex[j-1].x;
          pt.y = polygons_wrl[i].vertex[j-1].y;
          pt.z = .06;
          marker.points.push_back(pt);

          pt.x = polygons_wrl[i].vertex[j].x;
          pt.y = polygons_wrl[i].vertex[j].y;
          pt.z = 0;
          marker.points.push_back(pt);

          pt.x = polygons_wrl[i].vertex[j].x;
          pt.y = polygons_wrl[i].vertex[j].y;
          pt.z = .06;
          marker.points.push_back(pt);

          pt.x = polygons_wrl[i].vertex[j-1].x;
          pt.y = polygons_wrl[i].vertex[j-1].y;
          pt.z = .06;
          marker.points.push_back(pt);
          marker.colors.push_back(color);
      }

      pt.x = polygons_wrl[i].vertex[polygons_wrl[i].num_vertex].x;
      pt.y = polygons_wrl[i].vertex[polygons_wrl[i].num_vertex].y;
      pt.z = 0;
      marker.points.push_back(pt);

      pt.x = polygons_wrl[i].vertex[0].x;
      pt.y = polygons_wrl[i].vertex[0].y;
      pt.z = 0;
      marker.points.push_back(pt);

      pt.x = polygons_wrl[i].vertex[polygons_wrl[i].num_vertex].x;
      pt.y = polygons_wrl[i].vertex[polygons_wrl[i].num_vertex].y;
      pt.z = .06;
      marker.points.push_back(pt);

      pt.x = polygons_wrl[i].vertex[0].x;
      pt.y = polygons_wrl[i].vertex[0].y;
      pt.z = 0;
      marker.points.push_back(pt);

      pt.x = polygons_wrl[i].vertex[0].x;
      pt.y = polygons_wrl[i].vertex[0].y;
      pt.z = .06;
      marker.points.push_back(pt);

      pt.x = polygons_wrl[i].vertex[polygons_wrl[i].num_vertex].x;
      pt.y = polygons_wrl[i].vertex[polygons_wrl[i].num_vertex].y;
      pt.z = .06;
      marker.points.push_back(pt);
      marker.colors.push_back(color);
    }

    for(int k =0; k < point_list.size(); k++)
    {
      pt.x = point_list[k].x();
      pt.y = point_list[k].y();
      pt.z = .06;
      marker.points.push_back(pt);
      marker.colors.push_back(color);
    }

    for(int k =0; k < point_list.size(); k++)
    {
      pt.x = point_list[k].x();
      pt.y = point_list[k].y();
      pt.z = .0;
      marker.points.push_back(pt);
      marker.colors.push_back(color);
    }

    vis_pub.publish( marker );
    goal.pose.position.x = params.light_x;
    goal.pose.position.y = params.light_y;
    goal_pub.publish( goal );
    goal_base.pose.position.x = params.light_x;
    goal_base.pose.position.y = params.light_y;
    goal_base_pub.publish( goal_base );


    ros::spinOnce(); 
    r.sleep();

  }

  ROS_INFO_STREAM("Shutting down.");

  return 0;
}



