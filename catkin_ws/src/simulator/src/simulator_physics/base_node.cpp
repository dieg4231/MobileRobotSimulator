#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "ros/ros.h"
#include <ros/package.h>
#include "simulator/Parameters.h"
#include "../utilities/simulator_structures.h"
#include "simulator/simulator_robot_step.h"
#include "simulator/simulator_parameters.h"
#include "simulator/simulator_base.h"
#include <string.h>

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include <iterator>
#include <random>

#define MAX_NUM_POLYGONS 100
#define NUM_MAX_VERTEX 10
#define STRSIZ 300
#define SIZE_LINE 10000

float x2,  y2, theta2;

const double mean = 0.0;
double stddev_distance = 0.005;
double stddev_theta = 0.05;

std::default_random_engine generator;
std::normal_distribution<double> noise_distance(mean, stddev_distance);
std::normal_distribution<double> noise_theta(mean, stddev_theta);

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

float pDistance(float x,float y,float x1,float y1,float x2,float y2) {

  float A = x - x1;
  float B = y - y1;
  float C = x2 - x1;
  float D = y2 - y1;

  float dot = A * C + B * D;
  float len_sq = C * C + D * D;
  float param = -1;
  float dx,dy;
  float  xx, yy;

  if (len_sq != 0) //in case of 0 length line
      param = dot / len_sq;

  if (param < 0) {
    xx = x1;
    yy = y1;
  }
  else if (param > 1) {
    xx = x2;
    yy = y2;
  }
  else {
    xx = x1 + param * C;
    yy = y1 + param * D;
  }

   dx = x - xx;
   dy = y - yy;
   float aux =(dx * dx + dy * dy );

  return sqrt( aux );
}


int sat(float robot_x, float robot_y, float robot_r)
{
	int i,j;
	Vertex r_max;
	Vertex r_min;

	r_max.x = robot_x + robot_r; r_max.y = robot_y + robot_r;
	r_min.x = robot_x - robot_r; r_min.y = robot_y - robot_r;
	
	for(i = 0; i < num_polygons_wrl; i++)
		if( (r_min.x < polygons_wrl[i].max.x && polygons_wrl[i].max.x <   r_max.x) || ( r_min.x < polygons_wrl[i].min.x && polygons_wrl[i].min.x < r_max.x)  || ( polygons_wrl[i].min.x < r_min.x && r_max.x < polygons_wrl[i].max.x )  )
			if( (r_min.y < polygons_wrl[i].max.y && polygons_wrl[i].max.y < r_max.y) || ( r_min.y < polygons_wrl[i].min.y && polygons_wrl[i].min.y < r_max.y) || ( polygons_wrl[i].min.y < r_min.y && r_max.y < polygons_wrl[i].max.y )   )
				for(int j = 0; j <= polygons_wrl[i].num_vertex; j++)
		 			{
		 				if( pDistance(robot_x, robot_y, polygons_wrl[i].vertex[j].x, polygons_wrl[i].vertex[j].y, polygons_wrl[i].vertex[j + 1].x, polygons_wrl[i].vertex[j + 1].y) <= robot_r ) 
						{	
						    return 1;
		 				}
		 					}	
	return 0;
}
	
bool check_path(simulator::simulator_base::Request  &req ,simulator::simulator_base::Response &res)
{
	float x1 = req.x1;
	float y1 = req.y1;
	float m;

	//theta2=req.theta; 
	float x22,  y22;
	float distance_final,distance_test;
	float theta_final;
	char path[50];

		//res.theta = req.theta =  req.theta ;//+ noise_theta(generator);
	
	//req.distance += noise_distance(generator);
	if( params.noise  )
		theta_final = req.theta + noise_theta(generator) ;
	else
		theta_final = req.theta;

	res.theta = theta_final;



	if (req.distance == 0)
	{
		res.distance = 0; 
		return true;
	}

	m = tan(req.orientation + theta_final);

	if( params.noise  )
		distance_test = req.distance + noise_distance(generator);
	else
		distance_test = req.distance;


	if(m > 1 || m < -1 )
	{	
		y22 = distance_test * sin(req.orientation + theta_final) + y1;
		x2 = distance_test * cos(req.orientation + theta_final) + x1;
		y2 = 0;

		if(y22 > y1)
		{	
			for(y2 = y1; y2 <= y22; y2+=.005)
			{
				x2 =  (y2 - y1) / m + x1 ;
				
				if(sat(x2, y2, params.robot_radio))
				{
				break;}
			}
			if(x2 != x1)
			{	
				y2-=.005;
				x2 =  (y2 - y1) / m + x1 ;	
			}
			//printf("y1:%f x1:%f y2 %f x2 %f\n",y1,x1,y2,x2 );
			
		}
		else
		{
			//printf("BBB\n");
			for(y2 = y1; y2 >= y22; y2-=.005)
			{
				//y2 -y1= m ( x2 - x1)
				x2 =  (y2 - y1) / m + x1 ;
				if(sat(x2, y2, params.robot_radio))
				{//printf("Fuera\n");	
				break;}
			}
			if(x2 != x1)
			{
				y2+=.005;
				x2 =  (y2 - y1) / m + x1 ;
			}
		}

	}
	else
	{
		x22 = distance_test * cos(req.orientation + theta_final ) + x1;
		y2 = distance_test  * sin(req.orientation + theta_final ) + y1;
		x2 = 0;

		if(x22-x1 >= 0)
		{
			//printf("CCC\n");
			for(x2 = x1; x2 <= x22; x2+=.005)
			{   
				y2 = m * (x2 - x1) + y1;
				//printf(" x: %f y: %f\n",x2*600,y2*600 );
				if(sat(x2, y2, params.robot_radio))
				{//printf("Fuera\n");
				break;}
			}
			if(x2 != x1)
			{
				x2-=.005;
				y2 = m * (x2 - x1) + y1;
			}
		}
		else
		{
			//printf("DDD\n");
			for(x2 = x1; x2 >= x22; x2-=.005)
			{
				y2 = m * (x2 - x1) + y1;
				if(sat(x2, y2, params.robot_radio))
					break;
			}
			if(x2 != x1)
			{
				x2+=.005;
				y2 = m * (x2 - x1) + y1;
			}
		}
	}
			
  	distance_final = sqrt( pow( x1-x2  ,2) + pow(y1-y2 ,2)  );

    if (req.distance < 0)
    	distance_final = -distance_final;

    res.distance = distance_final/dimensions_room_x;

    //printf("====TTTTTdist reeq: %f   , resp %f \n",req.distance ,res.distance );

   return true;
}


void paramsCallback(const simulator::Parameters::ConstPtr& paramss)
{
  std::string paths = ros::package::getPath("simulator");
  char path[500];


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
	}

}

int main(int argc, char *argv[])
{	
	ros::init(argc, argv, "simulator_base_node");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("simulator_base", check_path);
	ros::Subscriber params_sub = n.subscribe("simulator_parameters_pub", 0, paramsCallback);
	ros::spin();
	/*	
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	tf::TransformBroadcaster odom_broadcaster;

  	double x = 0.0;
  	double y = 0.0;
  	double th = 0.0;

  	double vx = 1.1;
  	double vy = -1.1;
  	double vth = 1.1;

  	ros::Time current_time, last_time;
  	current_time = ros::Time::now();
  	last_time = ros::Time::now();
  

	ros::Rate r(2400.0);
	
	while(n.ok())
	{

    ros::spinOnce();               // check for incoming messages
    
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta2);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x2;
    odom_trans.transform.translation.y = y2;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "map";

    //set the position
    odom.pose.pose.position.x = x2;
    odom.pose.pose.position.y = y2;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = 0;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
   
    r.sleep();
  }
	 */
	return 0;
}

