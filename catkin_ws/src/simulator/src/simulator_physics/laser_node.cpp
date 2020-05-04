#include "ros/ros.h"
#include <ros/package.h>
#include "simulator/Parameters.h"
//#include "simulator/Laser_values.h"
#include "simulator/simulator_laser.h"
#include "../utilities/simulator_structures.h"
#include "simulator/simulator_robot_step.h"
#include "simulator/simulator_parameters.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Header.h"
//#include "simulator/simulator_base.h"
#include <string.h>

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <iterator>
#include <random>

#include <boost/thread/thread.hpp>  

#define MAX_NUM_POLYGONS 100
#define NUM_MAX_VERTEX 10
#define STRSIZ 300

std::default_random_engine generator;
double stddev = 0.015;
std::normal_distribution<double> noise(0, stddev);

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
float sensors[512];
float X_pose,Y_pose;
char actual_world[50];
double robot_x;
double robot_y;
double robot_theta;
parameters params;

// it reads the file that conteins the environment description
int ReadPolygons(char *file,Polygon *polygons){

	FILE *fp;
	char data[ STRSIZ ];
	int i;
	int num_poly = 0;
	int flg = 0;
	float tmp;
	float dimensions_room_x,dimensions_room_y;

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
	/* it reads the polygons */
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

float getValue(Vertex p1, Vertex p2, Vertex p3 ,Vertex p4, float laser_value)
{

	float denominadorTa,ta,xi;
	float denominadorTb,tb,yi;
    
	denominadorTa =(p4.x-p3.x)*(p1.y-p2.y) - (p1.x-p2.x)*(p4.y-p3.y);
	denominadorTb =(p4.x-p3.x)*(p1.y-p2.y) - (p1.x-p2.x)*(p4.y-p3.y);
	
	if( denominadorTa == 0 || denominadorTb ==0 )
		return laser_value;

	ta = ( (p3.y-p4.y)*(p1.x-p3.x) + (p4.x-p3.x)*(p1.y-p3.y) )  /  ( denominadorTa );
	tb = ( (p1.y-p2.y)*(p1.x-p3.x) + (p2.x -p1.x)*(p1.y-p3.y) ) / ( denominadorTb );
    
    //printf("TTT  %f  %f \n",ta,tb );
   // printf("1 %f    %f \n",p1.x,p1.y );
 	//printf("2 %f    %f \n",p2.x,p2.y );
 	//printf("3 %f    %f \n",p3.x,p3.y );
 	//printf("4 %f    %f \n",p4.x,p4.y );
	if(  0 <= ta  && ta <=1 &&   0 <= tb && tb <=1  )
	{
		xi = p1.x  + ta * ( p2.x - p1.x ) ;
		yi = p1.y  + ta * ( p2.y - p1.y ) ;
		//printf("%s x %f  y% f \n","holaa",xi,yi );
		//printf("x1 %f  y1 %f  x2 %f  y2%f \n",p1.x,p1.y,p2.x,p2.y );	
		//printf("x3 %f  y3 %f  x4 %f  y4%f \n",p3.x,p3.y,p4.x,p4.y );
		return sqrt( pow(p1.x-xi,2) + pow(p1.y-yi,2)  ) ;
	}		
	else
		return laser_value;
	//resa=[p1.x p1.y] + ta*( [p2.x p2.y]- [p1.x p1.y
	//resb= [p3.x p3.y] + tb*( [p4.x p4.y]- [p3.x p3.y]  )

}

int getValues(float laser_num_sensors, float laser_origin, float laser_range,float laser_value,float robot_x ,float robot_y ,float robot_theta, float *values)
{
	int i,j,k,m;
	float f,step,aux;
	Vertex r_max;
	Vertex r_min;

	Vertex p1;
	Vertex p2;
	p1.x = robot_x;
	p1.y = robot_y;

	r_max.x = robot_x + laser_value; r_max.y = robot_y + laser_value;
	r_min.x = robot_x - laser_value; r_min.y = robot_y - laser_value;

	int posible_collision[512];
	 //printf("self.w.create_rectangle(%f* self.canvasX, (self.canvasY-( %f* self.canvasY )) ,  (%f* self.canvasX), (self.canvasY-(%f* self.canvasX)), outline='#000000', width=1)\n", r_max.x, r_max.y, r_min.x, r_min.y);
	
	for (i=0;i < 512;i++)
		posible_collision[i]=-1;
	j=0;

	for (i=0;i < laser_num_sensors;i++)
		values[i]=laser_value;	

	for(i = 0; i < num_polygons_wrl; i++)
		if( (r_min.x < polygons_wrl[i].max.x && polygons_wrl[i].max.x <   r_max.x) || ( r_min.x < polygons_wrl[i].min.x && polygons_wrl[i].min.x < r_max.x)  || ( polygons_wrl[i].min.x < r_min.x && r_max.x < polygons_wrl[i].max.x )  )
			if( (r_min.y < polygons_wrl[i].max.y && polygons_wrl[i].max.y < r_max.y) || ( r_min.y < polygons_wrl[i].min.y && polygons_wrl[i].min.y < r_max.y) || ( polygons_wrl[i].min.y < r_min.y && r_max.y < polygons_wrl[i].max.y )   )
				//printf("%i Aa\n",
				posible_collision[j++]=i;// );

    f = robot_theta + laser_origin;

    step = laser_range  / ( laser_num_sensors - 1 );

    for ( k = 0; k < laser_num_sensors; k++)
    	{
    		for(i = 0; i < j; i++)
			{
				for( m = 0; m <= polygons_wrl[posible_collision[i]].num_vertex; m++)
				{
					p2.x = laser_value * cos( f ) + p1.x;
					p2.y = laser_value * sin( f ) + p1.y;
					//printf("Laser: %d  Poligon: %d  Valu: %f \n",k,i,values[k] );
					aux = getValue(p1,p2,polygons_wrl[posible_collision[i]].vertex[m],polygons_wrl[ posible_collision[i] ].vertex[m+1],laser_value);
					//printf("Laser: %d  Poligon: %d  Valu: %f \n",k,i,values[k] );
		            //printf("x1 %f  y1 %f  x2 %f  y2%f \n",p1.x,p1.y,p2.x,p2.y );	
		            //printf("x3 %f  y3 %f  x4 %f  y4%f \n\n",polygons_wrl[i].vertex[m].x,polygons_wrl[i].vertex[m].y,polygons_wrl[i].vertex[m+1].x,polygons_wrl[i].vertex[m+1].y );
		
					if(values[k] > aux)
						values[k] = aux;

					}
			}
			f += step;
    }
	return 0;
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

	if( strcmp( paramss->world_name.c_str(),actual_world) ) 
	{
		strcpy(path,paths.c_str());
		strcat(path,"/src/data/");
		strcat(path,paramss->world_name.c_str());
		strcat(path,"/");
		strcat(path,paramss->world_name.c_str());
		strcat(path,".wrl");
		read_environment(path,0);
		strcpy(actual_world,paramss->world_name.c_str());
	}
}


bool laserCallback(simulator::simulator_laser::Request  &req ,simulator::simulator_laser::Response &res)
{
	float   valores1[512];
	float   valores2[50];
	int j=0;
	getValues(params.laser_num_sensors ,params.laser_origin ,params.laser_range ,params.laser_value ,req.robot_x ,req.robot_y ,req.robot_theta ,valores1);
	
	if(params.noise)
		for (int i =0 ; i<params.laser_num_sensors;i++,j++)
			res.sensors[i] = valores1[i] + noise(generator);
	else
		for (int i =0 ; i<params.laser_num_sensors;i++,j++)
			res.sensors[i] = valores1[i];


	//boost::thread first (getValues,params.laser_num_sensors/2 ,params.laser_origin ,params.laser_range/2 ,params.laser_value ,req.robot_x ,req.robot_y ,req.robot_theta ,valores1);
	//boost::thread second (getValues,params.laser_num_sensors/2 ,params.laser_origin + params.laser_range/2 ,params.laser_range/2 ,params.laser_value ,req.robot_x ,req.robot_y ,req.robot_theta ,valores2);
	//first.join();
	//second.join();
	//for (int i =0 ; i<params.laser_num_sensors/2;i++,j++)
	//	res.sensors[j] = valores1[i];

	//for (int i =0 ; i<params.laser_num_sensors/2;i++,j++)
	//	res.sensors[j] = valores2[i];

	return true;
}


int main(int argc, char *argv[])
{	
	ros::init(argc, argv, "simulator_laser_node");
	ros::NodeHandle n;
	ros::Subscriber params_sub = n.subscribe("simulator_parameters_pub", 0, paramsCallback);
	ros::ServiceServer service = n.advertiseService("simulator_laser_serv", laserCallback);
	//ros::spin();

	
	ros::Publisher pubLaseScan = n.advertise<sensor_msgs::LaserScan>("/scan_simul", 1);
	
	actual_world[0] = '\0';
	
	char a[200]="/home/pumas/Documents/diego/catkin_ws/src/simulator/src/data/random_2/random_2.wrl";
	
	read_environment(a,0);
	float   valores1[512];
	float   valores2[50];
    std_msgs::Header header;
    sensor_msgs::LaserScan msg;
    ros::Rate loop_rate(50);

	while (ros::ok())
	{
		
	header.frame_id = "base_link_rob2w";
	msg.header = header;
	msg.angle_min = params.laser_origin;
	msg.angle_max = params.laser_origin + params.laser_range;
	msg.angle_increment = (double)params.laser_range / (double)params.laser_num_sensors;
	msg.range_min = 0.0;
	msg.range_max = params.laser_value;
	msg.ranges.resize(params.laser_num_sensors);


	getValues(params.laser_num_sensors ,params.laser_origin ,params.laser_range ,params.laser_value ,params.robot_x ,params.robot_y ,params.robot_theta ,valores1);
	
	for (int i =0 ; i<params.laser_num_sensors;i++)
		//printf("%f \n",valores1[i] );
		msg.ranges[i] = valores1[i];

		pubLaseScan.publish(msg);
	    
	    ros::spinOnce();
	    loop_rate.sleep();

	}
	

	return 0;
}