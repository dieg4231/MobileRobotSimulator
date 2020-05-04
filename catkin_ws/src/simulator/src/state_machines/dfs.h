/***********************************************
*                                              *
*      dfs.h			               *
*                                              *
*      Diego Cordero                           *
*                                              *
*              Bio-Robotics Laboratory         *
*              UNAM, 2019                      *
*                                              *
*                                              *
************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ros/package.h>

int stack[500];
int sp = 0;
void push(int v){ stack[sp++] = v;}

void pop(){sp--;}

void print_stack(){
	printf("\nstack: ");
    for(int i=0; i < sp; i++)
    	printf(" %d ",stack[i]);
    printf("\n");
}

void dfs_algorithm(int D ,int L)
{
	 sp=0;
	int menor,flagOnce;
	int contador=0;
	int node_actual = D;
	int flagPush;

	while(node_actual != L)
	{	
		print_stack();
		flagPush = 1;
		//printf("aa  %d \n",nodes[node_actual].num_conections );
		for( int j = 0; j < nodes[node_actual].num_conections; j++)
   		{
   			//printf("** %c** \n",nodes[nodes[node_actual].conections[j].node].flag );
   			if(nodes[nodes[node_actual].conections[j].node].flag == 'N')
   			{
   				//printf("Node actual %d \n",node_actual);
   				nodes[nodes[node_actual].conections[j].node].flag = 'Y';
   				push(node_actual);
   				nodes[node_actual].flag='Y';
   				node_actual = nodes[nodes[node_actual].conections[j].node].num_node;
   				//printf("Node actual %d \n",node_actual);
   				flagPush = 0;
   				break;	
   			}
   		}
   		
   		if( flagPush == 1 )
   		{	
   			pop();
   			node_actual = sp - 1;
		}
	}
	push(node_actual);
}




int dfs(float rx ,float ry ,float lx ,float ly, char *world_name,step *steps )
{
   //char archivo[]="../data/obstacles/obstacles.top";
    char archivo[150];
   int i;
     int start = 0;
   int goal = 0;
   //"../data/obstacles/obstacles.top";
   std::string paths = ros::package::getPath("simulator");
   strcpy(archivo,paths.c_str());
   strcat(archivo,"/src/data/");
   strcat(archivo,world_name);
   strcat(archivo,"/");
   strcat(archivo,world_name);
   strcat(archivo,".top");


   for(int i=0; i<200; i++)
   {
   		nodes[i].flag='N';
   		nodes[i].num_conections = 0;
   		nodes[i].parent = -1;
   		nodes[i].acumulado = 0;
   }
   num_nodes=read_nodes(archivo);
   //printf("NUmero de nodos #: %d \n",num_nodes);
   for(i = 1; i < num_nodes; i++)
   {
   		if ( sqrt(pow( nodes[i].x - rx ,2) + pow( nodes[i].y - ry ,2)) < sqrt( pow( nodes[start].x - rx ,2) + pow( nodes[start].y - ry ,2)) )
   		{	//	printf("r-n : %d Distancia %f  Node x %f  node y %f   rx %f  ry%f \n",i,sqrt(pow( nodes[i].x - rx ,2) + pow( nodes[i].y - ry ,2)),nodes[i].x,nodes[i].y,rx,ry  );
   			start = i;
   		}
   		if (sqrt(pow( nodes[i].x - lx ,2) + pow( nodes[i].y - ly ,2)) < sqrt(pow( nodes[goal].x - lx ,2) + pow( nodes[goal].y - ly ,2) ) )
   			goal = i;
   }
   //for(int i=0; i<num_nodes; i++)
   //	printNode(i);
  
   //printf("%d %d \n",atoi(argv[1]),atoi(argv[2]) );

   dfs_algorithm(start,goal);

   //printf("Final Stack\n");
   //print_stack();


    for(int i=0; i < sp; i++)
    {
    	//printf(" %d ",stack[i]);
    	steps[i].node = nodes[stack[i]].num_node;
	   	steps[i].x = nodes[stack[i]].x;
	   	steps[i].y = nodes[stack[i]].y;
    }
    //printf("\n");


	return 0;
} 
