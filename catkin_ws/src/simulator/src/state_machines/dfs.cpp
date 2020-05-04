#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct conection_
{
	int node;
	float cost;
}  conection;

typedef struct nodo_
{
	char flag;
	int num_node;
	float x;
	float y;
	int num_conections;
	conection conections[100];

	int parent;
	float acumulado;
}  nodo;

nodo nodes[250];
int num_nodes = 0;

// it reads the file that conteins the environment description
int read_nodes(char *file)
{
	FILE *fp;

	char data[ 100 ];
	int i=0;
	int flg = 0;
	float tmp;
	float dimensions_room_x,dimensions_room_y;


	int node_index,node_connection,cost;

	fp = fopen(file,"r"); 
	 
	if( fp == NULL )
	{
		sprintf(data, "File %s does not exists\n", file);
		printf("File %s does not exists\n", file);
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
				fscanf(fp, "%s", data);
				sscanf(data, "%f", &tmp);
				if(strcmp(")", data) == 0) flg = 0;
			}
		}
		else if((strcmp("node", data ) == 0) && ( flg == 0 ) )
		{
			//( node 0 0.309586 0.61346 )
			fscanf(fp, "%s" ,data);
			nodes[i].num_node = atoi(data);
			fscanf(fp, "%s" ,data);
			nodes[i].x = atof(data);
			fscanf(fp, "%s" ,data);
			nodes[i++].y = atof(data);
		}
		else if((strcmp("connection", data ) == 0) && ( flg == 0 ) )
		{
			//( node 0 0.309586 0.61346 )
			//( connection 0 4 0.112605 )
			fscanf(fp, "%s" ,data);
			node_index = atoi(data);
			
			fscanf(fp, "%s" ,data);
			node_connection = atoi(data);

			nodes[node_index].conections[nodes[node_index].num_conections].node = node_connection;

			fscanf(fp, "%s" ,data);
			nodes[node_index].conections[nodes[node_index].num_conections].cost = atof(data);
			nodes[node_index].num_conections++;
		}
	}
	fclose(fp);
	return i;
}



void printNode(int i)
{
	printf("\n\n");
   	printf("# %d  x   %f y %f\n",nodes[i].num_node,nodes[i].x,nodes[i].y );
   	printf("flag: %c parent: %d   acumulado: %f  \n",nodes[i].flag,nodes[i].parent,nodes[i].acumulado  );
   	printf("num_conections %d \n",nodes[i].num_conections);
   	for(int j=0 ; j < nodes[i].num_conections; j++  )
   		printf(     "%d  %f \n",nodes[i].conections[j].node,nodes[i].conections[j].cost );
  
}


int stack[250];
int sp=0;
void push(int v){stack[sp++] = v;}
void pop(){sp--;}
void print_stack(){
	printf("\nstack: ");
    for(int i=0; i < sp; i++)
    {
    	printf(" %d ",stack[i]);
    }
    printf("\n");
}

void dfs(int D ,int L)
{
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
   				node_actual = nodes[node_actual].conections[j].node;
   				//printf("Node actual %d \n",node_actual);
   				flagPush = 0;
   				break;	
   			}
   		}
   		
   		if( flagPush==1 )
   		{	
   			pop();
   			node_actual = sp - 1;
		}
	}
	push(node_actual);
}

int main(int argc, char ** argv)
{
   char archivo[]="../data/obstacles/obstacles.top";
   
   for(int i=0; i<200; i++)
   {
   		nodes[i].flag='N';
   		nodes[i].num_conections = 0;
   		nodes[i].parent = -1;
   		nodes[i].acumulado = 0;
   }
   printf("NUmero de nodos #: %d \n",num_nodes=read_nodes(archivo));

   //for(int i=0; i<num_nodes; i++)
   //	printNode(i);
  
  printf("%d %d \n",atoi(argv[1]),atoi(argv[2]) );
   dfs(atoi(argv[1]),atoi(argv[2]));
   printf("Final Stack\n");
   print_stack();
	return 0;
} 