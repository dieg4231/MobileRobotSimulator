#ifndef DIJSTRALIB_H
#define DIJSTRALIB_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ros/package.h>
#define MUM_NODES 250

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

nodo nodes[MUM_NODES];
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
      else if((strcmp("node", data ) == 0) && ( flg == 0 ) )
      {
         if(  0 < fscanf(fp, "%s", data));
         nodes[i].num_node = atoi(data);
         if(  0 < fscanf(fp, "%s", data));
         nodes[i].x = atof(data);
         if(  0 < fscanf(fp, "%s", data));
         nodes[i++].y = atof(data);
      }
      else if((strcmp("connection", data ) == 0) && ( flg == 0 ) )
      {
         if(  0 < fscanf(fp, "%s", data));
         node_index = atoi(data);
         
         if(  0 < fscanf(fp, "%s", data));
         node_connection = atoi(data);

         nodes[node_index].conections[nodes[node_index].num_conections].node = node_connection;

         if(  0 < fscanf(fp, "%s", data));
         nodes[node_index].conections[nodes[node_index].num_conections].cost = atof(data);
         nodes[node_index].num_conections++;
      }
   }
   fclose(fp);
   return i;
}
#endif