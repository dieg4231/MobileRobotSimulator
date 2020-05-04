/************************************************************************
*									*
*	random.h							*
*									*
*	This program generates Uniform or Gaussian random numbers.	*
*									*
*									*
*			J. Savage 					*
*			DEPFI-UNAM	11-2003				*
*									*
*************************************************************************/				
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

#define Lmin 0.0
#define Lmax 1.0
#define PERCENTAGE_CHANGE 0.10
#define MAX_NUM_SENSORS 1024



typedef struct _raw{
        int flag;
         int region;
         float x,y,theta[MAX_NUM_SENSORS],sensors[MAX_NUM_SENSORS];
} Raw;



// generates a uniform random variable
float generaR(float Min,float Max){
 float n;
 //int BIG= 0x7FFFFFFF;
 int BIG= RAND_MAX;

 n=Min+ ((float) random()/(float) BIG ) *(Max-Min);

 return n;
}



float random_gaussian(float mean, float variance,float *gaussian){
 float u1,u2,v1,v2,w,y,sqrt_variance;

 sqrt_variance=(float) sqrt(variance);

 while(1){

	// it generates a uniform random variable between 0 to 1
 	u1=generaR(Lmin,Lmax);
 	u2=generaR(Lmin,Lmax);

	//printf("u1 %f u2 %f",u1,u2);
 	v1=2*u1-1;
 	v2=2*u2-1;
 	w=v1*v1+v2*v2;

	//printf(" w %f \n",w);
	if(w < 1) {
		y=(float) sqrt( ((-2*log(w))/w));
		gaussian[1]=sqrt_variance*v1*y + mean;
		gaussian[2]=sqrt_variance*v2*y + mean;
 		return(1);
 	}
 }

}



// it adds noise to the sensors' values
void add_noise_obs(Raw *sensor_vector, int num_sensors, char *path){

 int k;
 float noise;
 float gaussian[3];
 float tmp1,tmp2;
 int j;
 FILE *fp;
 char file[300];
 static int flg=0;
 static float mean,variance;
 static float lmin,lmax;
 static int type;



 if(flg==0){
         // Initializes the random generator
        srandom((int)time((time_t *)NULL));
        sprintf(file,"%srandom_settings_advance_angle_sensors.dat",path);
	/* random_settings_advance_angle_sensors.dat, 0 Uniform PDF; 1 Gaussian PDF
		0 -0.004 0.004
		0 0.0 0.03927
		1 0.003 0.0003
	*/
        if((fp=fopen(file,"r")) == NULL){
                        printf("File %s can not be open\n",file);
                        exit(0);
        }
        printf("Random settings file %s\n",file);
        // it reads the settings for the advance noise
        // it reads the settings for the angle noise
	for(j=1;j<7;j++){
        	if(  0 < fscanf(fp,"%f",&tmp1))
        	{}
	}

	// it reads the settings for the sensors noise
        if(  0 < fscanf(fp,"%d",&type)){}
        if(  0 < fscanf(fp,"%f",&tmp1)){}
        if(  0 < fscanf(fp,"%f",&tmp2)){}
        if(type==0){
                lmin=tmp1;
                lmax=tmp2;
        }
        else {
                mean=tmp1;
                variance=tmp2;
        }

        fclose(fp);
        flg=1;
 }

 
 //printf("random num_sensors %d\n",num_sensors);

 for( k=0; k< num_sensors; k++) {

	if(type==1) {
        
		// it generates a Gaussian random variable 
		random_gaussian(mean,variance,gaussian);
        	noise=gaussian[1];
 	}
 	else{
        	// it generates a Uniform random variable
        	noise=generaR(lmin,lmax);
 	}

	   
        //printf("sensor(%d) = %f \n",k,sensor_vector[0].sensors[k]);
	//printf("noise %f\n",noise);
        sensor_vector[0].sensors[k]=sensor_vector[0].sensors[k]+ noise;
        //printf("noise+sensor(%d) = %f \n",k,sensor_vector[0].sensors[k]);

 }



}




void get_random_advance_angle(float *advance, float *angle, char *path)
{

 float gaussian[3],uniform;
 float tmp1,tmp2;
 int i,dist,num_random;
 FILE *fp;
 char file[300];
 static int flg=0;
 static float mean_a,variance_a;
 static float lmin_a,lmax_a;
 static float mean_t,variance_t;
 static float lmin_t,lmax_t;
 static int type_a,type_t;


 printf("Path %s\n",path);
 if(flg==0){
	 // Initializes the random generator
  	srandom((int)time((time_t *)NULL));
	sprintf(file,"%srandom_settings_advance_angle_sensors.dat",path);
	if((fp=fopen(file,"r")) == NULL){
                        printf("File %s can not be open\n",file);
                        exit(0);
       	}
        printf("Random settings file %s\n",file);
	// it reads the settings for the advance noise
    if(  0 < fscanf(fp,"%d",&type_a)){}
	if(  0 < fscanf(fp,"%f",&tmp1)){}
	if(  0 < fscanf(fp,"%f",&tmp2)){}
	if(type_a==0){
		lmin_a=tmp1;
		lmax_a=tmp2;
	}
	else {
		mean_a=tmp1;
		variance_a=tmp2;
	}
	// it reads the settings for the angle noise
    if(  0 < fscanf(fp,"%d",&type_t)){}
	if(  0 < fscanf(fp,"%f",&tmp1)){}
	if(  0 < fscanf(fp,"%f",&tmp2)){}

	if(type_t==0){
		lmin_t=tmp1;
		lmax_t=tmp2;
	}
	else {
		mean_t=tmp1;
		variance_t=tmp2;
	}

 	fclose(fp);
	flg=1;
 }


 if(type_a==1) {
        // it generates a Gaussian random variable 
	random_gaussian(mean_a,variance_a,gaussian);
        //printf("Gaussian %f %f\n",gaussian[1],gaussian[2]);
	*advance=gaussian[1];
 }
 else{
	// it generates a Uniform random variable
        uniform=generaR(lmin_a,lmax_a);
        //printf("Uniform %f",uniform);
	*advance=uniform;
 }

 if(type_t==1) {
        // it generates a Gaussian random variable 
        random_gaussian(mean_t,variance_t,gaussian);
        //printf("Gaussian %f %f\n",gaussian[1],gaussian[2]);
        *angle=gaussian[1];
 }
 else{
        // it generates a Uniform random variable
        uniform=generaR(lmin_t,lmax_t);
        //printf("Uniform %f",uniform);
        *angle=uniform;
 }

 //printf("random noise advance %f angle %f\n",*advance,*angle);
 

}




float read_random_percentage(char *path){

 FILE *fp;
 char file[300];
 int j;
 float percentage = PERCENTAGE_CHANGE;
 float tmp1;
 int type;

 // Initializes the random generator
 srandom((int)time((time_t *)NULL));
 sprintf(file,"%srandom_settings_advance_angle_sensors.dat",path);
 if((fp=fopen(file,"r")) == NULL){
                        printf("File %s can not be open\n",file);
			return percentage;
 }
 printf("Random settings file %s\n",file);
 // it reads the settings for the advance noise
 // it reads the settings for the angle noise
 for(j=1;j<7;j++){
        	if(  0 <  fscanf(fp,"%f",&tmp1) ){}
 }

 // it reads the settings for the sensors noise
 if(  0 <  fscanf(fp,"%d",&type)){}
 if(  0 <  fscanf(fp,"%f",&tmp1)){}
 if(  0 <  fscanf(fp,"%f",&tmp1)){}
 if(  0 <  fscanf(fp,"%f",&percentage)){}

 fclose(fp);

 return percentage;

}

 




int change_bits_random(int value,int num_bits, char *path){

 int new_value;
 int num_bits_mutation;
 int i,j,k;
 int mask;
 float random;
 static float percentage;
 static int flg=1;

 if(flg == 1){
 	percentage=read_random_percentage(path);
	flg=0;
 }

 new_value=value;
 random = generaR(0.0,1.0);
 //printf("random %f percentage %f\n",random,percentage);

 if(random > percentage) return (new_value);

 num_bits_mutation=num_bits;
 k= (int) generaR(0.0,num_bits_mutation);
 //printf("Max. num_bits_change %d num_bits_change %d\n",num_bits_mutation,k+1);

 for(j=0;j<= k; j++){
        i= (int) generaR(0.0,num_bits);
	mask =(1 << i);
	if( (new_value & mask ) == 0) new_value = new_value | mask;
	else {
		new_value = new_value & (~mask);
	} 
        //printf("num %d bit_changed %d\n",j,i);
 }

 return(new_value);

}



