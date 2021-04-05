#ifndef GFILTER_H
#define GFILTER_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include <inttypes.h>

/* Function Declarations */
//extern void GFilter(const short In[16], double val[16]);
//double GFilter(const short In[8]);
//double AFilter(const short In[8]);
void AFilter(const short In[8], double val[8]);
void GFilter(const short In[8], double val[8]);

typedef struct kalmanS{
	float est_last;
	float last;
	
	float Q;
	float R;

	float K;
	float P;
	float temp;
	float temp_est;
	float output;
	float input;
}kalmanS;

enum 
{
	X,
	Y,
	Z,
};

//typedef struct IMUSw{
//	short In[3][16];
//	double Out[3];
//	int32_t Bais[3];
//	double roll;
//	double pitch;
//	double yaw;
//	
//}IMUS;

typedef struct IMUS{
	short In[3][16];
	double Out[3][16];
	int32_t Bais[3];
	
	double mean[3];
	double roll;
	double pitch;
	double yaw;
	
}IMUS;

void InitKalman(kalmanS *kal,float Q,float R);
float ProcessKalman(kalmanS *kal,double in);

#endif
