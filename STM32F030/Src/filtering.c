#include "filtering.h"

//void GFilter(const short In[16], double val[16])
//{
//  int k;
//  short b[16];
//  int naxpy;
//  int j;
//  int val_tmp;
//  double as;
//  for (k = 0; k < 16; k++) {
//    b[k] = In[k];
//    val[k] = 0.0;
//  }

//  for (k = 0; k < 16; k++) {
//    if (16 - k < 2) {
//      naxpy = 15 - k;
//    } else {
//      naxpy = 1;
//    }

//    for (j = 0; j <= naxpy; j++) {
//      val_tmp = k + j;
//      val[val_tmp] += (double)b[k] * 0.0031;
//    }

//    if (15 - k < 1) {
//      naxpy = 14 - k;
//    } else {
//      naxpy = 0;
//    }

//    as = -val[k];
//    for (j = 0; j <= naxpy; j++) {
//      val_tmp = (k + j) + 1;
//      val[val_tmp] += as * (1.0 + -1.9937 * ((double)j + 1.0));
//    }
//  }
//}

/*
void GFilter(const short In[16], double val[16])
{
  int k;
  short b[16];
  int naxpy;
  int j;
  int val_tmp;
  static const double dv0[3] = { 3.913020539917E-5, 7.826041079834E-5,
    3.913020539917E-5 };

  double as;
  static const double dv1[3] = { 1.0, -1.9822289297925291, 0.982385450614126 };

  for (k = 0; k < 16; k++) {
    b[k] = In[k];
    val[k] = 0.0;
  }

  for (k = 0; k < 16; k++) {
    if (16 - k < 3) {
      naxpy = 15 - k;
    } else {
      naxpy = 2;
    }

    for (j = 0; j <= naxpy; j++) {
      val_tmp = k + j;
      val[val_tmp] += (double)b[k] * dv0[j];
    }

    if (15 - k < 2) {
      naxpy = 14 - k;
    } else {
      naxpy = 1;
    }

    as = -val[k];
    for (j = 0; j <= naxpy; j++) {
      val_tmp = (k + j) + 1;
      val[val_tmp] += as * dv1[j + 1];
    }
  }
}

*/


/*

void GFilter(const short In[8], double val[8])
{
  int k;
  short b[8];
  int naxpy;
  int j;
  int val_tmp;
  static const double dv0[3] = { 3.913020539917E-5, 7.826041079834E-5,
    3.913020539917E-5 };

  double as;
  static const double dv1[3] = { 1.0, -1.9822289297925291, 0.982385450614126 };

  for (k = 0; k < 8; k++) {
    b[k] = In[k];
    val[k] = 0.0;
  }

  for (k = 0; k < 8; k++) {
    if (8 - k < 3) {
      naxpy = 7 - k;
    } else {
      naxpy = 2;
    }

    for (j = 0; j <= naxpy; j++) {
      val_tmp = k + j;
      val[val_tmp] += (double)b[k] * dv0[j];
    }

    if (7 - k < 2) {
      naxpy = 6 - k;
    } else {
      naxpy = 1;
    }

    as = -val[k];
    for (j = 0; j <= naxpy; j++) {
      val_tmp = (k + j) + 1;
      val[val_tmp] += as * dv1[j + 1];
    }
  }
}
*/


void GFilter(const short In[8], double val[8])
{
  int k;
  short b[8];
  int naxpy;
  int j;
  int val_tmp;
  static const double dv0[3] = { 0.0015514842347572, 0.0031029684695144,
    0.0015514842347572 };

  double as;
  static const double dv1[3] = { 1.0, -1.964460580205232, 0.965081173899135 };

  for (k = 0; k < 8; k++) {
    b[k] = In[k];
    val[k] = 0.0;
  }

  for (k = 0; k < 8; k++) {
    if (8 - k < 3) {
      naxpy = 7 - k;
    } else {
      naxpy = 2;
    }

    for (j = 0; j <= naxpy; j++) {
      val_tmp = k + j;
      val[val_tmp] += (double)b[k] * dv0[j];
    }

    if (7 - k < 2) {
      naxpy = 6 - k;
    } else {
      naxpy = 1;
    }

    as = -val[k];
    for (j = 0; j <= naxpy; j++) {
      val_tmp = (k + j) + 1;
      val[val_tmp] += as * dv1[j + 1];
    }
  }
}



/*
void AFilter(const short In[16], double val[16])
{
  int k;
  short b[16];
  int naxpy;
  int j;
  int val_tmp;
  static const double dv0[5] = { 9.661396682681E-11, 3.8645586730723E-10,
    5.7968380096085E-10, 3.8645586730723E-10, 9.661396682681E-11 };

  double as;
  static const double dv1[5] = { 1.0, -3.9835812586585209, 5.9508784292667,
    -3.9510124365728339, 0.983715267510479 };

  for (k = 0; k < 16; k++) {
    b[k] = In[k];
    val[k] = 0.0;
  }

  for (k = 0; k < 16; k++) {
    if (16 - k < 5) {
      naxpy = 15 - k;
    } else {
      naxpy = 4;
    }

    for (j = 0; j <= naxpy; j++) {
      val_tmp = k + j;
      val[val_tmp] += (double)b[k] * dv0[j];
    }

    if (15 - k < 4) {
      naxpy = 14 - k;
    } else {
      naxpy = 3;
    }

    as = -val[k];
    for (j = 0; j <= naxpy; j++) {
      val_tmp = (k + j) + 1;
      val[val_tmp] += as * dv1[j + 1];
    }
  }
}
*/

void AFilter(const short In[8], double val[8])
{
  int k;
  short b[8];
  int naxpy;
  int j;
  int val_tmp;
  static const double dv0[5] = { 9.661396682681E-11, 3.8645586730723E-10,
    5.7968380096085E-10, 3.8645586730723E-10, 9.661396682681E-11 };

  double as;
  static const double dv1[5] = { 1.0, -3.9835812586585209, 5.9508784292667,
    -3.9510124365728339, 0.983715267510479 };

  for (k = 0; k < 8; k++) {
    b[k] = In[k];
    val[k] = 0.0;
  }

  for (k = 0; k < 8; k++) {
    if (8 - k < 5) {
      naxpy = 7 - k;
    } else {
      naxpy = 4;
    }

    for (j = 0; j <= naxpy; j++) {
      val_tmp = k + j;
      val[val_tmp] += (double)b[k] * dv0[j];
    }

    if (7 - k < 4) {
      naxpy = 6 - k;
    } else {
      naxpy = 3;
    }

    as = -val[k];
    for (j = 0; j <= naxpy; j++) {
      val_tmp = (k + j) + 1;
      val[val_tmp] += as * dv1[j + 1];
    }
  }
}






void InitKalman(kalmanS *kal,float Q,float R)
{
	kal->Q= Q;
	kal->R= R;
}

float ProcessKalman(kalmanS *kal,double in)
{
	 	kal->temp_est = kal->est_last;
		kal->temp = kal->last + kal->Q;
		kal->K = kal->temp * (1.0/(kal->temp + kal->R));
		kal->input = in; 
		kal->output = kal->temp_est + kal->K * (kal->input - kal->temp_est); 
		kal->P = (1- kal->K) * kal->temp;
		kal->last = kal->P;
		kal->est_last = kal->output;
		return kal->output;
}


void ApplyAccFilter(IMUS *Acc);
void ApplyGyroFilter(IMUS *Gyro);
void ApplyMagFilter(IMUS *Mag);
