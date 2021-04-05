#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "lis3dh.h"
//extern I2C_HandleTypeDef hi2c1;


//static int32_t mplatform_write(void *handle, uint8_t reg, uint8_t bufp, uint16_t len){
//	if (handle == &hi2c1){
//    reg |= 0x80;
//    HAL_I2C_Mem_Write(handle, LIS3DH_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, &bufp, len, 1000);
//  }
//	return 0;
//}

//static int32_t mplatform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len){
//	if (handle == &hi2c1){
//    reg |= 0x80;
//    HAL_I2C_Mem_Read(handle, LIS3DH_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
//	}
//	return 0;
//}
//int done=1;
//void AccSetGet(uint8_t reg,uint8_t *tmp){
//		mplatform_write(&hi2c1,reg,*tmp,1);
//		mplatform_read(&hi2c1,reg,tmp,1);
//}

//void AccGet(uint8_t reg,uint8_t *tmp){
//		mplatform_read(&hi2c1,reg,tmp,1);
//}

//void AccSet(uint8_t reg,uint8_t tmp){
//		mplatform_write(&hi2c1,reg,tmp,1);
//}

//void AccGetPos(AccXYZ *RawAcc){
//	static uint8_t Acc[6];
//	mplatform_read(&hi2c1,LIS3DH_OUT_X_L,Acc,6);
//		
//	RawAcc->X = (int16_t) (Acc[0]| (((uint16_t)Acc[1]) << 8));
//	RawAcc->Y = (int16_t) (Acc[2]| (((uint16_t)Acc[3]) << 8));
//	RawAcc->Z = (int16_t) (Acc[4]| (((uint16_t)Acc[5]) << 8));
//	//printf("Raw Data %d\t%d\t%d\t \n ",RawAcc->X,RawAcc->Y,RawAcc->Z);
//}

//void InitAcc(){
//	static uint8_t whoamI;
//	while(whoamI != LIS3DH_ID){
//		AccGet(LIS3DH_WHO_AM_I, &whoamI);
//		HAL_Delay(1000);
//	}
//	printf("LIS3DH Activated \n");
//	static uint8_t val=0;
//	val = 0x50| 0x04| 0x02| 0x01;
//	AccSetGet(LIS3DH_CTRL_REG1,&val);
//	AccSetGet(LIS3DH_CTRL_REG2,0);
//	AccSetGet(LIS3DH_CTRL_REG3,0);
//	AccSetGet(LIS3DH_CTRL_REG4,0);
//	AccSetGet(LIS3DH_CTRL_REG5,0);
//	AccSetGet(LIS3DH_CTRL_REG6,0);
//	AccGet(LIS3DH_STATUS_REG,&val);
//}
