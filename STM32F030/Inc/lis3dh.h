#include "main.h"

typedef struct Acc{
	int16_t RawX;
	int16_t RawY;
	int16_t RawZ;
}AccLIS;


#define LIS3DH_I2C_ADD_H   0x33U
#define LIS3DH_WHO_AM_I    0x0FU
#define LIS3DH_ID          0x33U
#define LIS3DH_OUT_X_L     0x28U

#define LIS3DH_CTRL_REG1   0x20U
#define LIS3DH_CTRL_REG2   0x21U
#define LIS3DH_CTRL_REG3   0x22U
#define LIS3DH_CTRL_REG4   0x23U
#define LIS3DH_CTRL_REG5   0x24U
#define LIS3DH_CTRL_REG6   0x25U

#define LIS3DH_STATUS_REG  0x27U

//void AccGetPos(int16_t *raw);
//static void Acc_read(uint8_t reg, uint8_t *bufp);
//static void Acc_write(uint8_t reg, uint8_t bufp);

void InitAcc(void);
