#include "stm32f03_discovery_accelerometer.h"
#include "lsm303dlhc.h"
#include "lis3dh.h"

ACCELERO_DrvTypeDef Lsm303dlhcDrv =
{
  LSM303DLHC_AccInit,
  LSM303DLHC_AccDeInit,
  LSM303DLHC_AccReadID,
  LSM303DLHC_AccRebootCmd,
  0,
  LSM303DLHC_AccZClickITConfig,
  0,
  0,
  0,
  0,
  LSM303DLHC_AccFilterConfig,
  LSM303DLHC_AccFilterCmd,
  LSM303DLHC_AccReadXYZ
};

void LSM303DLHC_AccInit(uint16_t InitStruct)
{  
  uint8_t ctrl = 0x00;
  
  COMPASSACCELERO_IO_Init();
  
  ctrl = (uint8_t) InitStruct;
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG1_A, ctrl);
  
  ctrl = (uint8_t) (InitStruct << 8);
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, ctrl);
}

void LSM303DLHC_MagInit(uint16_t MagSetup,uint16_t MagScale)
{  
  uint8_t ctrl = 0x00;
  
	//COMPASSACCELERO_IO_Init();
 
	ctrl = (uint8_t) MagSetup;
  COMPASSACCELERO_IO_Write(MAG_I2C_ADDRESS, LSM303DLHC_CRA_REG_M, ctrl);
	
	ctrl = (uint8_t) MagScale;
  COMPASSACCELERO_IO_Write(MAG_I2C_ADDRESS, LSM303DLHC_CRB_REG_M, ctrl);
	
	ctrl = (uint8_t) 00;
  COMPASSACCELERO_IO_Write(MAG_I2C_ADDRESS, LSM303DLHC_MR_REG_M, ctrl);
}

void LSM303DLHC_AccDeInit(void)
{  
}

uint8_t LSM303DLHC_AccReadID(void)
{  
  uint8_t ctrl = 0x00;
  COMPASSACCELERO_IO_Init();
	printf("Device ID is  -> %c  \n",ctrl);
  ctrl = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_WHO_AM_I_ADDR);
	printf("Device ID is  -> %02X  \n",ctrl);
  return ctrl;
}

void LSM303DLHC_AccRebootCmd(void)
{
  uint8_t tmpreg;
  tmpreg = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG5_A);
  tmpreg |= LSM303DLHC_BOOT_REBOOTMEMORY;
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG5_A, tmpreg);
}


void LSM303DLHC_AccFilterConfig(uint8_t FilterStruct) 
{
  uint8_t tmpreg;
  
  tmpreg = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A);
  
  tmpreg &= 0x0C;
  tmpreg |= FilterStruct;
  
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, tmpreg);
}

void LSM303DLHC_AccFilterCmd(uint8_t HighPassFilterState)
{
  uint8_t tmpreg;
  
  tmpreg = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A);
  
  tmpreg &= 0xF7;
  
  tmpreg |= HighPassFilterState;
  
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, tmpreg);
}
extern I2C_HandleTypeDef I2cHandle;
void LSM303DLHC_AccReadXYZ(int16_t* pData)
{
	uint8_t buff[2];
	
	HAL_I2C_Mem_Read(&I2cHandle, ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_H_A << 8 |LSM303DLHC_OUT_X_L_A, I2C_MEMADD_SIZE_16BIT, buff, 2, 10);
	pData[0] = (int16_t)((buff[1] << 8) | buff[0]);
	
	HAL_I2C_Mem_Read(&I2cHandle, ACC_I2C_ADDRESS, LSM303DLHC_OUT_Y_H_A << 8 |LSM303DLHC_OUT_Y_L_A, I2C_MEMADD_SIZE_16BIT, buff, 2, 10);
	pData[1] = (int16_t)((buff[1] << 8) | buff[0]);
	
	HAL_I2C_Mem_Read(&I2cHandle, ACC_I2C_ADDRESS, LSM303DLHC_OUT_Z_H_A << 8| LSM303DLHC_OUT_Z_L_A , I2C_MEMADD_SIZE_16BIT, buff, 2, 10);
	pData[2] = (int16_t)((buff[1] << 8) | buff[0]);	
}

void LSM303DLHC_MagReadXYZ(int16_t* pData)
{
  uint8_t buff[2];
  
	HAL_I2C_Mem_Read(&I2cHandle, MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_H_M << 8 |LSM303DLHC_OUT_X_L_M, I2C_MEMADD_SIZE_16BIT, buff, 2, 10);
	pData[0] = (int16_t)((buff[1] << 8) | buff[0]);
	
	HAL_I2C_Mem_Read(&I2cHandle, MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_H_M << 8 |LSM303DLHC_OUT_Y_L_M, I2C_MEMADD_SIZE_16BIT, buff, 2, 10);
	pData[1] = (int16_t)((buff[1] << 8) | buff[0]);
	
	HAL_I2C_Mem_Read(&I2cHandle, MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_H_M << 8| LSM303DLHC_OUT_Z_L_M , I2C_MEMADD_SIZE_16BIT, buff, 2, 10);
	pData[2] = (int16_t)((buff[1] << 8) | buff[0]);
}

void LSM303DLHC_AccFilterClickCmd(uint8_t HighPassFilterClickState)
{
  uint8_t tmpreg = 0x00;
  
  /* Read CTRL_REG2 register */
  tmpreg = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A);
  
  tmpreg &= ~(LSM303DLHC_HPF_CLICK_ENABLE);
  
  tmpreg |= HighPassFilterClickState;
  
  /* Write value to ACC MEMS CTRL_REG2 regsister */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, tmpreg);
}

void LSM303DLHC_AccIT1Enable(uint8_t LSM303DLHC_IT)
{
  uint8_t tmpval = 0x00;
  
  /* Read CTRL_REG3 register */
  tmpval = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG3_A);
  
  /* Enable IT1 */
  tmpval |= LSM303DLHC_IT;
  
  /* Write value to MEMS CTRL_REG3 register */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG3_A, tmpval);
}

void LSM303DLHC_AccIT1Disable(uint8_t LSM303DLHC_IT)
{
  uint8_t tmpval = 0x00;
  
  /* Read CTRL_REG3 register */
  tmpval = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG3_A);
  
  /* Disable IT1 */
  tmpval &= ~LSM303DLHC_IT;
  
  /* Write value to MEMS CTRL_REG3 register */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG3_A, tmpval);
}

void LSM303DLHC_AccIT2Enable(uint8_t LSM303DLHC_IT)
{
  uint8_t tmpval = 0x00;
  
  /* Read CTRL_REG3 register */
  tmpval = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG6_A);
  
  /* Enable IT2 */
  tmpval |= LSM303DLHC_IT;
  
  /* Write value to MEMS CTRL_REG3 register */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG6_A, tmpval);
}

void LSM303DLHC_AccIT2Disable(uint8_t LSM303DLHC_IT)
{
  uint8_t tmpval = 0x00;
  
  /* Read CTRL_REG3 register */
  tmpval = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG6_A);
  
  /* Disable IT2 */
  tmpval &= ~LSM303DLHC_IT;
  
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG6_A, tmpval);
}

void LSM303DLHC_AccINT1InterruptEnable(uint8_t ITCombination, uint8_t ITAxes)
{  
  uint8_t tmpval = 0x00;
  
  tmpval = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_INT1_CFG_A);
  
  tmpval |= (ITAxes | ITCombination);
  
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_INT1_CFG_A, tmpval);  
}

void LSM303DLHC_AccINT1InterruptDisable(uint8_t ITCombination, uint8_t ITAxes)
{  
  uint8_t tmpval = 0x00;
  
  tmpval = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_INT1_CFG_A);
  
  tmpval &= ~(ITAxes | ITCombination);
  
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_INT1_CFG_A, tmpval);
}

void LSM303DLHC_AccINT2InterruptEnable(uint8_t ITCombination, uint8_t ITAxes)
{  
  uint8_t tmpval = 0x00;
  
  tmpval = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_INT2_CFG_A);
  
  tmpval |= (ITAxes | ITCombination);
  
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_INT2_CFG_A, tmpval);
}

void LSM303DLHC_AccINT2InterruptDisable(uint8_t ITCombination, uint8_t ITAxes)
{  
  uint8_t tmpval = 0x00;
  
  tmpval = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_INT2_CFG_A);
  
  tmpval &= ~(ITAxes | ITCombination);
  
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_INT2_CFG_A, tmpval);
}

void LSM303DLHC_AccClickITEnable(uint8_t ITClick)
{  
  uint8_t tmpval = 0x00;
  
  tmpval = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_CLICK_CFG_A);
  
  tmpval |= ITClick;
  
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_CLICK_CFG_A, tmpval);
  
  tmpval = 0x0A;
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_CLICK_THS_A, tmpval);
  
  tmpval = 0x05;
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_TIME_LIMIT_A, tmpval);
  
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_TIME_LATENCY_A, tmpval);
  
  tmpval = 0x32;
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_TIME_WINDOW_A, tmpval);
}

void LSM303DLHC_AccClickITDisable(uint8_t ITClick)
{  
  uint8_t tmpval = 0x00;
  tmpval = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_CLICK_CFG_A);
  tmpval &= ~ITClick;
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_CLICK_CFG_A, tmpval);
}

void LSM303DLHC_AccZClickITConfig(void)
{  
  COMPASSACCELERO_IO_ITConfig();
  
  //LSM303DLHC_AccIT1Enable(LSM303DLHC_IT1_CLICK);
  
  LSM303DLHC_AccFilterClickCmd(LSM303DLHC_HPF_CLICK_ENABLE);
  
  //LSM303DLHC_AccClickITEnable(LSM303DLHC_Z_SINGLE_CLICK);
}





static ACCELERO_DrvTypeDef *AccelerometerDrv;

uint8_t BSP_ACCELERO_Init(void)
{
  uint8_t ret = ACCELERO_ERROR;
  uint16_t ctrl = 0x0000;
  ACCELERO_InitTypeDef         LSM303DLHC_InitStructure;
  ACCELERO_FilterConfigTypeDef LSM303DLHC_FilterStructure = {0,0,0,0};
  
  if(Lsm303dlhcDrv.ReadID() == I_AM_LMS303DLHC)
  {
		printf("Done\n");
    /* Initialize the Accelerometer driver structure */
    AccelerometerDrv = &Lsm303dlhcDrv;

    /* MEMS configuration ----------------------------------------------------*/
    /* Fill the Accelerometer structure */
    LSM303DLHC_InitStructure.Power_Mode = LSM303DLHC_NORMAL_MODE;
    LSM303DLHC_InitStructure.AccOutput_DataRate = LSM303DLHC_ODR_400_HZ;
    LSM303DLHC_InitStructure.Axes_Enable = LSM303DLHC_AXES_ENABLE;
    LSM303DLHC_InitStructure.AccFull_Scale = LSM303DLHC_FULLSCALE_16G;
    LSM303DLHC_InitStructure.BlockData_Update = LSM303DLHC_BlockUpdate_Continous;
    LSM303DLHC_InitStructure.Endianness = LSM303DLHC_BLE_LSB;
    LSM303DLHC_InitStructure.High_Resolution = LSM303DLHC_HR_ENABLE;
    
    /* Configure MEMS: data rate, power mode, full scale and axes */
    ctrl |= (LSM303DLHC_InitStructure.Power_Mode | LSM303DLHC_InitStructure.AccOutput_DataRate | \
                       LSM303DLHC_InitStructure.Axes_Enable);
    
    ctrl |= ((LSM303DLHC_InitStructure.BlockData_Update | LSM303DLHC_InitStructure.Endianness | \
                      LSM303DLHC_InitStructure.AccFull_Scale | LSM303DLHC_InitStructure.High_Resolution) << 8);
    
    /* Configure the Accelerometer main parameters */
    AccelerometerDrv->Init(ctrl);
    
    /* Fill the Accelerometer LPF structure */
    LSM303DLHC_FilterStructure.HighPassFilter_Mode_Selection =LSM303DLHC_HPM_NORMAL_MODE;
    LSM303DLHC_FilterStructure.HighPassFilter_CutOff_Frequency = LSM303DLHC_HPFCF_8;
    LSM303DLHC_FilterStructure.HighPassFilter_AOI1 = LSM303DLHC_HPF_AOI1_DISABLE;
    LSM303DLHC_FilterStructure.HighPassFilter_AOI2 = LSM303DLHC_HPF_AOI2_DISABLE;
    
    /* Configure MEMS: mode, cutoff frquency, Filter status, Click, AOI1 and AOI2 */
    ctrl = (uint8_t) (LSM303DLHC_FilterStructure.HighPassFilter_Mode_Selection |\
                      LSM303DLHC_FilterStructure.HighPassFilter_CutOff_Frequency|\
                      LSM303DLHC_FilterStructure.HighPassFilter_AOI1|\
                      LSM303DLHC_FilterStructure.HighPassFilter_AOI2);

    /* Configure the Accelerometer LPF main parameters */
    AccelerometerDrv->FilterConfig(ctrl);

    ret = ACCELERO_OK;
  }
  else
  {
    ret = ACCELERO_ERROR;
  }

  return ret;
}

void BSP_ACCELERO_Reset(void)
{
  if(AccelerometerDrv->Reset != NULL)
  {
    AccelerometerDrv->Reset();
  }  
}

void BSP_ACCELERO_Click_ITConfig(void)
{
  if(AccelerometerDrv->ConfigIT!= NULL)
  {
    AccelerometerDrv->ConfigIT();
  }
}

void BSP_ACCELERO_GetXYZ(int16_t *pDataXYZ)
{
  int16_t SwitchXY = 0;
  
  if(AccelerometerDrv->GetXYZ!= NULL)
  {
    AccelerometerDrv->GetXYZ(pDataXYZ);
    
    /* Switch X and Y Axes in case of LSM303DLHC MEMS */
    if(AccelerometerDrv == &Lsm303dlhcDrv)
    { 
      SwitchXY  = pDataXYZ[0];
      pDataXYZ[0] = pDataXYZ[1];
      
      pDataXYZ[1] = -SwitchXY;
    } 
  }
}
