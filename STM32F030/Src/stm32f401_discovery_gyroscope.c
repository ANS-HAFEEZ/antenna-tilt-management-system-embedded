#include "gyroscope.h"
#include "l3gd20.h"

uint32_t SpixTimeout = SPIx_TIMEOUT_MAX;    /*<! Value of Timeout when SPI communication fails */
I2C_HandleTypeDef I2cHandle;
SPI_HandleTypeDef SpiHandle;

/* SPIx bus function */
static void    SPIx_Init(void);
static uint8_t SPIx_WriteRead(uint8_t byte);
static void    SPIx_Error (void);
static void    SPIx_MspInit(SPI_HandleTypeDef *hspi);

/* Link function for GYRO peripheral */
void GYRO_IO_Init(void);
void GYRO_IO_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
void GYRO_IO_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);

static void SPIx_Init(void)
{
  if(HAL_SPI_GetState(&SpiHandle) == HAL_SPI_STATE_RESET)
  {
//		printf("Setting GYRO SPI\n");
//    /* SPI Configuration */
    SpiHandle.Instance = SPI2;
    
    SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    SpiHandle.Init.Direction = SPI_DIRECTION_2LINES;
    SpiHandle.Init.CLKPhase = SPI_PHASE_1EDGE;
    SpiHandle.Init.CLKPolarity = SPI_POLARITY_LOW;
    SpiHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    SpiHandle.Init.CRCPolynomial = 7;
    SpiHandle.Init.DataSize = SPI_DATASIZE_8BIT;
    SpiHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    SpiHandle.Init.NSS = SPI_NSS_SOFT;
    SpiHandle.Init.TIMode = SPI_TIMODE_DISABLE;
		SpiHandle.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    SpiHandle.Init.Mode = SPI_MODE_MASTER;
    
		SPIx_MspInit(&SpiHandle);
		HAL_SPI_Init(&SpiHandle);
  }
}

static uint8_t SPIx_WriteRead(uint8_t Byte)
{
  uint8_t receivedbyte = 0;
  
  if(HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*) &Byte, (uint8_t*) &receivedbyte, 1, SpixTimeout) != HAL_OK)
  {
    SPIx_Error();
  }
  
  return receivedbyte;
}

/**
  * @brief  SPIx error treatment function.
  */
static void SPIx_Error (void)
{
  /* De-initialize the SPI comunication BUS */
  HAL_SPI_DeInit(&SpiHandle);
  
  /* Re-Initiaize the SPI comunication BUS */
  SPIx_Init();
}

/**
  * @brief  SPI MSP Init.
  * @param  hspi: SPI handle
  */
static void SPIx_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef   GPIO_InitStructure;
  
  /* Enable SPIx clock  */
  DISCOVERY_SPIx_CLOCK_ENABLE();
  
  /* Enable SPIx GPIO clock */
  DISCOVERY_SPIx_GPIO_CLK_ENABLE();
  
  /* Configure SPIx SCK, MOSI and MISO */
  GPIO_InitStructure.Pin = (DISCOVERY_SPIx_SCK_PIN | DISCOVERY_SPIx_MOSI_PIN | DISCOVERY_SPIx_MISO_PIN);
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Alternate = GPIO_AF0_SPI2;
  HAL_GPIO_Init(DISCOVERY_SPIx_GPIO_PORT, &GPIO_InitStructure);
}

void GYRO_IO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Configure the Gyroscope Control pins ------------------------------------*/
  /* Enable CS GPIO clock and  Configure GPIO PIN for Gyroscope Chip select */  
  
	GYRO_CS_GPIO_CLK_ENABLE();  
  GPIO_InitStructure.Pin = GYRO_CS_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GYRO_CS_GPIO_PORT, &GPIO_InitStructure);

  /* Deselect : Chip Select high */
	//GYRO_CS_HIGH();

  /* Enable INT1, INT2 GPIO clock and Configure GPIO PINs to detect Interrupts */
//  GYRO_INT_GPIO_CLK_ENABLE();
//  GPIO_InitStructure.Pin = GYRO_INT1_PIN | GYRO_INT2_PIN;
//  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
//  GPIO_InitStructure.Pull= GPIO_NOPULL;
//  HAL_GPIO_Init(GYRO_INT_GPIO_PORT, &GPIO_InitStructure);
  
  SPIx_Init();
}

void GYRO_IO_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
  if(NumByteToWrite > 0x01)
  {
    WriteAddr |= (uint8_t)MULTIPLEBYTE_CMD;
  }
  /* Set chip select Low at the start of the transmission */
  GYRO_CS_LOW();
  
  /* Send the Address of the indexed register */
  SPIx_WriteRead(WriteAddr);
  
  /* Send the data that will be written into the device (MSB First) */
  while(NumByteToWrite >= 0x01)
  {
    SPIx_WriteRead(*pBuffer);
    NumByteToWrite--;
    pBuffer++;
  }
  
  /* Set chip select High at the end of the transmission */ 
  GYRO_CS_HIGH();
}

void GYRO_IO_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{  
  if(NumByteToRead > 0x01)
  {
    ReadAddr |= (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD);
  }
  else
  {
    ReadAddr |= (uint8_t)READWRITE_CMD;
  }
  
  /* Set chip select Low at the start of the transmission */
  GYRO_CS_LOW();
  
  /* Send the Address of the indexed register */
  SPIx_WriteRead(ReadAddr);
  
  /* Receive the data that will be read from the device (MSB First) */
  while(NumByteToRead > 0x00)
  {
    /* Send dummy byte (0x00) to generate the SPI clock to GYRO (Slave device) */
    *pBuffer = SPIx_WriteRead(DUMMY_BYTE);
    NumByteToRead--;
    pBuffer++;
  }
  
  /* Set chip select High at the end of the transmission */ 
  GYRO_CS_HIGH();
}  


GYRO_DrvTypeDef L3gd20Drv =
{
  L3GD20_Init,
  L3GD20_DeInit,
  L3GD20_ReadID,
  L3GD20_RebootCmd,
  L3GD20_LowPower,
  L3GD20_INT1InterruptConfig,
  L3GD20_EnableIT,
  L3GD20_DisableIT,
  0,
  0,
  L3GD20_FilterConfig,
  L3GD20_FilterCmd,
//  L3GD20_ReadXYZAngRate
};

void L3GD20_Init(uint16_t InitStruct){  
  uint8_t ctrl = 0x00;
 
  GYRO_IO_Init();
	
  ctrl = (uint8_t) InitStruct;
	printf("Setting Gyro Reg1 -> %02X\n",ctrl );
  GYRO_IO_Write(&ctrl, L3GD20_CTRL_REG1_ADDR, 1);
  
  ctrl = (uint8_t) (InitStruct >> 8);
	printf("Setting Gyro Reg4 -> %02X\n",ctrl );
  GYRO_IO_Write(&ctrl, L3GD20_CTRL_REG4_ADDR, 1);
}

void L3GD20_DeInit(void){}

uint8_t L3GD20_ReadID(void){
  uint8_t tmp;
  GYRO_IO_Init();
  GYRO_IO_Read(&tmp, L3GD20_WHO_AM_I_ADDR, 1);
  //printf("\nReading Gyro ID .......   %X \n\n",tmp);
  return (uint8_t)tmp;
}

void L3GD20_RebootCmd(void)
{
  uint8_t tmpreg;
  
  GYRO_IO_Read(&tmpreg, L3GD20_CTRL_REG5_ADDR, 1);
  
  tmpreg |= L3GD20_BOOT_REBOOTMEMORY;
  
  GYRO_IO_Write(&tmpreg, L3GD20_CTRL_REG5_ADDR, 1);
}

void L3GD20_LowPower(uint16_t InitStruct)
{  
  uint8_t ctrl = 0x00;

  ctrl = (uint8_t) InitStruct;
  GYRO_IO_Write(&ctrl, L3GD20_CTRL_REG1_ADDR, 1);
}

void L3GD20_INT1InterruptConfig(uint16_t Int1Config)
{
  uint8_t ctrl_cfr = 0x00, ctrl3 = 0x00;
  
  /* Read INT1_CFG register */
  GYRO_IO_Read(&ctrl_cfr, L3GD20_INT1_CFG_ADDR, 1);
  
  /* Read CTRL_REG3 register */
  GYRO_IO_Read(&ctrl3, L3GD20_CTRL_REG3_ADDR, 1);
  
  ctrl_cfr &= 0x80;
  ctrl_cfr |= ((uint8_t) Int1Config >> 8);
  
  ctrl3 &= 0xDF;
  ctrl3 |= ((uint8_t) Int1Config);   
  
  /* Write value to MEMS INT1_CFG register */
  GYRO_IO_Write(&ctrl_cfr, L3GD20_INT1_CFG_ADDR, 1);
  
  /* Write value to MEMS CTRL_REG3 register */
  GYRO_IO_Write(&ctrl3, L3GD20_CTRL_REG3_ADDR, 1);
}

void L3GD20_EnableIT(uint8_t IntSel)
{  
  uint8_t tmpreg;
  
  /* Read CTRL_REG3 register */
  GYRO_IO_Read(&tmpreg, L3GD20_CTRL_REG3_ADDR, 1);
  
  if(IntSel == L3GD20_INT1)
  {
    tmpreg &= 0x7F; 
    tmpreg |= L3GD20_INT1INTERRUPT_ENABLE;
  }
  else if(IntSel == L3GD20_INT2)
  {
    tmpreg &= 0xF7;
    tmpreg |= L3GD20_INT2INTERRUPT_ENABLE;
  }
  
  /* Write value to MEMS CTRL_REG3 register */
  GYRO_IO_Write(&tmpreg, L3GD20_CTRL_REG3_ADDR, 1);
}

void L3GD20_DisableIT(uint8_t IntSel)
{  
  uint8_t tmpreg;
  
  /* Read CTRL_REG3 register */
  GYRO_IO_Read(&tmpreg, L3GD20_CTRL_REG3_ADDR, 1);
  
  if(IntSel == L3GD20_INT1)
  {
    tmpreg &= 0x7F; 
    tmpreg |= L3GD20_INT1INTERRUPT_DISABLE;
  }
  else if(IntSel == L3GD20_INT2)
  {
    tmpreg &= 0xF7;
    tmpreg |= L3GD20_INT2INTERRUPT_DISABLE;
  }
  
  /* Write value to MEMS CTRL_REG3 register */
  GYRO_IO_Write(&tmpreg, L3GD20_CTRL_REG3_ADDR, 1);
}

void L3GD20_FilterConfig(uint8_t FilterStruct) 
{
  uint8_t tmpreg;
  
  /* Read CTRL_REG2 register */
  GYRO_IO_Read(&tmpreg, L3GD20_CTRL_REG2_ADDR, 1);
  
  tmpreg &= 0xC0;
  
  /* Configure MEMS: mode and cutoff frequency */
  tmpreg |= FilterStruct;
  
  /* Write value to MEMS CTRL_REG2 register */
  GYRO_IO_Write(&tmpreg, L3GD20_CTRL_REG2_ADDR, 1);
}

void L3GD20_FilterCmd(uint8_t HighPassFilterState)
{
  uint8_t tmpreg;
  
  GYRO_IO_Read(&tmpreg, L3GD20_CTRL_REG5_ADDR, 1);
  
  tmpreg &= 0xEF;
  
  tmpreg |= HighPassFilterState;
  
  GYRO_IO_Write(&tmpreg, L3GD20_CTRL_REG5_ADDR, 1);
}

uint8_t L3GD20_GetDataStatus(void)
{
  uint8_t tmpreg;
  GYRO_IO_Read(&tmpreg, L3GD20_STATUS_REG_ADDR, 1);
  return tmpreg;
}

void GyroGetRate(int16_t *pfData)
{
  uint8_t tmpbuffer[6] ={0};
  int16_t RawData[3] = {0};
  uint8_t tmpreg = 0;
  int i =0;
  
  GYRO_IO_Read(&tmpreg,L3GD20_CTRL_REG4_ADDR,1);
  GYRO_IO_Read(tmpbuffer,L3GD20_OUT_X_L_ADDR,6);
  
	for(i=0; i<3; i++)
  {
		RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i+1] << 8) + tmpbuffer[2*i]);
	}
 
  for(i=0; i<3; i++)
  {
    pfData[i]=(float)(RawData[i]);// * sensitivity);
  }
}


static GYRO_DrvTypeDef *GyroscopeDrv;
uint8_t BSP_GYRO_Init(void)
{  
  uint8_t ret = GYRO_ERROR;
  uint16_t ctrl = 0x0000;
  GYRO_InitTypeDef         L3GD20_InitStructure;
  GYRO_FilterConfigTypeDef L3GD20_FilterStructure = {0,0};
 
	//printf("Checking Gyro........................\n");
  if((L3gd20Drv.ReadID() == I_AM_L3GD20)){
		printf("Gyro Initilized\n");
    GyroscopeDrv = &L3gd20Drv;
    L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
    L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_2;
    L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
    L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_4;
    L3GD20_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
    L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB;
    L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_2000; 
  
    ctrl = (uint16_t) (L3GD20_InitStructure.Power_Mode | L3GD20_InitStructure.Output_DataRate | L3GD20_InitStructure.Axes_Enable | L3GD20_InitStructure.Band_Width);
  
    ctrl |= (uint16_t) ((L3GD20_InitStructure.BlockData_Update | L3GD20_InitStructure.Endianness | L3GD20_InitStructure.Full_Scale) << 8);

    GyroscopeDrv->Init(ctrl);
  
    L3GD20_FilterStructure.HighPassFilter_Mode_Selection =L3GD20_HPM_NORMAL_MODE;
    L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_1;
  
    ctrl = (uint8_t) ((L3GD20_FilterStructure.HighPassFilter_Mode_Selection | L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency));    

    GyroscopeDrv->FilterConfig(ctrl) ;
  
    GyroscopeDrv->FilterCmd(L3GD20_HIGHPASSFILTER_ENABLE);
  
    ret = GYRO_OK;
  }
	else{
		printf("Gyro Failed ...... :(\n");
	}
  return ret;
}

uint8_t BSP_GYRO_ReadID(void)
{
  uint8_t id = 0x00;
  if(GyroscopeDrv->ReadID != NULL){
    id = GyroscopeDrv->ReadID();
  }  
  return id;
}

void BSP_GYRO_Reset(void)
{  
  if(GyroscopeDrv->Reset != NULL)
  {
    GyroscopeDrv->Reset();
  }  
}

void BSP_GYRO_ITConfig(GYRO_InterruptConfigTypeDef *pIntConfig)
{
  uint16_t interruptconfig = 0x0000;
  
  if(GyroscopeDrv->ConfigIT != NULL)
  {
    /* Configure latch Interrupt request and axe interrupts */                   
    interruptconfig |= ((uint8_t)(pIntConfig->Latch_Request| \
                                  pIntConfig->Interrupt_Axes) << 8);
    
    interruptconfig |= (uint8_t)(pIntConfig->Interrupt_ActiveEdge);
    
    GyroscopeDrv->ConfigIT(interruptconfig);
  }  
}

void BSP_GYRO_EnableIT(uint8_t IntPin)
{
  if(GyroscopeDrv->EnableIT != NULL)
  {
    GyroscopeDrv->EnableIT(IntPin);
  }
}

void BSP_GYRO_DisableIT(uint8_t IntPin)
{
  if(GyroscopeDrv->DisableIT != NULL)
  {
    GyroscopeDrv->DisableIT(IntPin);
  }
}
  
void BSP_GYRO_GetXYZ(float *pfData)
{
  if(GyroscopeDrv->GetXYZ!= NULL)
  {   
    GyroscopeDrv->GetXYZ(pfData);
  }
} 
