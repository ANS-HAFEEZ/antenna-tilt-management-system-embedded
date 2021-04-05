#ifndef __STM32F401_DISCOVERY_GYROSCOPE_H
#define __STM32F401_DISCOVERY_GYROSCOPE_H

#include "l3gd20.h"   
#include "stm32f0xx_hal.h"
#include "gyroscope.h"   



typedef enum 
{
  GYRO_OK = 0,
  GYRO_ERROR = 1,
  GYRO_TIMEOUT = 2
}GYRO_StatusTypeDef;
uint8_t BSP_GYRO_Init(void);
void    BSP_GYRO_Reset(void);
uint8_t BSP_GYRO_ReadID(void);
void    BSP_GYRO_ITConfig(GYRO_InterruptConfigTypeDef *pIntConfigStruct);
void    BSP_GYRO_EnableIT(uint8_t IntPin);
void    BSP_GYRO_DisableIT(uint8_t IntPin);
void    BSP_GYRO_GetXYZ(float *pfData);


/*################################# SPI1 #####################################*/
#define DISCOVERY_SPIx                          SPI2
#define DISCOVERY_SPIx_CLOCK_ENABLE()           __HAL_RCC_SPI2_CLK_ENABLE()
#define DISCOVERY_SPIx_GPIO_PORT                GPIOB                     /* GPIOA */
#define DISCOVERY_SPIx_AF                       GPIO_AF5_SPI2
#define DISCOVERY_SPIx_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOB_CLK_ENABLE()
#define DISCOVERY_SPIx_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOB_CLK_DISABLE()
#define DISCOVERY_SPIx_SCK_PIN                  GPIO_PIN_13                 /* PA.05 */
#define DISCOVERY_SPIx_MISO_PIN                 GPIO_PIN_14                 /* PA.06 */
#define DISCOVERY_SPIx_MOSI_PIN                 GPIO_PIN_15                 /* PA.07 */

#define SPIx_TIMEOUT_MAX                        ((uint32_t)0x1000)

/*################################ GYROSCOPE #################################*/
/* Read/Write command */
#define READWRITE_CMD                           ((uint8_t)0x80) 
/* Multiple byte read/write command */ 
#define MULTIPLEBYTE_CMD                        ((uint8_t)0x40)
/* Dummy Byte Send by the SPI Master device in order to generate the Clock to the Slave device */
#define DUMMY_BYTE                              ((uint8_t)0x00)

/* Chip Select macro definition */

#define GYRO_CS_GPIO_PORT                       GPIOB                       /* GPIOE */
#define GYRO_CS_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOB_CLK_ENABLE()
#define GYRO_CS_GPIO_CLK_DISABLE()              __HAL_RCC_GPIOB_CLK_DISABLE()
#define GYRO_CS_PIN                             GPIO_PIN_12                  /* PE.03 */

#define GYRO_CS_LOW()       HAL_GPIO_WritePin(GYRO_CS_GPIO_PORT, GYRO_CS_PIN, GPIO_PIN_RESET)
#define GYRO_CS_HIGH()      HAL_GPIO_WritePin(GYRO_CS_GPIO_PORT, GYRO_CS_PIN, GPIO_PIN_SET)

#define GYRO_INT_GPIO_PORT                      GPIOE                       /* GPIOE */
#define GYRO_INT_GPIO_CLK_ENABLE()              __HAL_RCC_GPIOE_CLK_ENABLE()
#define GYRO_INT_GPIO_CLK_DISABLE()             __HAL_RCC_GPIOE_CLK_DISABLE()
#define GYRO_INT1_PIN                           GPIO_PIN_0                  /* PE.00 */
#define GYRO_INT1_EXTI_IRQn                     EXTI0_IRQn 
#define GYRO_INT2_PIN                           GPIO_PIN_1                  /* PE.01 */
#define GYRO_INT2_EXTI_IRQn                     EXTI1_IRQn 

#endif /* __STM32F401_DISCOVERY_GYROSCOPE_H */

