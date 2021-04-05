#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>


#include <stdio.h>
#include <math.h>
#include <float.h>
#include <assert.h>

#include "nordic_common.h"
#include "app_error.h"
#include "app_uart.h"
#include "app_timer.h"
#include "app_util.h"
#include "nrf_pwr_mgmt.h"
#include "app_fifo.h"
#include "nrfx.h"
#include "nrf_drv_uart.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_serial.h"
#include "boards.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


bool Bringup_Network(void);
bool MQTT_Start(void);
bool StartModem(void);
bool MQTT_Connect(void);
bool StartModem(void);
bool MQTTPub(const char *topic,const char *msg);
void InitGPS(void);
bool GetLoc(void);
void InitModemPort(void);
bool ReadModemResponse(uint32_t tout);
bool CMDSend( char * cmd);
bool CMDnWAIT( char *CMD, char *CMD_R,int TimeOut);
bool MQTT_Stop(void);
void flushUART(void);
void DisconnectData(void);
