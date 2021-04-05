#include <stdint.h>
#include <stdbool.h>

#include "app_timer.h"
#include "nrf_delay.h"

#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_drv_spi.h"
#include "app_error.h"
#include <string.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
void InitIMU();
void GetOrientation(char *AntROLL,char *AntPITCH,char *AntYAW);
void SendRequest(char *RX);