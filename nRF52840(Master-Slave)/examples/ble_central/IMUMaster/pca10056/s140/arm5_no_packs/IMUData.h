#include <stdint.h>
#include <stdbool.h>

#include "app_timer.h"
#include "nrf_drv_spi.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


void InitIMU(void);
void GetOrientation(char *AntROLL,char *AntPITCH,char *AntYAW);
void resetspi(void);
