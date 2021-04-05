#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "nordic_common.h"
#include "app_error.h"
#include "app_uart.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "app_util.h"

#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "ble_nus_c.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"
#include "nrfx.h"
#include "nrf.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name);
static void nus_error_handler(uint32_t nrf_error);
void scan_start(void);
static void scan_evt_handler(scan_evt_t const * p_scan_evt);
static void scan_init(void);
static void db_disc_handler(ble_db_discovery_evt_t * p_evt);
static void ble_nus_chars_received_uart_print(uint8_t * p_data, uint16_t data_len);
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt);
static bool shutdown_handler(nrf_pwr_mgmt_evt_t event);
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);
static void ble_stack_init(void);
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt);
void gatt_init(void);
static void nus_c_init(void);
static void timer_init(void);
static void log_init(void);
static void power_management_init(void);
static void db_discovery_init(void);
static void idle_state_handle(void);

void InitBLEMaster(void);
void sendBLECMD(void);
