#include <stdint.h>

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"
#include "nrf_drv_clock.h"
#include "nrf_fstorage_nvmc.h"


void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name);
static void timers_init(void);
static void gap_params_init(void);
static void nrf_qwr_error_handler(uint32_t nrf_error);
static void nus_data_handler(ble_nus_evt_t * p_evt);
static void services_init(void);
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt);
static void conn_params_error_handler(uint32_t nrf_error);
static void conn_params_init(void);
static void sleep_mode_enter(void);
static void on_adv_evt(ble_adv_evt_t ble_adv_evt);
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);
static void ble_stack_init(void);

void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt);
void gatt_init(void);
//void bsp_event_handler(bsp_event_t event);
void uart_event_handle(app_uart_evt_t * p_event);

static void uart_init(void);

static void advertising_init(void);
static void buttons_leds_init(bool * p_erase_bonds);

static void log_init(void);
static void power_management_init(void);
static void idle_state_handle(void);
static void advertising_start(void);
void InitBLESlave();
void advertise();

void SendOverBLE(char * AntOri, uint16_t DLen);
void readver();
void readID();