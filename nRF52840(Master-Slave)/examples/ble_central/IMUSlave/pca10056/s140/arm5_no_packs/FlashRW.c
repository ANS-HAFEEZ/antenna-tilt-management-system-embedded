#include "FlashRW.h"


//// Simple event handler to handle errors during initialization.
//static void fds_evt_handler(fds_evt_t const * p_fds_evt){
//	switch (p_fds_evt->id){
//		case FDS_EVT_INIT:
//			if (p_fds_evt->result != FDS_SUCCESS){
//				// Initialization failed.
//      }
//      break;
//		default:
//			break;
//    }
//}
//ret_code_t ret = fds_register(fds_evt_handler);
//if (ret != FDS_SUCCESS){
//    // Registering of the FDS event handler has failed.
//}
//ret_code_t ret = fds_init();
//if (ret != FDS_SUCCESS){
//    // Handle error.
//}


//#define FILE_ID         0x0001  /* The ID of the file to write the records into. */
//#define RECORD_KEY_1    0x1111  /* A key for the first record. */
//#define RECORD_KEY_2    0x2222  /* A key for the second record. */
//static uint32_t   const m_deadbeef = 0xDEADBEEF;
//static char       const m_hello[]  = "Hello, world!";
//fds_record_t        record;
//fds_record_desc_t   record_desc;
//// Set up record.
//record.file_id           = FILE_ID;
//record.key               = RECORD_KEY_1;
//record.data.p_data       = &m_deadbeef;
//record.data.length_words = 1;   /* one word is four bytes. */
//ret_code_t rc;
//rc = fds_record_write(&record_desc, &record);
//if (rc != FDS_SUCCESS){
//    /* Handle error. */
//}
//// Set up record.
//record.file_id           = FILE_ID;
//record.key               = RECORD_KEY_2;
//record.data.p_data       = &m_hello;
///* The following calculation takes into account any eventual remainder of the division. */
//record.data.length_words = (sizeof(m_hello) + 3) / 4;
//rc = fds_record_write(&record_desc, &record);
//if (rc != FDS_SUCCESS)
//{
//    /* Handle error. */
//}




//#define FILE_ID     0x1111
//#define RECORD_KEY  0x2222
//fds_flash_record_t  flash_record;
//fds_record_desc_t   record_desc;
//fds_find_token_t    ftok;
///* It is required to zero the token before first use. */
//memset(&ftok, 0x00, sizeof(fds_find_token_t));
///* Loop until all records with the given key and file ID have been found. */
//while (fds_record_find(FILE_ID, RECORD_KEY, &record_desc, &ftok) == FDS_SUCCESS){
//    if (fds_record_open(&record_desc, &flash_record) != FDS_SUCCESS)
//    {
//        /* Handle error. */
//    }
//    /* Access the record through the flash_record structure. */
//    /* Close the record when done. */
//    if (fds_record_close(&record_desc) != FDS_SUCCESS)
//    {
//        /* Handle error. */
//    }
//}

///* Assume a descriptor returned by a call to fds_record_write() or fds_record_find(),
//   as shown in the previous example. */
//fds_record_desc_t descriptor;
//ret_code_t ret = fds_record_delete(&descriptor);
//if (ret != FDS_SUCCESS)
//{
//    /* Error. */
//}



#include "BLESlave.h"
#include "IMUData.h"








#include "nrf.h"
#include "nrf_soc.h"
#include "nordic_common.h"
#include "boards.h"
#include "app_timer.h"
#include "app_util.h"
#include "nrf_fstorage.h"

#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_fstorage_sd.h"

#include "nrf_drv_clock.h"
#include "nrf_fstorage_nvmc.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include <stdbool.h>
#include <stdio.h>
#include "nrf.h"
#include "bsp.h"
#include "app_error.h"
#include "nrf_nvmc.h"
#include "nordic_common.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "app_timer.h"
#include "nrf_drv_clock.h"
