#include "IMUData.h"

#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"

#define CURRENTTIME (app_timer_cnt_get()*6)/100000
// ALL SPI Here

#define SPI_MOSI_PIN 33
#define SPI_MISO_PIN 34
#define SPI_SCK_PIN 35

#define SPI_INSTANCE 0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);/**< SPIS instance. */

#define TEST_STRING "\nOKO\n\0"
static char       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
static char       m_rx_buf[40]={0};    /**< RX buffer. */

static volatile bool spis_xfer_done; /**< Flag used to indicate that SPIS instance completed the transfer. */

bool spi_xfer_done =true;

#define ShowIMU 1
void spi_event_handler(nrf_drv_spi_evt_t const * p_event, void *p_context){
    spi_xfer_done = true;
}

int wait=0;
void InitIMU(){
	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
  spi_config.miso_pin 	= SPI_MISO_PIN;
  spi_config.mosi_pin 	= SPI_MOSI_PIN;
  spi_config.sck_pin  	= SPI_SCK_PIN;
	spi_config.frequency 	= NRF_DRV_SPI_FREQ_125K;
	spi_config.mode				= NRF_DRV_SPI_MODE_0;
  APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
	
	//nrf_drv_spi_transfer(&spi,(uint8_t *)m_tx_buf, sizeof(m_tx_buf), (uint8_t *)m_rx_buf, 40);
}	

void GetOrientation(char *AntROLL,char *AntPITCH,char *AntYAW){
	uint32_t SpiTime = CURRENTTIME;
	memset(m_rx_buf, 0, sizeof(m_rx_buf));
	if(spi_xfer_done){
		spi_xfer_done = false;
		nrf_drv_spi_transfer(&spi,(uint8_t *)m_tx_buf, 3, (uint8_t *)m_rx_buf,40);
		nrf_delay_ms(10);
	}
	else{
		wait++;
		if(wait>20){
			wait=0;
			spi_xfer_done = true;
		}
	}
	
	if(strstr(m_rx_buf,"R") && strstr(m_rx_buf,"P")&& strstr(m_rx_buf,"Y")){
		wait=0;
		//NRF_LOG_INFO(" Orientation:  %s",m_rx_buf); NRF_LOG_FLUSH();
		for(int i =0; i< strlen(m_rx_buf);i++){
				int ind=0;
				if(m_rx_buf[i]=='R'){
					for(int j =0; j< 9;j++){
						if(m_rx_buf[i+j]==',')
								break;
						if(m_rx_buf[i+j] > 0x2C && m_rx_buf[i+j]<0x3A){
							AntROLL[ind++]= m_rx_buf[i+j];
						}
					}
					//NRF_LOG_INFO("ROLL: %s",AntROLL); NRF_LOG_FLUSH(); 
				}
				
				ind=0;
				if(m_rx_buf[i]=='P'){
					for(int j =0; j< 9;j++){
						if(m_rx_buf[i+j]==',')
								break;
						if(m_rx_buf[i+j] > 0x2C && m_rx_buf[i+j]<0x3A){
							AntPITCH[ind++]= m_rx_buf[i+j];
						}
					}
					//NRF_LOG_INFO("Pitch: %s",AntPITCH); NRF_LOG_FLUSH(); 
	
				}
				ind=0;
				if(m_rx_buf[i]=='Y'){
					for(int j =0; j< 9;j++){
						if(m_rx_buf[i+j]==',')
								break;
						if(m_rx_buf[i+j] > 0x2C && m_rx_buf[i+j]<0x3A){
							AntYAW[ind++]= m_rx_buf[i+j];
						}
					}
					//NRF_LOG_INFO("R: %s P: %s Y: %s",AntROLL,AntPITCH,AntYAW); NRF_LOG_FLUSH();
				}
			}
		}
}

void resetspi(){
	nrf_spi_disable(spi.u.spi.p_reg);
	nrf_spi_int_disable(spi.u.spi.p_reg,SPIM_INTENSET_STOPPED_Msk);
	nrf_delay_ms(100);
	nrf_spi_int_enable(spi.u.spi.p_reg,NRF_SPIM_INT_STARTED_MASK);
	nrf_spi_enable(spi.u.spi.p_reg);
}

void SendRequest(char *RX){
////nrf_spi_txd_set(;
////nrf_spi_rxd_get(spi.u.spi.p_reg);	
//	//nrf_spi_enable(spi.u.spi.p_reg);
//	spi_xfer_done = false;
//	//nrf_drv_spi_transfer(&spi,(uint8_t *)"/r/ngive/r/n", sizeof("/r/ngive/r/n"), (uint8_t *)RX, sizeof(RX));
//	nrf_drv_spi_transfer(&spi,(uint8_t *)m_tx_buf, sizeof(m_tx_buf), (uint8_t *)RX, 40);
//	//nrf_spi_disable(spi.u.spi.p_reg);
//	//nrf_gpio_pin_clear(SPI_SCK_PIN);
//	nrf_delay_ms(200);
}
