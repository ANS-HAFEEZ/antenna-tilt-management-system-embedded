#include "BLESlave.h"
#include "IMUData.h"

#define CURRENTTIME (app_timer_cnt_get()*6)/100000
extern uint32_t ConnectTime;

#define OP_QUEUES_SIZE          3
#define APP_TIMER_PRESCALER     NRF_SERIAL_APP_TIMER_PRESCALER
extern bool isBLEConnected;
extern bool TimeToSleep;
#define WakeUpPlease 18

static void idle_state_handle(void){
	if(NRF_LOG_PROCESS() == false){
		nrf_pwr_mgmt_run();
	}
}
static char start[]     = "{\"attributes\": {\"device_id\":\"01.0\",\
\"api_key\":\"akS0\",\"lat\":\"";
static char lng[] 		= "\",\"lng\":\"";
static char site_id[]  	= "\",\"site_id\":\"0001\",\"region_id\":\"0234\" },\
\"telemetry\":{\"azimuth\":";
static char tilt[]		= ",\"tilt\":";
static char roll[]		= ",\"roll\":";
static char battery[]	= ",\"battery\":40},\"timestamp\":1568202796";
extern uint16_t   m_conn_handle;
extern bool IsAdvertising;
char ToMater[220]="";

int main(void){
	ret_code_t ret;
	APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
	NRF_LOG_DEFAULT_BACKENDS_INIT();
	APP_ERROR_CHECK(app_timer_init());
	APP_ERROR_CHECK(nrf_pwr_mgmt_init());
	InitIMU();
	InitBLESlave();
	nrf_delay_ms(1000);
	advertise();
	uint32_t SleepTime=0;
	
	int wakeup=0;
	char AntROLL[10]={0};
	char AntPITCH[10]={0};
	char AntYAW[10]={0};

	static float	WatchROLL=0;
	static float WatchPITCH=0;
	static float WatchYAW=0;
	static bool IMUChecked=false;
	static bool ShouldWakeUp=false;
	
	float diffR =0.0;
	float diffP =0.0;
	float diffY =0.0;
	
	int IMUFailCount=0;
	
	while(true){
		if(ShouldWakeUp && !isBLEConnected){
			ShouldWakeUp=false;
			if(!IsAdvertising){ 
				readver();
			} 
			TimeToSleep=false;
			NRF_LOG_INFO("\r\n...............Motion Detected..............\r\n"); NRF_LOG_FLUSH(); 
		}
		
		memset(AntROLL,	0,sizeof(AntROLL));
		memset(AntPITCH,0,sizeof(AntPITCH));
		memset(AntYAW,	0,sizeof(AntYAW));
		if(!isBLEConnected){
			GetOrientation(AntROLL,AntPITCH,AntYAW);
		}
		
		if(strlen(AntROLL)>0 && strlen(AntPITCH)>0 && strlen(AntYAW)>0 ){
			IMUFailCount=0;
			if(!IMUChecked){
				WatchROLL		= strtod(AntROLL,NULL);
				WatchPITCH	= strtod(AntPITCH,NULL);
				WatchYAW		= strtod(AntYAW,NULL);
				IMUChecked=true;
			}
			
			diffR = WatchROLL		- (strtod(AntROLL,NULL));
			diffP = WatchPITCH	- (strtod(AntPITCH,NULL));
			diffY = WatchYAW 		- (strtod(AntYAW,NULL));
					
			if((diffR > 1 || diffR < -1) || (diffP > 1 || diffP < -1)|| (diffY > 3 || diffY < -3) ){
					ShouldWakeUp	= true;
					IMUChecked 		= false;
					NRF_LOG_INFO("RollDiff " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(diffR));
					NRF_LOG_INFO("PollDiff " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(diffP));
					NRF_LOG_INFO("YollDiff " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(diffY));
				}
			sprintf(ToMater,"%s%s%s%s%s%s%s%s%s%s%s}\r\n",
            start, "%s",
            lng, "%s",
            site_id,
            AntYAW,
            tilt,
            AntPITCH,
            roll,
            AntROLL,
            battery);
		}
		else{
			if(IMUFailCount++>3){
				NVIC_SystemReset();
			}
		}
			
		if(CURRENTTIME-ConnectTime>30){	sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);	}
		nrf_delay_ms(200);
	}
}
