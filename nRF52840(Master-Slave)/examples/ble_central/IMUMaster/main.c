#include "BLEMaster.h"
#include "IMUData.h"
#include "Modem.h"

#define CURRENTTIME (app_timer_cnt_get()*6)/100000

static void log_init(void){
	ret_code_t err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);
	NRF_LOG_DEFAULT_BACKENDS_INIT();
}

#define EnableModem
extern bool network_flag;
extern char GPSLAT[15];
extern char GPSLNG[15];
extern bool ISBLEDataReceived;
extern char S_ANT[220];
extern bool IsBLEConnected;

static char start[]     = "{\"attributes\": {\"device_id\":\"0001\",\
\"api_key\":\"akM1\",\"lat\":\"";
static char lng[] 		= "\",\"lng\":\"";
static char site_id[]  	= "\",\"site_id\":\"0001\",\"region_id\":\"0234\" },\
\"telemetry\":{\"azimuth\":";
static char tilt[]		= ",\"tilt\":";
static char roll[]		= ",\"roll\":";
static char battery[]	= ",\"battery\":40},\"timestamp\":1568202796";
  
char ToServer[280]="";
const char *MQTTTopic = "iot_platform-iot.agents.event_tenant1/events";
static bool MDone = false;
bool GPSOK	=false;
extern bool ISScanning;
static char DumLAT[] = "0000.000000";
static char DumLNG[] = "00000.000000";

int main(void){
	log_init();
  nrf_drv_clock_init();
  nrf_drv_clock_lfclk_request(NULL);
	app_timer_init();

	InitIMU();
	nrf_delay_ms(100);
	InitBLEMaster();

#ifdef EnableModem	
	InitModemPort();
	StartModem();
	Bringup_Network();
	InitGPS();
#endif	
	
	static char AntROLL[10]	= {0};
	static char AntPITCH[10]= {0};
	static char AntYAW[10]	= {0};
	static float	WatchROLL		= 0;
	static float 	WatchPITCH	= 0;
	static float 	WatchYAW		= 0;
	static bool IMUChecked	= false;
	static bool ShouldWakeUp= false;
	
	float diffR =0.0;
	float diffP =0.0;
	float diffY =0.0;
	
	int IMUFailCount=0;
	/*main Loop Started Here*/
	bool CheckGPS =true; 
	int IMUSPIFAIL=0;
	scan_start();
	int showFlags=0;
	while(true){
		if(!ISScanning){
			scan_start();
		}
		if(showFlags++>10){
			NRF_LOG_INFO("\rBLEConnected %d : ShouldWakeUp %d : IsScanning %d",IsBLEConnected,ShouldWakeUp,ISScanning); NRF_LOG_FLUSH(); 
			showFlags=0;
		}
			
#ifdef EnableModem			
		if(!IsBLEConnected && CheckGPS && !ShouldWakeUp){ 
			if(!GetLoc()){
				strcpy(GPSLAT,DumLAT); strcpy(GPSLNG,DumLNG);	
			}
			else{
				CheckGPS=false;
			}
		}
#endif		
		if(ShouldWakeUp && !IsBLEConnected){
			ShouldWakeUp=false;
			if(ISScanning){
				sd_ble_gap_scan_stop();
				nrf_delay_ms(100);
				ISScanning=false;
			}
			MDone= false;
		}

		memset(ToServer,0,sizeof(ToServer));
		memset(AntROLL,	0,sizeof(AntROLL));
		memset(AntPITCH,0,sizeof(ToServer));
		memset(AntYAW,	0,sizeof(AntYAW));
	
		while(true){
			GetOrientation(AntROLL,AntPITCH,AntYAW);
			if(strlen(AntROLL) > 0 && strlen(AntPITCH) > 0 && strlen(AntYAW) > 0 ){
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
				//NRF_LOG_INFO("Orientation is -----> R: %s P: %s Y: %s",AntROLL,AntPITCH,AntYAW); NRF_LOG_FLUSH();
				break;
			}
			else{
				if(IMUSPIFAIL++>10){
					if(IMUFailCount++>3){
						NVIC_SystemReset();
					}
					IMUSPIFAIL=0;	
					break;
				}
			}
			nrf_delay_ms(100);	
		}
		sprintf(ToServer,"%s%s%s%s%s%s%s%s%s%s%s}\n",start,GPSLAT,lng,GPSLNG,site_id,AntYAW,tilt,AntPITCH,roll,AntROLL,battery);
#ifdef EnableModem	
		if(!MDone && !IsBLEConnected && !ISScanning){
			if(MQTT_Connect()){
				if(!MQTTPub(MQTTTopic,ToServer)){}
				else{
				MDone= true;
				NRF_LOG_INFO("Master Succesffully Send data to server\n"); NRF_LOG_FLUSH();
			}
		}
	}
#endif		
		if(IsBLEConnected){	
			sendBLECMD();	
		}
#ifdef EnableModem			
		if(strlen(S_ANT)>150 && !IsBLEConnected){
			sprintf(ToServer,S_ANT,GPSLAT,GPSLNG);
			if(MQTT_Connect()){
				if(!MQTTPub(MQTTTopic,ToServer)){}
				else{
				NRF_LOG_INFO("Slave Succesffully Send data to server\n"); NRF_LOG_FLUSH();
				memset(S_ANT,0,sizeof(S_ANT));
				}
			}
		}
#endif	
		if(IsBLEConnected){
			nrf_delay_ms(2000);
		}
	nrf_delay_ms(200);
	}
}
