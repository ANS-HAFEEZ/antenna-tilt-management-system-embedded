//ModemOK2
#include "Modem.h"

#define CURRENTTIME (app_timer_cnt_get()*6)/100000
#define ModemLogs 1

//#define ShowPub 1
#define ShowCMD 1

static nrf_drv_uart_t Modem_Ins = NRF_DRV_UART_INSTANCE(1);
nrf_drv_uart_config_t Modem_config = NRF_DRV_UART_DEFAULT_CONFIG;
static nrf_uart_event_handler_t   Modem_handler;            

#define OP_QUEUES_SIZE          3
#define APP_TIMER_PRESCALER     NRF_SERIAL_APP_TIMER_PRESCALER

static void sleep_handler(void){
    __WFE();
    __SEV();
    __WFE();
}

#define SERIAL_FIFO_TX_SIZE 128
#define SERIAL_FIFO_RX_SIZE 8128
#define SERIAL_BUFF_TX_SIZE 20
#define SERIAL_BUFF_RX_SIZE 20

//NRF_SERIAL_DRV_UART_CONFIG_DEF(Modem_Config, 28, 29, RTS_PIN_NUMBER, CTS_PIN_NUMBER,NRF_UART_HWFC_DISABLED, NRF_UART_PARITY_EXCLUDED, NRF_UART_BAUDRATE_115200, UART_DEFAULT_CONFIG_IRQ_PRIORITY);
NRF_SERIAL_DRV_UART_CONFIG_DEF(Modem_Config, 5, 4, RTS_PIN_NUMBER, CTS_PIN_NUMBER,NRF_UART_HWFC_DISABLED, NRF_UART_PARITY_EXCLUDED, NRF_UART_BAUDRATE_115200, UART_DEFAULT_CONFIG_IRQ_PRIORITY);

NRF_SERIAL_QUEUES_DEF(serial0_queues, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE);
NRF_SERIAL_BUFFERS_DEF(serial0_buffs, SERIAL_BUFF_TX_SIZE, SERIAL_BUFF_RX_SIZE);
NRF_SERIAL_CONFIG_DEF(serial0_config, NRF_SERIAL_MODE_DMA, &serial0_queues, &serial0_buffs, NULL, sleep_handler);
NRF_SERIAL_UART_DEF(Modem_Uart, 0);

char* cmd;
uint32_t CommandTime;
uint32_t WaitingTime;
bool CommandSuccess;
static char DataFromModem[300] = {0};
int simrestart=0;
bool network_flag = false;
char GPSData[35]= {0};

//BringUpCommands   
static char AT[] = "\r\nAT\r\n";
static char CRESET[] = "\r\nAT+CRESET\r\n";

static char ATE[]  = "\r\nATE1\r\n";
static char CPSI[] = "\r\nAT+CPSI?\r\n";
static char CREG[] = "\r\nAT+CREG?\r\n";
static char CGREG[] = "\r\nAT+CGREG?\r\n";

static char CGSOCKCONT[] = "\r\nAT+CGSOCKCONT=1,\"IP\",\"zonginternet\"\r\n";
static char CIPMODE[] = "\r\nAT+CIPMODE=0\r\n";
static char NETOPEN[] = "\r\nAT+NETOPEN\r\n";
static char IPADDR[] = "\r\nAT+IPADDR\r\n";

//static char CCLK[]= "\r\nAT+CCLK?\r\n";
//static char CCLK_R[]= "+CCLK:";

//static char CLTS[]= "\r\nAT+CLTS=1\r\n";
//static char CLTS_R[]= "+CCLK:";

//static char CPING[]= "\r\nAT+CPING=\"www.baidu.com\",1,4,64,1000,10000,255\r\n";
//static char CPING_R[]= "+CPING:";
//BringUpCommands Responses
static char AT_R[] = "OK";
static char CRESET_R[] = "OK";

static char ATE_R[]  = "OK";
static char CPSI_R[] = "WCDMA";
static char CREG_R[] = "+CREG: 0,1";
static char CGREG_R[] = "+CGREG: 0,1";

static char CGSOCKCONT_R[] = "OK";
static char CIPMODE_R[] = "OK";
static char NETOPEN_R[] = "+NETOPEN: 0";
static char IPADDR_R[] = "+IPADDR:";

static char NETCLOSE[] = "\r\nAT+NETCLOSE\r\n";
static char NETCLOSE_R[] = "+NETCLOSE: 0";
//////////////////MQTT Commands//////////////////
static char CMQTTSTART[]  = "\r\nAT+CMQTTSTART\r\n";
static char CMQTTACCQ[] = "\r\nAT+CMQTTACCQ=0,\"ATMS1\"\r\n";
static char CMQTTCONNECT[] = "\r\nAT+CMQTTCONNECT=0,\"tcp://18.217.131.166:1883\",20,1,\"admin\",\"admin\"\r\n";
static char CMQTTPUB[]  = "\r\nAT+CMQTTPUB=0,1,60\r\n";

static char CMQTTDISC[] = "\r\nAT+CMQTTDISC=0,120\r\n";
static char CMQTTREL[]  =   "\r\nAT+CMQTTREL=0\r\n";
static char CMQTTSTOP[] = "\r\nAT+CMQTTSTOP\r\n";

static char CMQTTDISC_R[] = "+CMQTTDISC: 0,0";
static char CMQTTREL_R[]    =   "OK";
static char CMQTTSTOP_R[]   = "+CMQTTSTOP: 0";


//MQTT Commands Responses
static char CMQTTSTART_R[]  = "+CMQTTSTART: 0";
static char CMQTTACCQ_R[] = "OK";
static char CMQTTCONNECT_R[] = "+CMQTTCONNECT: 0,0";
static char CMQTTPUB_R[] = "QTTPUB: 0,0";

//GPS Commands
static char CGPS[] = "\r\nAT+CGPS=1\r\n";
static char CGPSNMEA[] = "\r\nAT+CGPSNMEA=0\r\n";
static char CGPSINFO[] = "\r\nAT+CGPSINFO\r\n";  

static char CGPS_R[] = "OK";
static char CGPSNMEA_R[] = "OK";
static char CGPSINFO_R[] = "+CGPSINFO";  


int CommandFailCount=0;
bool CMDnWAIT(char *CMD,char *CMD_R,int TimeOut){
	if(CMDSend(CMD)){
		WaitingTime = CURRENTTIME;
    ReadModemResponse(1);
    //NRF_LOG_INFO("Start CMD Response: %s\n",DataFromModem); NRF_LOG_FLUSH();
    if(strstr(DataFromModem,"OK")){ CommandFailCount=0;}
			while(!strstr(DataFromModem,CMD_R)){
				ReadModemResponse(3);
        if(CURRENTTIME - WaitingTime>TimeOut){
					//NRF_LOG_INFO("Command TimeOut\n"); NRF_LOG_FLUSH();
          if(CommandFailCount++>3){
						NVIC_SystemReset();
          }
          return false;
        }
			}
		if(strstr(DataFromModem,CGPSINFO_R)){
			if(strlen(DataFromModem)>30){
				return true; 
			}
			else{
				return false;
			}
    }
		//NRF_LOG_INFO("Data From Modem: %s\n",DataFromModem); NRF_LOG_FLUSH();
    return true; 
	}
   NRF_LOG_INFO("Sending Failed on TX : %s\n",CMD); NRF_LOG_FLUSH();
   return false;
}


bool Bringup_Network(void){
	if(!CMDnWAIT(ATE,ATE_R    	,10)){
		nrf_delay_ms(50); return false;   
	}
  if(!CMDnWAIT(CPSI,CPSI_R    ,20)){nrf_delay_ms(50); return false;   }
  if(!CMDnWAIT(CREG,CREG_R    ,20)){nrf_delay_ms(50); return false;   }
  if(!CMDnWAIT(CGREG,CGREG_R  ,20)){nrf_delay_ms(50); return false;   }
  else{ return true;    }
}

//////////NetworkFlags////////////
bool IS_CGSOCKCONT=false;
bool IS_CIPMODE=false;
bool IS_NETOPEN=false;
bool IS_IPADDR=false;
bool IS_CMQTTSTART=false;
bool IS_CMQTTACCQ=false;
bool IS_CMQTTCONNECT=false;
bool MQTT_Connect(){
	if(CMDnWAIT(CGSOCKCONT,CGSOCKCONT_R		, 7)){
		IS_CGSOCKCONT=true;
	}
	else{
		if(IS_CGSOCKCONT){
		
		}
		else{
			return false;   
		}
	}
	nrf_delay_ms(200);
	
  if(CMDnWAIT(CIPMODE,CIPMODE_R        ,5)){
		IS_CIPMODE = true;
	}
	else{
		if(IS_CIPMODE){
		
		}
		else{
			return false;
		}
	}
	nrf_delay_ms(200);
  
	if(CMDnWAIT(NETOPEN,NETOPEN_R        ,10)){
		IS_NETOPEN=true;
	}
	else{
		if(IS_NETOPEN){
		
		}
		else{
			return false;
		}
	}
	nrf_delay_ms(200);
	
  if(!CMDnWAIT(IPADDR,IPADDR_R          ,10)){
		nrf_delay_ms(50); return false;   
	}
	
	if(CMDnWAIT(CMQTTSTART,CMQTTSTART_R  ,10)){
		IS_CMQTTSTART=true;
	}
	else{
		if(IS_CMQTTSTART){
		}
		else{
			return false;
		}
	}
	nrf_delay_ms(200);
	
  if(CMDnWAIT(CMQTTACCQ,CMQTTACCQ_R    ,10)){
		IS_CMQTTACCQ =true;
	}
	else{
		if(IS_CMQTTACCQ){
		}
		else{
			return false;
		}
	}
	nrf_delay_ms(200);
	if(CMDnWAIT(CMQTTCONNECT,CMQTTCONNECT_R,10)){
		IS_CGSOCKCONT=true;
	}
	else{
		if(IS_CGSOCKCONT){
		}
		else{
			return false;
		}
	}
	return true;
}

bool MQTT_Stop(void){
	if(!CMDnWAIT(CMQTTDISC	,CMQTTDISC_R ,10)){
		//return false;  
	}
	if(!CMDnWAIT(CMQTTREL   ,CMQTTREL_R  ,10)){
		//return false;   
	}
	if(!CMDnWAIT(NETCLOSE  	,NETCLOSE_R ,10)){
		//return false;   
	}
	if(!CMDnWAIT(CMQTTSTOP  ,CMQTTSTOP_R ,10)){
		return false;   
	}
  else{ 
		return true;    
	}
}

bool CMDSend(char * cmd){
#ifdef ShowCMD 
	NRF_LOG_INFO("Sending %s ",cmd); NRF_LOG_FLUSH();
#endif  
  int failcount=0;
  for(int ind=0; ind<strlen(cmd) ;ind++){
		if(nrf_serial_write(&Modem_Uart, &cmd[ind], sizeof(cmd[ind]), NULL, 100)!=NRF_SUCCESS){
			failcount++;
    }
	}
  if(strlen(cmd)>150){
		if(nrf_serial_write(&Modem_Uart,(uint8_t *)0x0A, 1, NULL, 100)!=NRF_SUCCESS){
			failcount++;
		}
	}
  if(failcount>1)
		return false;
  else
		return true;
}

bool shakhand(void){
    if(CMDSend(AT)){
        WaitingTime = CURRENTTIME;
        while(!strstr(DataFromModem,AT_R)){
            if(ReadModemResponse(2)){
                NRF_LOG_INFO("Data From Modem: %s\n",DataFromModem); NRF_LOG_FLUSH();
            }
            if(CURRENTTIME - WaitingTime>5)
                return false;
        }
        return true;
    }
    return false;
}

bool ReadModemResponse(uint32_t tout){
  int recv = 0;
    memset(DataFromModem,0,sizeof(DataFromModem));
    uint32_t start = CURRENTTIME;
    bool RapIt=false;
    do{
      uint8_t c;
			uint32_t ErrCode = nrf_serial_read(&Modem_Uart, &c, sizeof(c),NULL, 100);       
      nrf_delay_us(100);
			if(ErrCode == 0){
			//NRF_LOG_INFO("Per char %c  %02X\n",c,c); NRF_LOG_FLUSH();
				if((c==0x0A || c==0x0D ) && recv> 3 && RapIt==true){
					break;
				}
				if(c==0x0D || c==0x41 || c==0x0A){
					RapIt=true;
				}
				if(RapIt){
					DataFromModem[recv++] = c;
				}
			}
			else{
//				nrf_serial_flush(&Modem_Uart, 0);
//				nrf_serial_rx_drain(&Modem_Uart);
			}
		} 
  while((CURRENTTIME - start) < tout); 
    if(RapIt){
        if(strstr(DataFromModem,CGPSINFO_R)){
            if(strlen(DataFromModem)>30){ return true; }
            else{ return false; }
        }
        return true;
    }
    return false;
}

bool StartModem(){
    nrf_delay_ms(1000);
    CMDnWAIT(CRESET,CRESET_R,5); // Reset module
    CommandTime = CURRENTTIME;
    while(!strstr(DataFromModem,"PB DONE")){
        if(ReadModemResponse(3)){
            //NRF_LOG_INFO("Data From Modem: %s\n",DataFromModem); NRF_LOG_FLUSH();
        }
        if(CURRENTTIME - CommandTime>40)
            return false;
    }
    NRF_LOG_INFO("Modem Started\n"); NRF_LOG_FLUSH();
    return true;
}

static char cpytopic[100] = {0};
static char cpymsg[300] = {0};
bool MQTTPub(const char *topic,const char *msg){   
  static char CMQTTTOPIC[30]={0};
  static char CMQTTPAYLOAD[100]={0};
  unsigned int tlen = strlen(topic);
  unsigned int mlen = strlen(msg);
        
	memset(cpymsg,0,sizeof(cpymsg));
	memset(cpytopic,0,sizeof(cpytopic));
  sprintf(CMQTTTOPIC,"\r\nAT+CMQTTTOPIC=0,%u\r\n", tlen);
    if(CMDnWAIT(CMQTTTOPIC,">",5)){
        strcpy(cpytopic,topic);
        if(CMDnWAIT(cpytopic,AT_R,20)){
//#ifdef ShowPub                          
						NRF_LOG_INFO("Writting Topic Successfully\n"); NRF_LOG_FLUSH();
//#endif          
        }
				else{  
					return false;   
				}
    }
    else{   
			return false;   
		}
  sprintf(CMQTTPAYLOAD,"AT+CMQTTPAYLOAD=0,%u\r", mlen);
  if(CMDnWAIT(CMQTTPAYLOAD,">",5)){
		strcpy(cpymsg,msg);             
    if(CMDnWAIT(cpymsg,AT_R,20)){
//#ifdef ShowPub          
			NRF_LOG_INFO("Writting Message Successfully %s\n",cpymsg); NRF_LOG_FLUSH();
//#endif          
    }
		else{
			return false;
		}
  }
  else{
		return false;
	}
	nrf_delay_ms(10);
  if(!CMDnWAIT(CMQTTPUB,CMQTTPUB_R,10)){
		NRF_LOG_INFO("MQTT Msg Failed to send to server \n"); NRF_LOG_FLUSH();
		MQTT_Stop();
    return false;
	}
	
	MQTT_Stop();
	
  return true;    
}

void InitGPS(void){
    if(CMDnWAIT(CGPS,CGPS_R,10)){
        if(CMDnWAIT(CGPSNMEA,CGPSNMEA_R,4)){}
    }
}

char GPSLAT[15]={0};
char GPSLNG[15]={0};
bool GetLoc(){
	char Latd[2]= {0};
  char Lngd[2]= {0};
  int GpsSplitter = 0;
  if(CMDnWAIT(CGPSINFO,CGPSINFO_R,7)){
		char *ptr = strtok(DataFromModem, ":");
		while(ptr != NULL){
			ptr = strtok(NULL, ",");
			if(GpsSplitter==0){ 
				strcpy(GPSLAT,ptr); 
			}
      if(GpsSplitter==1){ 
				strcpy(Latd,ptr);   
			}
      if(GpsSplitter==2){ 
				strcpy(GPSLNG,ptr); 
			} 
      if(GpsSplitter==3){ 
				strcpy(Lngd,ptr);
				GpsSplitter=0;
				break;
			}
    GpsSplitter++;
		}
		NRF_LOG_INFO("GPS Location is: %s %s %s %s    :\n",GPSLAT,Latd,GPSLNG,Lngd);  NRF_LOG_FLUSH();
		nrf_delay_ms(10);
		return true;
	}
	return false;
}

void InitModemPort(void){
    nrf_serial_init(&Modem_Uart, &Modem_Config, &serial0_config);   
    nrf_serial_flush(&Modem_Uart, 0);
    nrf_serial_rx_drain(&Modem_Uart);
}

void DisconnectData(void){}
