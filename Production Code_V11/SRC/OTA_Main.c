#include "stm32f446xx.h"
#include "gpio_driver.h"
#include "can_driver2.h"
#include "delay.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rcc_clock.h"
#include "sdio_driver.h"
#include "ff.h"
#include "gps_driver.h"
#include "timer_driver.h"
#include "math.h"
#include "device_id.h"
#include "daq.h"
#include "esp.h"
#include "k_line.h"
#include "file_handle_api.h"
#include "ymodem.h"
#include "watchdog_driver.h"
#include <stdbool.h>
#include "rtc_driver.h"
//Timeout Macros
#define ESP_COUNTER_TIMEOUT 40//CHANGE IT BAKC TO 40
#define UPLOAD_TIMER_TIMEOUT 1000
#define RTC_WAKEUP_TIMEOUT 14344
	//FATFS Variables
	FATFS fs;            	  	/* Filesystem object */
	FIL file, boot_config, file_down, IFT, filex,RTC_file;           			/* File object */
	FRESULT res;        			/* API result code */
	FILINFO fno;							/* File information */
	UINT bw, br;            	/* Bytes written */

	//GPS Variables
	GPS_Data z;
	// New file
	uint32_t temp_file_counter=0;
	uint8_t filename_flag = 0;
	char filename[15]="t_";
	char file_ending[40], temp_buf[40];

	//Load DBC
	uint8_t data_log_mode, num_can_ids, response_fifo, can_speed, num_pgns, fifo1_pgn, copy_of_data_log_mode, copy_of_broadcast_params;
	uint8_t req_params, rpm_mode;
	uint32_t request_can_id[2]= {0xFFFFFFFF,0xFFFFFFFF} , response_can_id[2]={0xFFFFFFFF,0xFFFFFFFF};
	uint32_t heartbeat=0, file_timer=0;

	//CAN
	broadcast_param_can arr_brd_param[20];
	request_param arr_req_param[30];
	can_pgn pgn[15];

	//Data (updaterb, writecsv)
	uint8_t Data[45][4];
	char rb1[2500],rb2[2500];
	uint16_t rb1_size,rb2_size;
	uint8_t rb_flag =1;
	uint32_t file_size;

	//TIMER Parameters
	TIM_Base_InitTypeDef TIM6_param;
	TIM_Base_InitTypeDef TIM7_param;
	uint8_t stop_logging=0;

	//CAN Init Variables
	CAN_ConfigParamTypeDef ConfArg;
	CAN_ClockParamTypeDef ClkArg;	
	CAN_RxMsgType rx_can;
	CAN_TxMsgType tx_can;
	CAN_TxMailBox_TypeDef mail_box;
	uint8_t can_inactive=0;

	//K-Line
	USART_Type param6;
	uint8_t timer_mode=0;
	uint8_t kline_init=0;
	uint8_t kline_crc=0; //for tx
	uint8_t kline_inactive=0;
	uint8_t b;
	uint8_t msg_status, check_calc = 0, check_uart = 0; //usart6
	uint8_t Kline_RxBuff[50], kline_buf[7];   //increase buffer size when need arises
	uint8_t kline_process_flag = 0;
	uint8_t last_req=0;
	uint16_t kline_len = 0, buff_length = 0, kline_cnt = 0;

	//Flags & Counters
	uint8_t ENG_RUNNING=0;
	uint8_t IGN_ON=0;
    //RTC WAKE UP FLAG

	uint8_t RTC_Fail = 0;
	uint8_t RTC_Wake = 0;
	char buff[35];
	uint16_t rtc_counter = 0;
	uint16_t rtc_timeout_gap;
	RTCError resrtc;
	uint32_t tempregister;
	uint8_t ps = 0;
	typedef struct
	{
		char time[6];
		uint8_t type;
	} wifi_history;
	wifi_history wh[100];
	uint8_t wh_counter = 0;

	//ESP
	int esp_counter = 0, upload_timer = 0;
	uint8_t esp_counter_active = 0, upload_timer_active = 0, upload_active = 0, upload_complete = 0;
	uint8_t delay_timer = 0, delay_timer_active = 0;
	uint8_t wifi_fail = 0, esp_filename_start = 0, esp_filename_counter = 0;
	uint8_t esp_filename_wait = 0, esp_lastfilename[15];
	char* IFT_filename = "";
	_Bool IFT_complete = 0,IFT_active = 0, STM_SLEEP = 0, exception_01 = 0, write_once = 0;
	uint8_t	IFT_fail=0;
	char wh_temp[2];

	uint8_t file_transfer_ready = 0;

	char version_log[145];
	uint8_t file1, file2, file3, file4, file5;
	uint8_t pay_ver, dbc_ver, cmd_ver, ecu_ver, stm_ver;			// 1 - Payload version  2 - DBC
														// 3 - CMD  4 - ECU   5 - STM

	int stat1 = 0,stat2 = 0, stat3 = 0, stat4 = 0, stat5 = 0;
	uint8_t ecu_ver, pay_ver, cmd_ver;
	uint8_t message_start = 0, abort_file = 0, set_variable = 1, update_checked = 0, error = 0, do_once = 0, play_once = 0, Update_payload = 0;   // Use set variable before ymodem_receive
	uint8_t Data_IFT[PACKET_1K_SIZE + PACKET_OVERHEAD];
	uint16_t packet_size=0, count=0, index_IFT=0;
	char orig_name[25] = {'\0'};
	char size_IFT[256] = {'\0'};
	uint32_t filesize_IFT = 0, filesize_check = 0;
	uint8_t a_bit, rename_flag = 0, IFT_Start = 0, download_fail = 0, down_status = 0, its_done = 0, down_done = 0;
	char Start_Date[6], dbc_ver[12];
	int get_date = 0, timer7_mode = 1, counter = 0, download_timeout = 60;

	uint32_t tracker_req = 0, log_tracker_brd = 0, log_tracker_req = 0;

	//Wifi Initialization
	void wifi_init()
	{
		RCC_GPIOA_CLOCK_ENABLE_CLK();
		GPIO_Config_Mode(GPIOA, GPIO_PIN_12, GPIO_MODE_OUT);
		GPIO_Config_OType(GPIOA, GPIO_PIN_12, GPIO_OTYPE_PP);
		GPIO_Config_Speed(GPIOA, GPIO_PIN_12, GPIO_LOW_SPEED);
		GPIO_Config_PuPd(GPIOA, GPIO_PIN_12, GPIO_PUPD_DOWN);
	}
	//wifi power on
	void wifi_power_on()
	{
		GPIO_Write_Bit(GPIOA, GPIO_PIN_12, 0);
	}
	//wifi power off
	void wifi_power_off()
	{
		GPIO_Write_Bit(GPIOA, GPIO_PIN_12, 1);
	}

	void RTC_Heartbeat(uint8_t success)
	{
		if(success == 1)
		{
			if(!gps_log_flag)
			{
				ps = 1;
				//DONGLE WILL BE SWITCHED ON HERE
				char temp[2];
				char buff[35];
				char temp_buff[50];
				//UNIQUE ID
				sprintf(buff, "%08X%08X%08X\n",
						TM_ID_GetUnique32(0),    /* LSB */
						TM_ID_GetUnique32(1),
						TM_ID_GetUnique32(2));    /* MSB */
				memset(temp_buff,0,sizeof(temp_buf));

				//Log the timestamp
				memcpy(temp_buff,"&",1);
				strncat(temp_buff,"000000",6);
				strncat(temp_buff,",",1);
				//Log the latitude value

				strncat(temp_buff,"0000000000",10);
				strncat(temp_buff,",",1);
				//Log the longitude value
				strncat(temp_buff,"0000000000",10);
				strncat(temp_buff,",",1);
				strncat(temp_buff,buff,35);
				strncat(temp_buff,"\n",1);
				ESP_CMD_SEND(temp_buff);//send command to esp
				memset(temp_buf,0,sizeof(temp_buf));
		       }
			else
			{
				//DONGLE WILL BE SWITCHED ON HERE
				char temp[2];
				char buff[35];
				char temp_buff[50];
				//UNIQUE ID
				sprintf(buff, "%08X%08X%08X\n",
						TM_ID_GetUnique32(0),    /* LSB */
						TM_ID_GetUnique32(1),
						TM_ID_GetUnique32(2));    /* MSB */
				memset(temp_buff,0,sizeof(temp_buf));

				//Log the timestamp
				memcpy(temp_buff,"&",1);
				strncat(temp_buff,z.time,6);
				strncat(temp_buff,",",1);
				//Log the latitude value

				strncat(temp_buff,z.lat,10);
				strncat(temp_buff,",",1);
				//Log the longitude value
				strncat(temp_buff,z.longi,10);
				strncat(temp_buff,",",1);
				strncat(temp_buff,buff,35);
				strncat(temp_buff,"\n",1);
				ESP_CMD_SEND(temp_buff);//send command to esp
				memset(temp_buf,0,sizeof(temp_buf));

			}
		}
		else
		{
			if(!gps_log_flag)
			{
				ps = 1;
				//DONGLE WILL BE SWITCHED ON HERE
				char temp[2];
				char buff[35];
				char temp_buff[50];
				//UNIQUE ID
				sprintf(buff, "%08X%08X%08X\n",
						TM_ID_GetUnique32(0),    /* LSB */
						TM_ID_GetUnique32(1),
						TM_ID_GetUnique32(2));    /* MSB */
				memset(temp_buff,0,sizeof(temp_buf));

				//Log the timestamp
				memcpy(temp_buff,"*",1);
				strncat(temp_buff,"000000",6);
				strncat(temp_buff,",",1);
				//Log the latitude value

				strncat(temp_buff,"0000000000",10);
				strncat(temp_buff,",",1);
				//Log the longitude value
				strncat(temp_buff,"0000000000",10);
				strncat(temp_buff,",",1);
				strncat(temp_buff,buff,35);
				strncat(temp_buff,"\n",1);
				ESP_CMD_SEND(temp_buff);//send command to esp
				memset(temp_buf,0,sizeof(temp_buf));
		       }
			else
			{
				//DONGLE WILL BE SWITCHED ON HERE
				char temp[2];
				char buff[35];
				char temp_buff[50];
				//UNIQUE ID
				sprintf(buff, "%08X%08X%08X\n",
						TM_ID_GetUnique32(0),    /* LSB */
						TM_ID_GetUnique32(1),
						TM_ID_GetUnique32(2));    /* MSB */
				memset(temp_buff,0,sizeof(temp_buf));

				//Log the timestamp
				memcpy(temp_buff,"*",1);
				strncat(temp_buff,z.time,6);
				strncat(temp_buff,",",1);
				//Log the latitude value

				strncat(temp_buff,z.lat,10);
				strncat(temp_buff,",",1);
				//Log the longitude value
				strncat(temp_buff,z.longi,10);
				strncat(temp_buff,",",1);
				strncat(temp_buff,buff,35);
				strncat(temp_buff,"\n",1);
				ESP_CMD_SEND(temp_buff);//send command to esp
				memset(temp_buf,0,sizeof(temp_buf));

			}

		}




	}
	//Config File
	void config_file()
	{
//		unsigned int device_id_val;
		char config_buf[20];
		char s[5];
		uint8_t k,i;
		//Create a new Config File


			res = f_open(&file, "config.csv", FA_OPEN_ALWAYS|FA_WRITE|FA_READ);
		
			memset(config_buf,0,sizeof(config_buf));
			f_gets(config_buf,20,&file);
			
			memset(config_buf,0,sizeof(config_buf));
			f_gets(config_buf,20,&file);
			i=0;
			k=1;
			while(k)
			{
				if(config_buf[i] != '\n')
				{
						s[i]=config_buf[i];
						i++;
				}
				else 
				{
					s[i]='\0';
					k=0;
				}
			}
			temp_file_counter=atoi(s);

			memset(config_buf,0,sizeof(config_buf));				// To flash / Not
			f_gets(config_buf,20,&file);
			memset(config_buf,0,sizeof(config_buf));
			f_gets(config_buf,20,&file);							// BIN to be flashed

			memset(config_buf,0,sizeof(config_buf));
			f_gets(config_buf,20,&file);							// Payload to flash ? Yes / No
			memset(s,0,sizeof(s));

			i=0;
			k=1;
			while(k)
			{
				if(config_buf[i] != '\n')
				{
						s[i]=config_buf[i];
						i++;
				}
				else
				{
					s[i]='\0';
					k=0;
				}
			}
			Update_payload=atoi(s);

			memset(config_buf,0,sizeof(config_buf));				// Version Numbers
			f_gets(config_buf,10,&file);
			memset(s,0,sizeof(s));

			i=0;
			k=1;
			while(k)
			{
				if(config_buf[i] != '\n')
				{
						s[i]=config_buf[i];
						i++;
				}
				else
				{
					s[i]='\0';
					k=0;
				}
			}
			pay_ver=atoi(s);

			memset(config_buf,0,sizeof(config_buf));
			f_gets(config_buf,10,&file);
			memset(s,0,sizeof(s));
			i=0;
			k=1;
			while(k)
			{
				if(config_buf[i] != '\n')
				{
						s[i]=config_buf[i];
						i++;
				}
				else
				{
					s[i]='\0';
					k=0;
				}
			}
			dbc_ver=atoi(s);

			memset(config_buf,0,sizeof(config_buf));
			f_gets(config_buf,10,&file);
			memset(s,0,sizeof(s));
			i=0;
			k=1;
			while(k)
			{
				if(config_buf[i] != '\n')
				{
						s[i]=config_buf[i];
						i++;
				}
				else
				{
					s[i]='\0';
					k=0;
				}
			}
			cmd_ver=atoi(s);

			memset(config_buf,0,sizeof(config_buf));
			f_gets(config_buf,10,&file);
			memset(s,0,sizeof(s));
			i=0;
			k=1;
			while(k)
			{
				if(config_buf[i] != '\n')
				{
						s[i]=config_buf[i];
						i++;
				}
				else
				{
					s[i]='\0';
					k=0;
				}
			}
			ecu_ver=atoi(s);


			memset(config_buf,0,sizeof(config_buf));
			f_gets(config_buf,10,&file);
			memset(s,0,sizeof(s));
			i=0;
			k=1;
			while(k)
			{
				if(config_buf[i] != '\n')
				{
						s[i]=config_buf[i];
						i++;
				}
				else
				{
					s[i]='\0';
					k=0;
				}
			}
			stm_ver=atoi(s);
			memset(config_buf, 0, sizeof(config_buf));
			f_gets(config_buf, 10, &file);
			memset(s, 0, sizeof(s));
			i = 0;
			k = 1;
			while (k)
			{
				if (config_buf[i] != '\n')
				{
					s[i] = config_buf[i];
					i++;
				}
				else
				{
					s[i] = '\0';
					k = 0;
				}
			}
			RTC_Wake = atoi(s);

			do
			{
				res = f_close(&file);
			}while(res!=FR_OK);



	}

	void mount_card()
	{
		//SDIO
			/*FOR SD card pwr enable (got from Initialization file)*/	
			RCC_GPIOA_CLOCK_ENABLE_CLK();
			GPIO_Config_Mode (GPIOA,GPIO_PIN_7,GPIO_MODE_OUT);        
			GPIO_Config_OType (GPIOA, GPIO_PIN_7, GPIO_OTYPE_PP); 
			GPIO_Config_Speed (GPIOA, GPIO_PIN_7, GPIO_LOW_SPEED);    
			GPIO_Write_Bit (GPIOA, GPIO_PIN_7,1);	
		
		//Initialize pins and SDIO peripheral
			SD_SDIOInit();  
			delay(100);
		
		//FATFS
			/* Register work area */
				res=f_mount(&fs, "", 0);
				delay(200);
	}
	void load_dbc()
	{
		//Required variable for this funct
			char temp_buff[30];
			uint8_t i,j,k,m,n;
			int p;
			k=1;
			i=0;
			j=0;
			char s[9];
		
		//Open required file1
			res= f_open(&file, "dbc.csv", FA_OPEN_EXISTING|FA_READ);
	
		//First line has some additional charc attached to what's in the fil3. This is a workaround the bug
			f_gets (temp_buff,30,&file);
			memset(temp_buff,0,30);

		//Read DBC Version
			f_gets (temp_buff,30,&file);
			while(k)
			{
				if(temp_buff[i] != ',')
				{
						s[i]=temp_buff[i];
						i++;
				}
				else
				{
					s[i]='\0';
					k=0;
				}
			}
			strncpy(dbc_ver, s, 11);
			memset(s,0,sizeof(s));
			memset(temp_buff,0,30);
			k=1;
			i=0;
			
		//Read the mode for data logging
			f_gets (temp_buff,30,&file);
			while(k)
			{
				if(temp_buff[i] != ',')
				{
						s[i]=temp_buff[i];
						i++;
				}
				else 
				{
					s[i]='\0';
					k=0;
				}
			}
			data_log_mode=(uint8_t)atoi(s);
			copy_of_data_log_mode = data_log_mode;
			memset(s,0,4);
			memset(temp_buff,0,sizeof(temp_buff));
			k=1;
			i=0;

		//Read No.of sets of CAN IDs
			f_gets (temp_buff,30,&file);
			while(k)
			{
				if(temp_buff[i] != ',')
				{
						s[i]=temp_buff[i];
						i++;
				}
				else 
				{
					s[i]='\0';
					k=0;
				}
			}
			num_can_ids=(uint8_t)atoi(s);
			memset(s,0,4);
			memset(temp_buff,0,sizeof(temp_buff));
			k=1;
			i=0;	
			j=0;
		while(j<num_can_ids)
		{	
		//Read the Request Can ID
			f_gets (temp_buff,30,&file);
			while(k)
			{
				if(temp_buff[i] != ',')
				{
						s[i]=temp_buff[i];
						i++;
				}
				else 
				{
					s[i]='\0';
					k=0;
				}
			}
			sscanf(s,"%8x",&request_can_id[j]);
			memset(s,0,sizeof(s));
			memset(temp_buff,0,sizeof(temp_buff));
			k=1;
			i=0;
		
		//Read the Response Can ID
			f_gets (temp_buff,30,&file);
			while(k)
			{
				if(temp_buff[i] != ',')
				{
						s[i]=temp_buff[i];
						i++;
				}
				else 
				{
					s[i]='\0';
					k=0;
				}
			}
			sscanf(s,"%8x",&response_can_id[j]);
			memset(s,0,sizeof(s));
			memset(temp_buff,0,sizeof(temp_buff));
			k=1;
			i=0;
			j++;
		}
		
		//Read the response FIFO #
			f_gets (temp_buff,15,&file);
			while(k)
			{
				if(temp_buff[i] != ',')
				{
						s[i]=temp_buff[i];
						i++;
				}
				else 
				{
					s[i]='\0';
					k=0;
				}
			}
			response_fifo=(uint8_t)atoi(s);
			memset(s,0,4);
			memset(temp_buff,0,sizeof(temp_buff));
			k=1;
			i=0;
			
		//Read baud rate
			f_gets (temp_buff,15,&file);
			while(k)
			{
				if(temp_buff[i] != ',')
				{
						s[i]=temp_buff[i];
						i++;
				}
				else 
				{
					s[i]='\0';
					k=0;
				}
			}
			can_speed=(uint8_t)atoi(s);
			memset(s,0,4);
			memset(temp_buff,0,sizeof(temp_buff));
			k=1;
			i=0;
			
		//Read the # of PGNs
			f_gets (temp_buff,15,&file);
			while(k)
			{
				if(temp_buff[i] != ',')
				{
						s[i]=temp_buff[i];
						i++;
				}
				else 
				{
					s[i]='\0';
					k=0;
				}
			}
			num_pgns=(uint8_t)atoi(s);
			memset(s,0,4);
			memset(temp_buff,0,sizeof(temp_buff));
			k=1;
			i=0;
			
		//Read the # of PGNs for FIFO1
			f_gets (temp_buff,15,&file);
			while(k)
			{
				if(temp_buff[i] != ',')
				{
						s[i]=temp_buff[i];
						i++;
				}
				else 
				{
					s[i]='\0';
					k=0;
				}
			}
			fifo1_pgn=(uint8_t)atoi(s);
			memset(s,0,4);
			memset(temp_buff,0,sizeof(temp_buff));
			k=1;
			i=0;
			
		//Read the # of broadcast params
			f_gets (temp_buff,15,&file);
			while(k)
			{
				if(temp_buff[i] != ',')
				{
						s[i]=temp_buff[i];
						i++;
				}
				else 
				{
					s[i]='\0';
					k=0;
				}
			}
			broadcast_params=(uint8_t)atoi(s);
			copy_of_broadcast_params = broadcast_params;
			memset(s,0,4);
			memset(temp_buff,0,sizeof(temp_buff));
			k=1;
			i=0;
			
		//Read the # of request params
			f_gets (temp_buff,15,&file);
			while(k)
			{
				if(temp_buff[i] != ',')
				{
						s[i]=temp_buff[i];
						i++;
				}
				else 
				{
					s[i]='\0';
					k=0;
				}
			}
			req_params=(uint8_t)atoi(s);
			memset(s,0,4);
			memset(temp_buff,0,sizeof(temp_buff));
			k=1;
			i=0;
		
		//Read the RPM mode 
			f_gets (temp_buff,30,&file);
			while(k)
			{
				if(temp_buff[i] != ',')
				{
						s[i]=temp_buff[i];
						i++;
				}
				else 
				{
					s[i]='\0';
					k=0;
				}
			}
			rpm_mode=(uint8_t)atoi(s);
			memset(s,0,sizeof(s));
			memset(temp_buff,0,sizeof(temp_buff));
			k=1;
			i=0;
			j=0;
		//Loading PGNs		
		while(j<num_pgns)
		{
			memset(temp_buff,0,sizeof(temp_buff));
			memset(s,0,sizeof(s));			
			f_gets(temp_buff,30,&file);
			i=0;
			m=0;
			n=0;
			k=1;		
			while(k)
			{
				if(temp_buff[i]=='\0')
					k=0;
				else if(temp_buff[i]==',')
				{
					switch (m)
					{
						case 0:
						{
							s[n]='\0';
							sscanf(s,"%8x",&pgn[j].pgn_id);
							memset(s,0,sizeof(s));
							break;
						}
						case 1:
						{
							s[n]='\0';
							sscanf(s,"%d",&p);
							pgn[j].num_params=p;
							memset(s,0,sizeof(s));
							break;									
						}
						case 2:
						{
							s[n]='\0';
							sscanf(s,"%d",&p);
							pgn[j].param_strt=p;
							memset(s,0,sizeof(s));
							break;									
						}
					}
					m++;
					n=0;
				}
				else
				{
					s[n]=temp_buff[i];
					n++;
				}
				i++;
			}
			j++;
		}
		j=0;
			
		//Loading Brd params		
		while(j<broadcast_params)
		{
			memset(temp_buff,0,sizeof(temp_buff));
			memset(s,0,sizeof(s));
			f_gets(temp_buff,30,&file);
			i=0;
			m=0;
			n=0;
			k=1;		
			while(k)
			{
				if(temp_buff[i]=='\0')
					k=0;
				else if(temp_buff[i]==',')
				{
					switch (m)
					{
						case 0:
						{
							s[n]='\0';
							sscanf(s,"%d",&p);
							arr_brd_param[j].length=p;
							memset(s,0,sizeof(s));
							break;						
						}
						case 1:
						{
							s[n]='\0';
							sscanf(s,"%d",&p);
							arr_brd_param[j].log_freq=p;
							memset(s,0,sizeof(s));
							break;										
						}
						case 2:
						{
							s[n]='\0';
							sscanf(s,"%d",&p);
							arr_brd_param[j].position=p;
							memset(s,0,sizeof(s));
							break;				
						}
					}
					m++;
					n=0;
				}
				else
				{
					s[n]=temp_buff[i];
					n++;
				}
				i++;
			}
			j++;
		}
	
		j=0;
	
		//Loading req params	
		while(j<req_params) 
		{
			memset(temp_buff,0,sizeof(temp_buff));
			memset(s,0,sizeof(s));
			f_gets(temp_buff,50,&file);
			i=0;
			m=0;
			n=0;
			k=1;		
			while(k)
			{
				if(temp_buff[i]=='\0')
					k=0;
				else if(temp_buff[i]==',')
				{
					switch (m)
					{
						case 0:
						{
							s[n]='\0';																	// Add into daq 
							sscanf(s,"%d",&p);
							arr_req_param[j].can_id_no = p;
							memset(s,0,sizeof(s));
							break;			
						}

						case 1:
						{
							s[n]='\0';																	// Add into daq 
							sscanf(s,"%d",&p);
							arr_req_param[j].can_extn_param=p;
							memset(s,0,sizeof(s));
							break;			
						}

						case 2:
						{
							s[n]='\0';
							sscanf(s,"%d",&p);
							arr_req_param[j].length=p;
							memset(s,0,sizeof(s));
							break;			
						}
						case 3:
						{
							s[n]='\0';
							sscanf(s,"%d",&p);
							arr_req_param[j].log_freq=p;
							memset(s,0,sizeof(s));
							break;	
						}
						case 4:
						{
							s[n]='\0';
							sscanf(s,"%x",&p);
							arr_req_param[j].service=p;
							memset(s,0,sizeof(s));
							break;															
						}
						case 5:
						{
							s[n]='\0';
							sscanf(s,"%x",&p);
							arr_req_param[j].PID=p;
							memset(s,0,sizeof(s));
							break;				
						}
						case 6:
						{
							if(arr_req_param[j].can_extn_param)
							{
								s[n]='\0';
								sscanf(s,"%x",&p);
								arr_req_param[j].PID1=p;								// add into daq
								memset(s,0,sizeof(s));
							}
							break;				
						}
					}
					m++;
					n=0;
				}
				else
				{
					s[n]=temp_buff[i];
					n++;
				}
				i++;
			}
			j++;
		}
		do
		{
			res = f_close(&file);
		}while(res!=FR_OK);
	}
	
	void new_file()
	{

		char temp_buf[6];
		memset(filename,0,15);
		strncpy(filename, "t_", 2);
		temp_file_counter++;
		sprintf(temp_buf,"%06d",temp_file_counter);
		strncat(filename,temp_buf,6);
		strncat(filename,".csv",4);
		res=FR_NOT_READY;
		file_timer = 0;
		do
		{
			res = f_stat(filename, &fno);
			switch (res) 
			{
				case FR_OK:
					memset(filename, 0, sizeof(filename));
					memset(temp_buf, 0, 6);
					temp_file_counter++;
					strncat(filename, "t_", 2);
					sprintf(temp_buf, "%06d", temp_file_counter);
					strncat(filename, temp_buf, 6);
					strncat(filename, ".csv", 4);
					break;

				case FR_NO_FILE:
					res = f_open(&file, filename, FA_CREATE_NEW | FA_WRITE);
					filename_flag = 1;
				break;

				default:
				break;
			}
		}	while(filename_flag == 0); 
		file_size=0;
	}
	//Update Config File with new temp_counter
	void update_config_temp_counter(void)
	{
		char config_buf[20];
		res = f_open(&file, "config.csv", FA_OPEN_ALWAYS|FA_WRITE|FA_READ);
		sprintf(config_buf,"Blank\n%6d",temp_file_counter);
		strncat(config_buf,"\n",1);
		f_write(&file, config_buf, 13, &bw);
		do
		{
			res = f_close(&file);
		}while(res!=FR_OK);
	}

	/*
		Initiate the DAQ module 
		Input Data log mode
	*/
		/*
		Intitalize CAN Peripheral & Interrupts
	*/
	void Init_CAN()
	{

		RCC_GPIOB_CLOCK_ENABLE_CLK();                          //GPIO CLOCK
		RCC_CAN1_CLOCK_ENABLE_CLK();                           // CAN clock enabling
			
		GPIO_Config_Mode (GPIOB,GPIO_PIN_8,GPIO_MODE_AF);        //Alternate function  for CAN Rx
		GPIO_Config_PuPd (GPIOB, GPIO_PIN_8, GPIO_PUPD_UP);	
		GPIO_Config_Speed (GPIOB, GPIO_PIN_8, GPIO_HIGH_SPEED); 
			
		GPIO_Config_Mode (GPIOB,GPIO_PIN_9,GPIO_MODE_AF);        //Alternate function  for CAN Tx
		GPIO_Config_OType (GPIOB, GPIO_PIN_9, GPIO_OTYPE_PP);
		GPIO_Config_PuPd (GPIOB, GPIO_PIN_9, GPIO_PUPD_UP);	
		GPIO_Config_Speed (GPIOB, GPIO_PIN_9, GPIO_HIGH_SPEED); 
			
		GPIO_Config_AF(GPIOB, GPIO_PIN_9, 9);                 /*Alt function configuration function */
		GPIO_Config_AF(GPIOB, GPIO_PIN_8, 9);                  /*Alt function configuration function */
		
		//can_stb
		/*FOR can_stb enable (got from Initialization file)*/	
		GPIO_Config_Mode (GPIOB,GPIO_PIN_7,GPIO_MODE_OUT);        
		GPIO_Config_OType (GPIOB, GPIO_PIN_7, GPIO_OTYPE_PP); 
		GPIO_Config_Speed (GPIOB, GPIO_PIN_7, GPIO_LOW_SPEED);    
		GPIO_Write_Bit (GPIOB, GPIO_PIN_7,0);				
	}

	void can_filt_init()
	{
		switch(data_log_mode)
		{
			case 0:						//Pure CAN 2.0
			{
				uint8_t i=0,j=0;
				CAN_FilterInit_SlaveBankStart(num_pgns+num_can_ids);   
				while(i<num_pgns)
				{
					CAN_FilterInit_Req(i);
					CAN_FilterInit_FilterMode(i,CAN_FILTERMODE_IDMASK );
					CAN_FilterInit_FilterScale(i,CAN_FILTERSCALE_32BIT );					
					CAN_Filter_IDMaskModify(i,1,((pgn[i].pgn_id<<3)|0X04)); 
					CAN_Filter_IDMaskModify(i,2,((0xFFFFFF<<3)|0X04));
					if(i<fifo1_pgn)
						CAN_FilterInit_FilterAssign(i,0);
					else
						CAN_FilterInit_FilterAssign(i,1);
					
					CAN_FilterActivate(i);
					i++;
				}
				//Assign filter for responses for requested params
				while(j<num_can_ids)
				{	
					CAN_FilterInit_Req(i);
					CAN_FilterInit_FilterMode(i,CAN_FILTERMODE_IDMASK );
					CAN_FilterInit_FilterScale(i,CAN_FILTERSCALE_32BIT );					
					CAN_Filter_IDMaskModify(i,1,((response_can_id[j]<<3)|0X04)); 
					CAN_Filter_IDMaskModify(i,2,((0xFFFF00<<3)));                             			//mask for PGN
					CAN_FilterInit_FilterAssign(i,(response_fifo-1));
					CAN_FilterActivate(i);
					j++;
					i++;
				}
				break;
			}
			case 1:					//Pure CAN 1.0
			{
				uint8_t i=0,j=0;
				CAN_FilterInit_SlaveBankStart(num_pgns+num_can_ids);   
				while(i<num_pgns)
				{
					CAN_FilterInit_Req(i);
					CAN_FilterInit_FilterMode(i,CAN_FILTERMODE_IDMASK );
					CAN_FilterInit_FilterScale(i,CAN_FILTERSCALE_32BIT );					
					CAN_Filter_IDMaskModify(i,1,(pgn[i].pgn_id<<21)); 
					CAN_Filter_IDMaskModify(i,2,0xFFE00000);                             			//mask for PGN
					if(i<fifo1_pgn)
						CAN_FilterInit_FilterAssign(i,0);
					else
						CAN_FilterInit_FilterAssign(i,1);
					
					CAN_FilterActivate(i);
					i++;
				}
				while(j<num_can_ids)
				{
					CAN_FilterInit_Req(i);
					CAN_FilterInit_FilterMode(i,CAN_FILTERMODE_IDMASK );
					CAN_FilterInit_FilterScale(i,CAN_FILTERSCALE_32BIT );					
					CAN_Filter_IDMaskModify(i,1,(response_can_id[j]<<21)); 
					CAN_Filter_IDMaskModify(i,2,0xFFE00000);                             			//mask for PGN
					CAN_FilterInit_FilterAssign(i,(response_fifo-1));
					CAN_FilterActivate(i);
					j++;
				}
				break;
			}
			case 2:					//CAN 2.0 & Kline
			{
				uint8_t i=0;
				CAN_FilterInit_SlaveBankStart(num_pgns);   
				while(i<num_pgns)
				{
					CAN_FilterInit_Req(i);
					CAN_FilterInit_FilterMode(i,CAN_FILTERMODE_IDMASK );
					CAN_FilterInit_FilterScale(i,CAN_FILTERSCALE_32BIT );					
					CAN_Filter_IDMaskModify(i,1,((pgn[i].pgn_id<<3)|0X04)); 
					CAN_Filter_IDMaskModify(i,2,((0xFFFFFF<<3)|0X04));                             			//mask for PGN
					if(i<fifo1_pgn)
						CAN_FilterInit_FilterAssign(i,0);
					else
						CAN_FilterInit_FilterAssign(i,1);
					
					CAN_FilterActivate(i);
					i++;
				}
				break;
			}
			case 3:					//CAN 1.0 & Kline
			{
				uint8_t i=0;
				CAN_FilterInit_SlaveBankStart(num_pgns);   
				while(i<num_pgns)
				{
					CAN_FilterInit_Req(i);
					CAN_FilterInit_FilterMode(i,CAN_FILTERMODE_IDMASK );
					CAN_FilterInit_FilterScale(i,CAN_FILTERSCALE_32BIT );					
					CAN_Filter_IDMaskModify(i,1,(pgn[i].pgn_id<<21)); 
					CAN_Filter_IDMaskModify(i,2,0xFFE00000);                             			//mask for PGN
					if(i<fifo1_pgn)
						CAN_FilterInit_FilterAssign(i,0);
					else
						CAN_FilterInit_FilterAssign(i,1);
					CAN_FilterActivate(i);
					i++;
				}
				break;			
			}
		}
		CAN_FilterInit_Quit();
	}

	void setup_can()
	{
		ClkArg.BS1=CAN_BS1_13TQ;
		ClkArg.BS2=CAN_BS2_2TQ;                             //bit timing
		ClkArg.Mode=CAN_MODE_NORMAL;                        //NORMAL MODE................
		switch(can_speed)
		{
			case 0:
			{
				ClkArg.Prescaler=3;																//For 500kbps 3
				break;
			}
			case 1:
			{
				ClkArg.Prescaler=6;																//For 250kbps-6 
				break;
			}
		}
																		
		ClkArg.SJW=CAN_SJW_1TQ;
			
		ConfArg.AutoBusOff=CAN_ABOM_ENABLE;
		ConfArg.AutoWakeUp=CAN_AWUM_ENABLE;
		ConfArg.NoAutoRetransmission=CAN_NART_DISABLE;              // additional features
		ConfArg.ReceiveFifoLocked=CAN_RXFIFO_OVERRUN_LOCKED;
		ConfArg.TimeTriggeredMode= CAN_TTCM_DISABLE;
		ConfArg.TransmitFifoPriority=CAN_TXFIFO_PRIO_BY_ID;
					
		CAN_Init_Req(CAN1);                                  
		CAN_Init_Clock(CAN1, &ClkArg);                             						//initialization and configuration
		CAN_Init_Config(CAN1, &ConfArg);
		CAN_DBGFreeze(CAN1, DISABLE);
		CAN_Init_Quit(CAN1);
		
		
		//Filters
			can_filt_init();

		CAN_IntMaskEnable(CAN1, CAN_IT_FMP0 );  //set the interrupt    Rx interrupt FIFO0

		CAN_IntMaskEnable(CAN1, CAN_IT_FMP1 );  //set the interrupt    Rx interrupt FIFO1

						
		//set the interrupt    Rx interrupt FIFO0
		NVIC_ClearPendingIRQ(CAN1_RX0_IRQn);   //clear any pending interrupt
		NVIC_SetPriority(CAN1_RX0_IRQn,3);
		NVIC_EnableIRQ(CAN1_RX0_IRQn);         //enable from NVIC
		
		//set the interrupt    Rx interrupt FIFO1
		NVIC_ClearPendingIRQ(CAN1_RX1_IRQn);   //clear any pending interrupt
		NVIC_SetPriority(CAN1_RX1_IRQn,4);
		NVIC_EnableIRQ(CAN1_RX1_IRQn);         //enable from NVIC
		
	}


	void TIM6_Init_kline_daq()
	{
		RCC_TIM6_CLOCK_ENABLE_CLK();
		
		TIM6_param.Period = 9;
		TIM6_param.Prescaler = 47999;
		TIM6_Base_Init(&TIM6_param);
		
		NVIC_SetPriority(TIM6_DAC_IRQn,2);
		NVIC_ClearPendingIRQ(TIM6_DAC_IRQn); 
		NVIC_EnableIRQ(TIM6_DAC_IRQn);
	}

	void TIM6_Init_kline_init()
	{
		RCC_TIM6_CLOCK_ENABLE_CLK();
		
		TIM6_param.Period = 24;
		TIM6_param.Prescaler = 47999;
		TIM6_Base_Init(&TIM6_param);
		
		NVIC_SetPriority(TIM6_DAC_IRQn,1);
		NVIC_ClearPendingIRQ(TIM6_DAC_IRQn); 
		NVIC_EnableIRQ(TIM6_DAC_IRQn);
	}

	void TIM7_Init()
	{
		RCC_TIM7_CLOCK_ENABLE_CLK();
		
		TIM7_param.Period = 999;
		TIM7_param.Prescaler = 47999;
		TIM7_Base_Init(&TIM7_param);
		
		NVIC_SetPriority(TIM7_IRQn,5);
		NVIC_ClearPendingIRQ(TIM7_IRQn); 
		NVIC_EnableIRQ(TIM7_IRQn);
	}

	void init_daq()
		{
			switch(data_log_mode)
			{
				case 0:
				{
					//CAN
						Init_CAN();
						setup_can();
					//Init the TX_CAN messages
						tx_can.IDE=CAN_ID_EXT;
						tx_can.RTR=CAN_RTR_DATA;
						tx_can.DLC=8;
					//Timer Mode
						timer_mode=1;

					break;
				}
				case 1:
				{
					//CAN
						Init_CAN();
						setup_can();
					//Init the TX_CAN messages
						tx_can.IDE=CAN_ID_STD;
						tx_can.RTR=CAN_RTR_DATA;
						tx_can.DLC=8;
					//Timer Mode
						timer_mode=1;

					break;
				}
				case 2:
				{
					//K Line Init
						KLINE_INIT_SEQ();
						NVIC_EnableIRQ(USART2_IRQn);
					//CAN
						Init_CAN();
						setup_can();
					//Init DAQ Timer
						TIM6_Init_kline_daq();
						NVIC_DisableIRQ(TIM6_DAC_IRQn);
					break;
				}
				case 3:
				{
					//K Line Init
						KLINE_INIT_SEQ();
						NVIC_EnableIRQ(USART2_IRQn);
					//CAN
						Init_CAN();
						setup_can();
					//Init DAQ Timer
						TIM6_Init_kline_daq();
						NVIC_DisableIRQ(TIM6_DAC_IRQn);
					break;
				}
			}
		}
	/*
		K-Line Init sequence will be implemented by this function 
	*/	
	void KLINE_INIT_SEQ()
	{
		// Init TX Pin of USART6 as GPIO
			RCC_GPIOC_CLOCK_ENABLE_CLK();
	
		// Disable all other NVICS
			NVIC_DisableIRQ(USART2_IRQn);
		
		// Prepare for USART6 init. Initiate after the GPIO signal 
			param6.USART_BaudRate=10400;
			param6.USART_WordLength=USART_WORDLENGTH_8B;
			param6.USART_HardwareFlowControl=USART_HARDWAREFLOWCONTROL_NONE;
			param6.USART_Mode=USART_MODE_Rx_Tx;
			param6.USART_StopBits=USART_STOPBITS_1;
			param6.USART_Parity=USART_PARITY_NO;
			USART_Disable(USART6);
		
		//Init TIm 6 @ 25ms
			timer_mode=0;
			kline_init=0;
			TIM6_Init_kline_init();
			TIM6_Base_Start_IT();
		
		//Wait for the TIM6 IRQ handler to run twice
			while(kline_init<3)
			{}
		timer_mode=1;

	}
	/*
		K-Line Init sequence will be implemented by this function 
	*/
	void KLINE_REINIT()
	{
		NVIC_DisableIRQ(USART2_IRQn);
		NVIC_DisableIRQ(USART6_IRQn);
		NVIC_DisableIRQ(TIM6_DAC_IRQn);
		NVIC_DisableIRQ(TIM7_IRQn);
		NVIC_DisableIRQ(CAN1_RX1_IRQn);
		NVIC_DisableIRQ(CAN1_RX0_IRQn);
		NVIC_DisableIRQ(USART3_IRQn);
		KLINE_INIT_SEQ();
		TIM6_Init_kline_daq();
		NVIC_EnableIRQ(USART2_IRQn);
		NVIC_EnableIRQ(USART6_IRQn);
		NVIC_EnableIRQ(TIM6_DAC_IRQn);
		NVIC_EnableIRQ(TIM7_IRQn);
		NVIC_EnableIRQ(CAN1_RX1_IRQn);
		NVIC_EnableIRQ(CAN1_RX0_IRQn);
		NVIC_EnableIRQ(USART3_IRQn);
	}

	void update_rb()
	{
		char temp_buf[20];
		char buf[2];
		int j=0;
		if(rb_flag==1)
		{
			memset(temp_buf,0,sizeof(temp_buf));
			//Log the timestamp 
				memcpy(temp_buf,z.time,6);
				strncat(temp_buf,",",1);
				strncat(rb1,temp_buf,7);
				rb1_size=rb1_size+7;
				memset(temp_buf,0,sizeof(temp_buf));
			
			if(gps_log_flag)
			{

				//Log the Lat
					memcpy(temp_buf,z.lat,9);
					strncat(temp_buf,",",1);
					strncat(rb1,temp_buf,10);
					rb1_size=rb1_size+10;
					memset(temp_buf,0,sizeof(temp_buf));

				//Log the Long 
					memcpy(temp_buf,z.longi,10);
					strncat(temp_buf,",",1);
					strncat(rb1,temp_buf,11);
					rb1_size=rb1_size+11;
					memset(temp_buf,0,sizeof(temp_buf));

				//Log the Altitude
					memcpy(temp_buf,z.altitude,6);
					strncat(temp_buf,",",1);
					strncat(rb1,temp_buf,7);
					rb1_size=rb1_size+7;
					memset(temp_buf,0,sizeof(temp_buf));
			}
			else
			{
				strncat(rb1,",,,",3);
				rb1_size = rb1_size + 3;
			}
			for (int k=0;k<broadcast_params+req_params;k++)
			{
				if(k < broadcast_params)
				{
					for(j=0; j<arr_brd_param[k].length; j++)
					{
						sprintf(temp_buf,"%02x",Data[k][j]);
						strncat(rb1,temp_buf,2);
						rb1_size=rb1_size+2;
					}
				}
				else
				{
					if(arr_req_param[k-broadcast_params].length > 12)
					{
						for(j=0; (j<arr_req_param[k-broadcast_params].length) && (j<=3); j++)
						{
							sprintf(temp_buf,"%02x",Data[k][j]);
							strncat(rb1,temp_buf,2);
							rb1_size=rb1_size+2;
						}
					}
					else
					{
						for(j=0; (j<arr_req_param[k-broadcast_params].length) && (j<=1); j++)
						{
							sprintf(temp_buf,"%02x",Data[k][j]);
							strncat(rb1,temp_buf,2);
							rb1_size=rb1_size+2;
						}
					}
				}
					memset(temp_buf,0,sizeof(temp_buf));
					strncpy(temp_buf, ",", 1);
					strncat(rb1,temp_buf,1);
					rb1_size=rb1_size+1;
					memset(temp_buf,0,sizeof(temp_buf));

			}
			memset(temp_buf,0,sizeof(temp_buf));
			sprintf(temp_buf,"%08x",log_tracker_brd);
			strncat(rb1,temp_buf,8);
			strncat(rb1,",",1);
			memset(temp_buf,0,sizeof(temp_buf));
			sprintf(temp_buf,"%08x",log_tracker_req);
			strncat(rb1,temp_buf,8);
			rb1_size=rb1_size+17;
			strncpy(temp_buf, ",", 1);
			strncat(rb1,temp_buf,1);
			rb1_size=rb1_size+1;
			strncat(rb1,"\r\n",2);
			rb1_size=rb1_size+2;
		}
		if(rb_flag==2)
		{
			memset(temp_buf,0,sizeof(temp_buf));
			//Log the timestamp 
				memcpy(temp_buf,z.time,6);
				strncat(temp_buf,",",1);
				strncat(rb2,temp_buf,7);
				rb2_size=rb2_size+7;
				memset(temp_buf,0,sizeof(temp_buf));
			
			if(gps_log_flag)
			{
				//Log the Lat
					memcpy(temp_buf,z.lat,9);
					strncat(temp_buf,",",1);
					strncat(rb2,temp_buf,10);
					rb2_size=rb2_size+10;
					memset(temp_buf,0,sizeof(temp_buf));

				//Log the Long 
					memcpy(temp_buf,z.longi,10);
					strncat(temp_buf,",",1);
					strncat(rb2,temp_buf,11);
					rb2_size=rb2_size+11;
					memset(temp_buf,0,sizeof(temp_buf));

				//Log the Altitude
					memcpy(temp_buf,z.altitude,6);
					strncat(temp_buf,",",1);
					strncat(rb2,temp_buf,7);
					rb2_size=rb2_size+7;
					memset(temp_buf,0,sizeof(temp_buf));
			}
			else
			{
				strncat(rb2,",,,",3);
				rb2_size = rb2_size + 3;
			}
			for (int k=0;k<broadcast_params+req_params;k++)
			{
				if(k < broadcast_params)
				{
					for(j=0; j<arr_brd_param[k].length; j++)
					{
						sprintf(temp_buf,"%02x",Data[k][j]);
						strncat(rb2,temp_buf,2);
						rb2_size=rb2_size+2;
					}
				}
				else
				{
					if(arr_req_param[k-broadcast_params].length > 12)
					{
						for(j=0; (j<arr_req_param[k-broadcast_params].length) && (j<=3); j++)
						{
							sprintf(temp_buf,"%02x",Data[k][j]);
							strncat(rb2,temp_buf,2);
							rb2_size=rb2_size+2;
						}
					}
					else
					{
						for(j=0; (j<arr_req_param[k-broadcast_params].length) && (j<=1); j++)
						{
							sprintf(temp_buf,"%02x",Data[k][j]);
							strncat(rb2,temp_buf,2);
							rb2_size=rb2_size+2;
						}
					}
				}
					memset(temp_buf,0,sizeof(temp_buf));
					strncpy(temp_buf, ",", 1);
					strncat(rb2,temp_buf,1);
					rb2_size=rb2_size+1;
					memset(temp_buf,0,sizeof(temp_buf));

			}
			memset(temp_buf,0,sizeof(temp_buf));
			sprintf(temp_buf,"%08x",log_tracker_brd);
			strncat(rb2,temp_buf,8);
			strncat(rb2,",",1);
			memset(temp_buf,0,sizeof(temp_buf));
			sprintf(temp_buf,"%08x",log_tracker_req);
			strncat(rb2,temp_buf,8);
			rb2_size=rb2_size+17;
			strncpy(temp_buf, ",", 1);
			strncat(rb2,temp_buf,1);
			rb2_size=rb2_size+1;
			strncat(rb2,"\r\n",2);
			rb2_size=rb2_size+2;
	 }
		log_tracker_brd = 0;
		log_tracker_req = 0;
	}
	/*
	TIM6 IRQ handler 
	*/	
	void TIM6_DAC_IRQHandler()                 //IRQ FOR TIMER=250ms
	{
		/* TIM Update event */
		if((TIM6->SR & (TIMER_UPDATE_FLAG)) == TIMER_UPDATE_FLAG)
		{
			TIM6->SR = ~(TIMER_INTERRUPT);  
		}
		switch(timer_mode)
		{
			case 0:														//K-Line Init Mode 
			{
				switch(kline_init)
				{
					case 0:
					{
						GPIO_Config_Mode (GPIOC,GPIO_PIN_6,GPIO_MODE_OUT);        
						GPIO_Config_Speed (GPIOC, GPIO_PIN_6, GPIO_HIGH_SPEED);
						GPIO_Write_Bit (GPIOC, GPIO_PIN_6, 0);
						kline_init++;
						break;
					}
					case 1:
					{
						GPIO_Write_Bit (GPIOC, GPIO_PIN_6, 1);
						kline_init++;
						break;
					}
					case 2:
					{
						
						//Init as USART6
							USART6_Init(&param6);
							USART_Enable(USART6);	
						
						//Init IRQ for USART6 
							USART_IntMaskEnable(USART6,USART_IT_RXNE);
							NVIC_SetPriority(USART6_IRQn,1);	
							NVIC_ClearPendingIRQ(USART6_IRQn);             //clear any pending interrupt

						// Send First Message 

								kline_init++;
								KLINE_SEND_CMD(START_COMM_REQ,5);
							

						
							memset(Kline_RxBuff,0,kline_len+1);
							kline_cnt=0;

		                    //delay(5);
							//for(int i=0; i<90;i++);
							msg_status=0;
							check_calc=0;
							
						break;
					}
				}
				break;
			}
			case 1:														//Normal Timer Mode used for request messages
			{
				if(data_log_mode == 2 || data_log_mode == 3)
				{
					if (kline_init == 4)
					{
					 	kline_buf[0]=0x82;
						kline_buf[1]=0x10;
						kline_buf[2]=0xF1;
						kline_buf[3]=arr_req_param[last_req].service;
						kline_buf[4]=arr_req_param[last_req].PID;	
						kline_crc=0;
						for(int i=0;i<5;i++)
						{
							kline_crc=kline_crc+kline_buf[i];
						}
						kline_buf[5]=kline_crc;
						kline_buf[6]='\0';

						KLINE_SEND_CMD(kline_buf,6);
						memset(Kline_RxBuff,0,kline_len+1);
						kline_cnt=0;
						msg_status=0;
						check_calc=0;
						NVIC_DisableIRQ(TIM6_DAC_IRQn);
					}
				}
				break;
			}
		}
	}	
	
	void TIM7_IRQHandler()                 //IRQ FOR TIMER=500ms
	{

		heartbeat++;
		file_timer++;
		switch(timer7_mode)
		{
				case 0:
						counter++;
						break;

				case 1:
						if(esp_counter_active)
						{
							esp_counter++;
						}
						if(upload_timer_active)
						{
							upload_timer++;
						}
						if(delay_timer_active)
						{
							delay_timer++;
						}

						//Send request every cycle - acts as trigger
						switch(data_log_mode)
						{
							case 0:
							{

							}

							case 1:
							{
								can_inactive++;

								log_tracker_brd = tracker_brd;
								log_tracker_req = tracker_req;
								tracker_req = tracker_brd = 0;

								if(last_req >= req_params)
									last_req=0;
								//Reactivate Filters
								for(int i=0;i<num_pgns;i++)
								{
									CAN_FilterActivate(i);
								}

								if(arr_req_param[last_req].can_extn_param)
									tx_can.Data[0]=0x03;
								else
									tx_can.Data[0]=0x02;

								tx_can.Data[1]=arr_req_param[last_req].service;
								tx_can.Data[2]=arr_req_param[last_req].PID;

								if(arr_req_param[last_req].can_extn_param)
									tx_can.Data[3]=arr_req_param[last_req].PID1;

								tx_can.ExtId = request_can_id[arr_req_param[last_req].can_id_no];
//
								if(CAN_Which_MailboxIsEmpty(CAN1) != 0x0F)
									CAN_Transmit(CAN1,&tx_can,CAN_Which_MailboxIsEmpty(CAN1));
								break;
							}

							case 2:
							{

							}

							case 3:
							{
									can_inactive++;
									kline_inactive++;

									log_tracker_brd = tracker_brd;
									log_tracker_req = tracker_req;
									tracker_req = tracker_brd = 0;

									//Reactivate Filters
									for(int i=0;i<num_pgns;i++)
									{
										CAN_FilterActivate(i);
									}

									if(kline_inactive > 5 && can_inactive < 7)
									{
										memset(temp_buf,0,sizeof(temp_buf));
										memcpy(temp_buf,z.time,6);
										strncat(temp_buf,",",1);
										strncat(temp_buf,"ATTEMPT REINIT\n",15);

										res=f_write(&file,temp_buf,22, &bw);

										KLINE_REINIT();
										kline_inactive=0;
										break;

									}
									if (kline_init == 4)
									{
										if(last_req >=req_params)
											last_req=0;

										TIM6_Base_Start_IT();
										NVIC_EnableIRQ(TIM6_DAC_IRQn);
									}

								break;
							}
							default:
						    	break;
						}

						if(ENG_RUNNING==0 && IGN_ON)
						{
							//ENG-RUNNING check
							switch(rpm_mode)
							{
								case 0:
								{
									if( (Data[0][0]!=0x0 || Data[0][1]!=0x0) &&
											(Data[0][0]!=0xFF || Data[0][1]!=0xFF) )
									{
										ENG_RUNNING=1;
										if(filename_flag==0)
										{
											new_file();
										}
									}
									break;
								}
								case 1:
								{
									if( (Data[broadcast_params][0]!=0x0 || Data[broadcast_params][1]!=0x0) &&
											(Data[broadcast_params][0]!=0xFF || Data[broadcast_params][1]!=0xFF) )
									{
										ENG_RUNNING=1;
										if(filename_flag==0)
										{
											new_file();
										}

									}

									break;
								}
							}

						}
						else if(ENG_RUNNING && IGN_ON)
						{

						//Check if RPM reading is greater than zero if not increment stop_logging
							switch(rpm_mode)
							{
								case 0:
								{
									if((Data[0][0]==0x0 && Data[0][1]==0x0) || (Data[0][0]==0xFF && Data[0][1]==0xFF))
									{
										stop_logging++;
									}
									else
									{
										stop_logging=0;
									}
									break;
								}
								case 1:
								{
									if((Data[broadcast_params][0]==0x0 && Data[broadcast_params][1]==0x0)
										|| (Data[broadcast_params][0]==0xFF && Data[broadcast_params][1]==0xFF))
									{
										stop_logging++;
									}
									else
									{
										stop_logging=0;
									}
									break;
								}
							}
						//Update the Roll Buffer
							update_rb();


						}

					break;
			
			}
		
		/* TIM Update event */
		if((TIM7->SR & (TIMER_UPDATE_FLAG)) == TIMER_UPDATE_FLAG)
		{
			TIM7->SR = ~(TIMER_INTERRUPT);  
		}
	}

	/*
		SIM28 Usart interrupt handler - stores the incoming GPRMC message in a string 
		When a complete message is recieved stores the data in z(GPS DATA) 
	*/
	void USART2_IRQHandler()                 
	{
		if(freq_check_success == 1 && gprmc_check_success ==1)
		{
			GPS_NMEAString();
			if(msg_process_flag == 1)
			{
				if(GPS_RxBuff[3] == 'M')
					tracker_brd |= (1<<broadcast_params);
				if(GPS_RxBuff[3] == 'G')
					tracker_brd |= (1<<(broadcast_params + 1));

				GPS_NMEAData(&z);
			}
		}
		else 
			FREQUENCY_CHECK();

		if(!get_date && gps_log_flag)
		{
			strncpy(Start_Date,z.date,6);
			get_date = 1;
		}

		USART_IntFlagClear(USART2,USART_IT_RXNE);
		NVIC_ClearPendingIRQ(USART2_IRQn);         //clear pending interrupt .....best at the start of ISR
	}

	static unsigned short crc16(const unsigned char *buf, unsigned long count)
	 {
	   unsigned short crc = 0;
	   int i;

	   while(count--) {
	     crc = crc ^ *buf++ << 8;

	     for (i=0; i<8; i++) {
	       if (crc & 0x8000) crc = crc << 1 ^ 0x1021;
	       else crc = crc << 1;
	     }
	   }
	   return crc;
	 }


	void Init_IFT()
	{
		IFT_Start = 1;
		NVIC_DisableIRQ(TIM7_IRQn);
		res= f_open(&file_down, "IFT_Da.bin", FA_CREATE_ALWAYS | FA_WRITE);

		 while(res != FR_OK)
		 {
			 f_close(&file_down);
			 res= f_open(&file_down, "IFT_Da.bin", FA_CREATE_ALWAYS | FA_WRITE);
		 }
		 memset(Data_IFT,0,sizeof(Data_IFT));
		 index_IFT = 1;
		 message_start = 0;
		 filesize_check = 0;
		 filesize_IFT = 0;
		 set_variable = 1;
	}


	void Update_Check()
		{
			res = f_open(&IFT, "server.csv", FA_OPEN_EXISTING | FA_READ);

			char temp_stor[10];

		/*** Get updates version from server.csv ***/
			memset(temp_stor,0,sizeof(temp_stor));
			f_gets(temp_stor,10,&IFT);

			memset(temp_stor,0,sizeof(temp_stor));
			f_gets(temp_stor,10,&IFT);
			sscanf(temp_stor,"%d",&stat1);

			memset(temp_stor,0,sizeof(temp_stor));
			f_gets(temp_stor,10,&IFT);
			sscanf(temp_stor,"%d",&stat2);

			memset(temp_stor,0,sizeof(temp_stor));
			f_gets(temp_stor,10,&IFT);
			sscanf(temp_stor,"%d",&stat3);

			memset(temp_stor,0,sizeof(temp_stor));
			f_gets(temp_stor,10,&IFT);
			sscanf(temp_stor,"%d",&stat4);

			memset(temp_stor,0,sizeof(temp_stor));
			f_gets(temp_stor,10,&IFT);
			sscanf(temp_stor,"%d",&stat5);

			file1 = (stat1 != pay_ver)?1:0;
			file2 = (stat2 != dbc_ver)?1:0;
			file3 = (stat3 != cmd_ver)?1:0;
			file4 = (stat4 != ecu_ver)?1:0;
			file5 = (stat5 != stm_ver)?1:0;

			res = f_close(&IFT);

			IFT_Start = 0;
			timer7_mode = 0;
			counter = 0;
			NVIC_EnableIRQ(TIM7_IRQn);
			download_timeout = (file1 == 1)?500:60;

			if(file1)
				ESP_CMD_SEND(DOWN1);
			else if(file2)
				ESP_CMD_SEND(DOWN2);
			else if(file3)
				ESP_CMD_SEND(DOWN3);
			else if(file4)
				ESP_CMD_SEND(DOWN4);
			else if(file5)
				ESP_CMD_SEND(DOWN5);
			else
				its_done = 1;

		}

	void USART3_IRQHandler()                 
	{
		USART_IntFlagClear(USART3,USART_IT_RXNE);
		NVIC_ClearPendingIRQ(USART3_IRQn);  

		char esp_ch=0x00;
		esp_ch = USART_InChar(USART3);

		if(!IFT_Start)
		{
			switch(esp_ch)
			{
				case '@':
				{

					if(update_checked == 0)
					{
						esp_counter_active = 1;
						esp_counter = 0;
						if(Update_payload && !play_once)
						{
							play_once = 1;
							ESP_CMD_SEND(PRESS_BUTTON);
							while(esp_counter < 8);
							GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 0);
							for(int i = 0; i<1000; i++);
							GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 1);
						}

						error = 0;
						download_fail = 0;
						abort_file = 0;
						esp_counter = 0;
						its_done = 0;
						down_done = 0;
						if(wifi_fail < 2)
							ESP_CMD_SEND(CHECK_UPDATES);
						IFT_Start = 0;

					}

					else
					{
						if(RTC_Wake == 2)
						{
							if(RTC_Fail <= 2)
							{
								esp_counter_active = 1;
								esp_counter = 0;
								RTC_Heartbeat(1);
							}
							else
							{

								wifi_power_off();

								esp_counter_active = 0;
								esp_counter = 0;
								RTC_Wake = 0;
								RTC_Fail = 0;
							    RTC_Heartbeat(0);
							}

						}
						else if(upload_active == 0 && upload_complete==0)
						{
							memcpy(wh[wh_counter].time,z.time,6);
							wh[wh_counter].type = 1;
							wh_counter++;
							ESP_CMD_SEND(UPLDFXS);
						}
						else if (upload_active ==1 && upload_complete==0) // Initial
						{
							wifi_fail++;
							upload_active = 0;
							upload_complete = 0;
							esp_counter_active = 0;

							memcpy(wh[wh_counter].time,z.time,6);
							wh[wh_counter].type = 7;
							wh_counter++;

							GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 0);
							for(int i = 0; i<1000; i++);
							GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 1);
						}
					}

					break;
				}
				case '%':   //No file to upload
				{
					memcpy(wh[wh_counter].time,z.time,6);
					wh[wh_counter].type = 2;
					wh_counter++;
					upload_complete = 1;
					break;
				}
				case '$':
				{
					memcpy(wh[wh_counter].time,z.time,6);
					wh[wh_counter].type = 3;
					wh_counter++;
					if(wifi_fail<3)
					{
						ESP_CMD_SEND(WIFI_ON);
						esp_counter_active = 1;
					}
					else
						esp_counter_active = 0;
					esp_counter = 0;


					break;
				}
				case '&':
				{
					memcpy(wh[wh_counter].time,z.time,6);
					wh[wh_counter].type = 4;
					wh_counter++;
					esp_counter = 0;
					esp_counter_active = 0;
					upload_active = 1;
					break;
				}
				case '#':
				{
					esp_counter = 0;
					esp_counter_active = 0;
					upload_timer_active = 1;
					upload_timer = 0;
					break;
				}
				case '+':	//turn off hotspot
				{
					memcpy(wh[wh_counter].time,z.time,6);
					wh[wh_counter].type = 6;
					wh_counter++;
					upload_active = 0;
					upload_complete = 1;
					upload_timer_active = 0;
					upload_timer = 0;
					GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 0);
					for(int i = 0; i<1000; i++);
					GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 1);
					break;
				}
				case 'C':
				{
					esp_counter_active = 0;
					esp_counter = 0;
					file_transfer_ready=1;
					break;
				}
//				case 'W':
//						counter = 0;
//						NVIC_DisableIRQ(TIM7_IRQn);
//						break;

				case '=':
						if(wifi_fail < 3)
						{
							 update_checked = 0;
							 esp_counter = 0;
							 ESP_CMD_SEND(CHECK_UPDATES);
						}
						else
							//WIFI NOT CONNECTED  -  DO OTHER OPERATIONS
							error = 5;

						wifi_fail++;
						break;

				case 'X':
						download_fail = 1;
						error++;
						break;

				case '^':
						abort_file = 1;
						error++;
						f_close(&file_down);
						break;

				case '~':
						// Transfer server.csv
						 esp_counter_active = 0;
						 esp_counter = 0;
						 timer7_mode = 0;

						 NVIC_SetPriority(USART3_IRQn,1);

					     NVIC_DisableIRQ(USART6_IRQn);
						 NVIC_DisableIRQ(USART2_IRQn);
						 NVIC_DisableIRQ(TIM7_IRQn);
						 NVIC_DisableIRQ(TIM6_DAC_IRQn);
						 NVIC_DisableIRQ(CAN1_RX1_IRQn);
						 NVIC_DisableIRQ(CAN1_RX0_IRQn);
						 for(int i=0; i<1000; i++);
						 Init_IFT();
						 update_checked = 1;
						 error = 0;
						 res = f_unlink("server.csv");
						 ESP_CMD_SEND(FILE0);
						break;

				case 'a':

						NVIC_EnableIRQ(TIM7_IRQn);

						if(download_fail)
						{
							if(error < 3)
								ESP_CMD_SEND(DOWN1);
						}
						else
						{
							download_timeout = 60;

							if(file2)
								ESP_CMD_SEND(DOWN2);
							else if(file3)
								ESP_CMD_SEND(DOWN3);
							else if(file4)
								ESP_CMD_SEND(DOWN4);
							else if(file5)
								ESP_CMD_SEND(DOWN5);
							else
							{
								down_done = 1;
								error = 0;
								Init_IFT();
								if(file1)
									ESP_CMD_SEND(FILE1);
								else if(file2)
									ESP_CMD_SEND(FILE2);
								else if(file3)
									ESP_CMD_SEND(FILE3);
								else if(file4)
									ESP_CMD_SEND(FILE4);
								else if(file5)
									ESP_CMD_SEND(FILE5);
								else
									its_done = 1;
							}
						}
						download_fail = 0;
						counter = 0;
						break;

				case 'b':

						NVIC_EnableIRQ(TIM7_IRQn);

						if(download_fail)
						{
							if(error < 3)
								ESP_CMD_SEND(DOWN2);
						}
						else
						{
							download_timeout = 60;
							if(file3)
								ESP_CMD_SEND(DOWN3);
							else if(file4)
								ESP_CMD_SEND(DOWN4);
							else if(file5)
								ESP_CMD_SEND(DOWN5);
							else
							{
								down_done = 1;
								Init_IFT();
								if(file1)
									ESP_CMD_SEND(FILE1);
								else if(file2)
									ESP_CMD_SEND(FILE2);
								else if(file3)
									ESP_CMD_SEND(FILE3);
								else if(file4)
									ESP_CMD_SEND(FILE4);
								else if(file5)
									ESP_CMD_SEND(FILE5);
								else
									its_done = 1;
							}
						}
						download_fail = 0;
						counter = 0;
						break;

				case 'c':
						NVIC_EnableIRQ(TIM7_IRQn);

						if(download_fail)
						{
							if(error < 3)
								ESP_CMD_SEND(DOWN3);
						}
						else
						{
							download_timeout = 60;
							if(file4)
								ESP_CMD_SEND(DOWN4);
							else if(file5)
								ESP_CMD_SEND(DOWN5);
							else
							{
								down_done = 1;
								Init_IFT();
								if(file1)
									ESP_CMD_SEND(FILE1);
								else if(file2)
									ESP_CMD_SEND(FILE2);
								else if(file3)
									ESP_CMD_SEND(FILE3);
								else if(file4)
									ESP_CMD_SEND(FILE4);
								else if(file5)
									ESP_CMD_SEND(FILE5);
								else
									its_done = 1;
							}
						}
						download_fail = 0;
						counter = 0;
						break;

				case 'd':
						NVIC_EnableIRQ(TIM7_IRQn);

						if(download_fail)
						{
							if(error < 3)
								ESP_CMD_SEND(DOWN4);
						}
						else
						{
							download_timeout = 60;
							if(file5)
								ESP_CMD_SEND(DOWN5);
							else
							{
								down_done = 1;
								Init_IFT();
								if(file1)
									ESP_CMD_SEND(FILE1);
								else if(file2)
									ESP_CMD_SEND(FILE2);
								else if(file3)
									ESP_CMD_SEND(FILE3);
								else if(file4)
									ESP_CMD_SEND(FILE4);
								else if(file5)
									ESP_CMD_SEND(FILE5);
								else
									its_done = 1;
							}
						}
						download_fail = 0;
						counter = 0;
						break;

				case 'e':
						NVIC_EnableIRQ(TIM7_IRQn);

						if(download_fail)
						{
							if(error < 3)
								ESP_CMD_SEND(DOWN5);
						}
						else
						{
							down_done = 1;
							Init_IFT();
							if(file1)
								ESP_CMD_SEND(FILE1);
							else if(file2)
								ESP_CMD_SEND(FILE2);
							else if(file3)
								ESP_CMD_SEND(FILE3);
							else if(file4)
								ESP_CMD_SEND(FILE4);
							else if(file5)
								ESP_CMD_SEND(FILE5);
							else
								its_done = 1;

						}
						download_fail = 0;
						counter = 0;
						break;

				case 'A':
						if(abort_file)
						{
							if(error < 3)
							{
								Init_IFT();
								ESP_CMD_SEND(FILE0);
							}
						}
						else
						{
							down_status = 1;
							download_fail = 0;
							error = 0;
						}
						abort_file = 0;
						break;


				case 'B':
						if(!abort_file)
						{
							if(!file2 && !file3 && !file4 && !file5)
							{
								its_done = 1;
								break;
							}

							 Init_IFT();

							if(file2)
								ESP_CMD_SEND(FILE2);
							else if(file3)
								ESP_CMD_SEND(FILE3);
							else if(file4)
								ESP_CMD_SEND(FILE4);
							else if(file5)
								ESP_CMD_SEND(FILE5);
							else
								its_done = 1;
						}
						else
						{
							if(error < 3)
							{
								Init_IFT();
								ESP_CMD_SEND(FILE1);
							}
						}
						abort_file = 0;
						break;

				case 'G':
						if(!abort_file)
						{
							if(!file3 && !file4 && !file5)
							{
								its_done = 1;
								break;
							}

							Init_IFT();

							if(file3)
								ESP_CMD_SEND(FILE3);
							else if(file4)
								ESP_CMD_SEND(FILE4);
							else if(file5)
								ESP_CMD_SEND(FILE5);
							else
								its_done = 1;

						}
						else
						{
							if(error < 3)
							{
								Init_IFT();
								ESP_CMD_SEND(FILE2);
							}
						}
						abort_file = 0;
						break;

				case 'D':
						if(!abort_file)
						{
							if(!file4 && !file5)
							{
								its_done = 1;
								break;
							}
							Init_IFT();

							if(file4)
								ESP_CMD_SEND(FILE4);
							else if(file5)
								ESP_CMD_SEND(FILE5);
							else
								its_done = 1;

						}
						else
						{
							if(error < 3)
							{
								Init_IFT();
								ESP_CMD_SEND(FILE3);
							}
						}
						abort_file = 0;
						break;

				case 'E':
						if(!abort_file)
						{
							if(!file5)
							{
								its_done = 1;
								break;
							}

							Init_IFT();
 							if(file5)
								ESP_CMD_SEND(FILE5);
 							else
 								its_done = 1;
						}
						else
						{
							if(error < 3)
							{
								Init_IFT();
								ESP_CMD_SEND(FILE4);
							}
						}
						abort_file = 0;
						break;

				case 'F' :
						if(abort_file)
						{
							if(error < 3)
							{
								Init_IFT();
								ESP_CMD_SEND(FILE0);
							}
						}
						else
						{
							// IFT Done
							abort_file = 0;
							its_done = 1;
							IFT_Start = 0;
						}
						abort_file = 0;
						break;
				/* RTC HEARTBEAT FAILURE */
				case '!':
					//RESET ESP COUNTER
					esp_counter=0;
					esp_counter_active=0;
					RTC_Fail ++;
					//send command for hearbeat again try for 2 times
					if(RTC_Fail<= 2)
					{
						GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 0);
						for(int i=0; i<1000; i++);
						GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 1);
						for(int i=0; i<10000; i++);
						//RTC_Heartbeat(1);
					}
					else
					{
						//store the data by creating a file
						RTC_Wake = 0;
						RTC_Fail = 0;
						wifi_power_off();




						//DONGLE SWITCH OFF HERE
					}

					break;
                /* RTC HEARTBEAT SUCCESS */
				case 'R':
					RTC_Fail = 0;//RESET RTC FAIL FLAG TO 0
					wifi_power_off();
					RTC_Wake = 0;




					//DONGLE SWITCH OFF HERE
					//RESET ESP COUNTER
					esp_counter=0;
					esp_counter_active=0;
					//RESET ESP
					GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 0);
					for(int i=0; i<1000; i++);
					GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 1);
					for(int i=0; i<1000; i++);

					break;
				default:
					break;
			}

			if(esp_ch == 'T' || esp_ch == 't')
			{
				esp_filename_start = 1;
				esp_filename_counter = 0;
				esp_lastfilename[esp_filename_counter++] = esp_ch;
			}
			else if(esp_filename_start)
			{
				esp_lastfilename[esp_filename_counter++] = esp_ch;
				if(esp_filename_counter == 12)
				{
					esp_counter_active = 0;
					esp_counter = 0;
					esp_filename_start = 0;
					esp_filename_wait = 0;
				}
			}

		}
		else
		{
			switch(message_start)
			{
				case 0:
						switch (esp_ch)
						{
						  case SOH:
								    packet_size = PACKET_SIZE;
								    message_start = 1;
								    Data_IFT[0] = SOH;
								    count = 133;

								    break;

						  case STX:
								    packet_size = PACKET_1K_SIZE;
								    message_start = 1;
								    Data_IFT[0] = STX;
								    count = 1029;

								    break;

						  case EOT:
							  	  	filesize_check = f_size(&file_down);
								    res = f_close(&file_down);
								    if(filesize_check == filesize_IFT)
								    {
								    	res = f_unlink(orig_name);
								    	res = f_rename("IFT_Da.bin", orig_name);
								    }
								    else
								    {
								    	abort_file = 1;
								    }
									IFT_Start = 0;
								    USART_OutChar(USART3, CRC16);
								    break;

						  case CA:
							  	  	  	  	  	  	  	  	  	  // USER ABORT


						  default:								// Sync Error - how to handle??
						      		message_start = 0;
						      		memset(Data_IFT,0,sizeof(Data_IFT));
						      		break;
						}

						break;

				case 1:
						Data_IFT[index_IFT++] = esp_ch;
						if(index_IFT == 3)
						{
							if((Data_IFT[1] + Data_IFT[2]) == 0xFF)
								message_start = 2;
							else
								message_start = 0;
						}

						break;

				case 2:
						Data_IFT[index_IFT++] = esp_ch;

						if(count == index_IFT)
						{
							message_start = 0;
							index_IFT = 1;

							if(crc16(&Data_IFT[3], packet_size + PACKET_TRAILER) == 0)	// Success Packet
							{
								if(set_variable)
								{
									set_variable = 0;
									int i,j;
									for(i = PACKET_HEADER; (Data_IFT[i] != 0) && (i < PACKET_SIZE); i++)	//
									{
										orig_name[i-PACKET_HEADER] = Data_IFT[i];
									}
									orig_name[i-PACKET_HEADER] = '\0';
									i++;
									for(j=0; (Data_IFT[i] != 0) && (i < PACKET_SIZE); i++, j++)
									{
										size_IFT[j] = Data_IFT[i];
									}
									size_IFT[j] = '\0';
									int k = j;
									for(int i=0; i<k; i++)
									{
										a_bit = ascii_to_hex(size_IFT[i]);
										filesize_IFT += a_bit*(pow(10,--j));
									}

									memset(Data_IFT,0,sizeof(Data_IFT));

									USART_OutChar(USART3, CRC16);

								}
								else
								{
									file_size = f_size(&file_down);

									if((filesize_IFT - file_size)>128)
										res = f_write(&file_down, &Data_IFT[3], packet_size, &bw);
									else
										res = f_write(&file_down, &Data_IFT[3], (filesize_IFT - file_size), &bw);

									while(res != FR_OK)
									{
										f_close(&file_down);
										for(int p = 0;p < 100; p++);
										res = f_open(&file_down, "IFT_Da.bin", FA_OPEN_EXISTING | FA_WRITE);
										res = f_lseek(&file_down, file_size);
										res = f_write(&file_down, &Data_IFT[3], packet_size, &bw);
									}
									f_sync(&file_down);

									if(IWDG_SET)
										TM_WATCHDOG_Reset();

									memset(Data_IFT,0,sizeof(Data_IFT));
									USART_OutChar(USART3, CRC16);

								}

							}
							else
							{
								// CRC Fail
								USART_OutChar(USART3,NAK);
							}
						}
						break;
					}
		}

	}
	/*
		Internal File transfer 
	*/
	int file_transmit(char* filenamextc, char* CMD)
	{
		res=f_stat(filenamextc, &fno);
		if(res!=FR_OK)
		{ 
			return -1; 
		}
		file_transfer_ready=0;
		delay(1000);
		NVIC_EnableIRQ(USART3_IRQn);
		ESP_CMD_SEND(CMD);

		esp_counter_active = 1;
		NVIC_EnableIRQ(TIM7_IRQn);
		esp_counter = 0;
		TM_WATCHDOG_Reset();
		while (file_transfer_ready==0 && esp_counter< 25)
		{}
		if(file_transfer_ready==0)
		{
			ymodem_error=1;
			return -1;
		}
		esp_counter_active = 0;
		NVIC_DisableIRQ(USART3_IRQn);
		NVIC_DisableIRQ(TIM7_IRQn);
		TM_WATCHDOG_Reset();

			res = f_open(&file, filenamextc, FA_OPEN_EXISTING|FA_READ);
			if(res!=FR_OK) return -1;
			int status = 1;
			status = Ymodem_Transmit(&file, (uint8_t *)filenamextc, f_size(&file));

		do
		{
			res = f_close(&file);
		}while(res!=FR_OK);

		if(status==0 || status==0x0E)
			ymodem_error=0;
		else
		{  
			ymodem_error=1;
			return 0xFF;
		}
		return ((ymodem_error == 0)?1:0);
	}
	void ESPTRANSMITx(void)
	{ 

		NVIC_EnableIRQ(USART3_IRQn);
		esp_filename_wait = 1;
		ESP_CMD_SEND(LAST_FILE);

		esp_counter_active = 1;
		esp_counter = 0;
		IFT_Start = 0;
		
		while( (esp_filename_start == 1 || esp_filename_wait == 1) && esp_counter< ESP_COUNTER_TIMEOUT)
		{

		}
		esp_counter_active = 0;
		IFT_active = 1;
		if (esp_filename_start != 1 && esp_filename_wait != 1)
		{
			NVIC_DisableIRQ(USART3_IRQn);
			NVIC_DisableIRQ(TIM7_IRQn);
			NVIC_DisableIRQ(USART6_IRQn);
			NVIC_DisableIRQ(USART2_IRQn);
			NVIC_DisableIRQ(TIM6_DAC_IRQn);
			NVIC_DisableIRQ(CAN1_RX1_IRQn);
			NVIC_DisableIRQ(CAN1_RX0_IRQn);

			do
			{
				if(ymodem_error==0)
				{
					IFT_filename = TEMP_FILENAME_HANDLE(esp_lastfilename);
					IFT_fail=0;
				}
				if(file_transmit(IFT_filename, FILE_TRANSFER) !=1)
				{
					IFT_fail++;
				}
				else
				{
					IFT_complete=0;
					IFT_fail=0;
				}
				
				NVIC_EnableIRQ(CAN1_RX1_IRQn);
				NVIC_EnableIRQ(CAN1_RX0_IRQn);
				delay(100);

				if((temp_file_counter%1000000)/100000 + 0x30 <= IFT_filename[2])
				{
					if((temp_file_counter%100000)/10000 + 0x30 <= IFT_filename[3])
					{
						if((temp_file_counter%10000)/1000 + 0x30 <= IFT_filename[4])
						{
							if((temp_file_counter%1000)/100 + 0x30 <= IFT_filename[5])
							{
								if((temp_file_counter%100)/10 + 0x30 <= IFT_filename[6])
								{
									if(temp_file_counter%10 + 0x30 <= IFT_filename[7])
									{
										exception_01=1;
									}
								}
							}
						}
					}
				}
				if(IGN_ON != 0)
				{
					NVIC_DisableIRQ(CAN1_RX1_IRQn);
					NVIC_DisableIRQ(CAN1_RX0_IRQn);
				}
			}while(exception_01==0 && IGN_ON==0 && IFT_fail<3);

			if(exception_01 == 1)
				IFT_complete = 1;
		}
	}
	//RTC HANDLER
	void RTC_WKUP_IRQHandler(void)
	{


		wifi_power_on();

		tempregister = (uint32_t)(RTC->ISR & RTC_ISR_WUTF);

		EXTI->PR |= EXTI_PR_PR22;
		NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);

		RTC_Auto_Wakeup_Unit_Reset();

		if(STM_SLEEP)
		{
			TM_WATCHDOG_Init(TM_WATCHDOG_Timeout_32s);
		    IWDG_SET=1;
			//mount card
			RCC_GPIOA_CLOCK_ENABLE_CLK();
			GPIO_Config_Mode(GPIOA, GPIO_PIN_7, GPIO_MODE_OUT);
			GPIO_Config_OType(GPIOA, GPIO_PIN_7, GPIO_OTYPE_PP);
			GPIO_Config_Speed(GPIOA, GPIO_PIN_7, GPIO_LOW_SPEED);
			GPIO_Write_Bit(GPIOA, GPIO_PIN_7, 1);
			//Initialize pins and SDIO peripheral
			SD_SDIOInit();
			for (int i = 0; i < 1000; i++)
				;

			//FATFS
			/* Register work area */
			res = f_mount(&fs, "", 0);
			for (int i = 0; i < 1000; i++)
				;

			res = f_open(&boot_config, "config.csv", FA_OPEN_ALWAYS | FA_WRITE | FA_READ);

			int k = 10;
			while (k)
			{
				memset(temp_buf, 0, sizeof(temp_buf));
				f_gets(temp_buf, 20, &boot_config);
				k--;
			}
			res = f_write(&boot_config, "1", 1, &bw);

			res = f_close(&boot_config);


			RETFROM_SLEEP();
			//RETFROM_SLEEP();//SO THAT DEVICE DOES NOT REENTER SLEEP
		}
        rtc_counter ++;
		RTC_Wake = 1;


	}
	void USART6_IRQHandler()                 //
	{
	 //for rx handler

		b = USART_InHex(USART6);
		uint8_t i;
		USART_IntFlagClear(USART6,USART_IT_RXNE);
		NVIC_ClearPendingIRQ(USART6_IRQn);

		//Store Message in KLineRx_Buff
		switch(msg_status)
		{
			case 0:		//Determine Length of message based on Format Byte (Refer ISO 14230-2)
			{
				if((b&0xC0) == 0x80 || (b&0xC0) ==0xC0 )
				{
					if((b&0x3F) == 0x0) //Additional Data byte present 
					{
						msg_status=1;
						Kline_RxBuff[kline_cnt++] = b;
						check_calc += b;
					}
					else // Data info available ob the LSB 6bits of byte 1 
					{
						kline_len = b&0x3F; //length of data
						buff_length = kline_len+4; //total length of msg ( 1 FRMT Byte , 1 Dest Adress, 1 Src Adress , 1 CS) 
						Kline_RxBuff[kline_cnt++] = b;
						check_calc += b;
						msg_status=3;
					}
				}
				else if ((b&0xC0) == 0x0)
				{
					if((b&0x3F) == 0x0) //Additional Data byte present no address info
					{
						msg_status=2;
						Kline_RxBuff[kline_cnt++] = b;
						check_calc += b;
					}
				}
				break;
			}
			case 1: //Additional Data Byte with Address Information 
			{
				kline_len=b;
				buff_length = kline_len+5; //Total length of msg ( 1 FRMT Byte , 1 Length Byte, 1 Dest Adress, 1 Src Adress , 1 CS) 
				Kline_RxBuff[kline_cnt++] = b;
				check_calc += b;
				break;
			}
			
			case 2: //Additional Data byte present no address info
			{
				kline_len=b;
				buff_length = kline_len+3; //Total length of msg ( 1 FRMT Byte , 1 Length Byte, 1 CS) 
				Kline_RxBuff[kline_cnt++] = b;
				check_calc += b;
				msg_status=3;
				break;					
			}
			case 3: //Body of Message 
			{
				if((buff_length-1) != kline_cnt)
				{
					Kline_RxBuff[kline_cnt++] = b;
					check_calc += b;
				}
				else
				{
					check_uart = b;
					Kline_RxBuff[kline_cnt++] = b;
					msg_status = 0;
					kline_cnt = 0;

					if(check_calc == check_uart)
						kline_process_flag = 1;
					else
						memset(Kline_RxBuff,0,sizeof(Kline_RxBuff));
					check_calc = 0;
					check_uart = 0;
				}
				break;
			}
			default:
				break;
		}

		if(kline_process_flag == 1 && kline_init == 3)
		{
			kline_process_flag = 0;
			for(int r = 0; r < 7; r++)
			{
				if(Kline_RxBuff[r] == POS_RESPONSE[r])
					kline_init = 4;
				else
				{
					kline_init = 0; break;
				}
			}
			if (kline_init == 4)
			{
				kline_inactive=0;
				memset(Kline_RxBuff,0,sizeof(Kline_RxBuff));
				kline_len=0;
				TIM6_Base_Start_IT();
				NVIC_EnableIRQ(TIM6_DAC_IRQn);
			}
			else
				KLINE_REINIT();

		}

		if(kline_process_flag == 1 && kline_init == 4)
		{
			kline_process_flag = 0;
			i=last_req;
			
			if(Kline_RxBuff[3]==(arr_req_param[i].service+0x40))
			{
				if(Kline_RxBuff[4]==arr_req_param[i].PID)
				{
					switch(arr_req_param[i].length)
					{
						case 32:  //0,1,6,7
						{

						}
						case 14:  //0,1,6,7
						{

						}
						case 16:  //0,1,6,7
						{
							Data[last_req+broadcast_params][0] = Kline_RxBuff[5];
							Data[last_req+broadcast_params][1] = Kline_RxBuff[6];
							Data[last_req+broadcast_params][2] = Kline_RxBuff[11];
							Data[last_req+broadcast_params][3] = Kline_RxBuff[12];
							break;
						}
						case 12:  //0,1
						{

						}
						case 8:   //0,1
						{

						}
						case 2:	  //0,1
						{
							Data[last_req+broadcast_params][0] = Kline_RxBuff[5];
							Data[last_req+broadcast_params][1] = Kline_RxBuff[6];
							break;
						}
						case 1:  //0
						{
							Data[last_req+broadcast_params][0] = Kline_RxBuff[5];
							break;
						}
						default:
							break;
					}
					tracker_req |= (1<<last_req);
					last_req++;
					if(last_req<req_params)
					{
						TIM6_Base_Start_IT();
						NVIC_EnableIRQ(TIM6_DAC_IRQn);
					}

				}
			}
			
			kline_inactive=0;
			memset(Kline_RxBuff,0,sizeof(Kline_RxBuff));
			kline_len=0;
		}

	}	

	void CAN1_RX0_IRQHandler()                 //CAN1 FIFO0 ISR 
	{
		CAN_Receive(CAN1,0,&rx_can);
		NVIC_ClearPendingIRQ(CAN1_RX0_IRQn);         //clear pending interrupt .....best at the start of ISR 
		uint8_t i,j,k,l;
		if(IGN_ON==0)
		{
			IGN_ON=1;
			if(upload_active == 1 || IFT_active == 1)
			{
				//we need to close our file here
				for(uint32_t x = 0; x < 2000000; x++);
				RETFROM_SLEEP();
			}
			NVIC_EnableIRQ(USART2_IRQn);
			NVIC_EnableIRQ(USART6_IRQn);
			NVIC_EnableIRQ(TIM7_IRQn);
			NVIC_EnableIRQ(CAN1_RX0_IRQn);
			NVIC_EnableIRQ(CAN1_RX1_IRQn);
			//Watchdog Initialised
				TM_WATCHDOG_Init(TM_WATCHDOG_Timeout_32s);
				IWDG_SET=1;
			//Variables Reset
				write_once = 0;
				do_once = 0;
				IFT_complete = 0;
				IFT_fail = 0;
				tracker_req = 0;
				tracker_brd = 0;

		}
		if(STM_SLEEP)
		{
			//WAKE Up procedure from sleep 
			RETFROM_SLEEP();
		}
		can_inactive=0;
		if( (rx_can.ExtId==response_can_id[0]) || (rx_can.StdId==response_can_id[0])
		 		|| (rx_can.ExtId==response_can_id[1]) || (rx_can.StdId==response_can_id[1]) )
		{
			//Update the params
			i = last_req;
					if(rx_can.Data[1]==(arr_req_param[i].service+0x40))
					{
						if(rx_can.Data[2]==arr_req_param[i].PID)
						{
							if(arr_req_param[i].can_extn_param)
							{
								if(rx_can.Data[3]==arr_req_param[i].PID1)
								{
									for(j=0;j<arr_req_param[i].length;j++)
									{
										Data[last_req+broadcast_params][j] = rx_can.Data[j+4];
									}
									last_req++;
									tracker_req |= (1<<last_req);
								}

							}
							else
							{
								for(j=0;j<arr_req_param[i].length;j++)
								{
									Data[last_req+broadcast_params][j] = rx_can.Data[j+3];
								}
								last_req++;
								tracker_req |= (1<<last_req);
							}
						}
					}
			//Request Params 
				if(last_req<req_params)
				{
					if(arr_req_param[last_req].log_freq==0)
					{
						if(arr_req_param[last_req].can_extn_param)
							tx_can.Data[0]=0x03;	
						else
							tx_can.Data[0]=0x02;			

						tx_can.Data[1]=arr_req_param[last_req].service;
						tx_can.Data[2]=arr_req_param[last_req].PID;
						
						if(arr_req_param[last_req].can_extn_param)
							tx_can.Data[3]=arr_req_param[last_req].PID1;									// change tx_can data
						
						tx_can.ExtId = request_can_id[arr_req_param[last_req].can_id_no];

						while(CAN_Which_MailboxIsEmpty(CAN1)==0x0F)
						{}
						CAN_Transmit(CAN1,&tx_can,CAN_Which_MailboxIsEmpty(CAN1));
					}

				}
		}
		else
		{
			j=0;
			while(j<pgn[rx_can.FMI].num_params)
			{
				l=pgn[rx_can.FMI].param_strt+j;
				i=arr_brd_param[l].position-1;
				k=0;
				while(i<arr_brd_param[l].position+arr_brd_param[l].length-1)
				{
					Data[l][k] = rx_can.Data[i];
					i++;
					k++;
				}
				j++;
				tracker_brd |= (1<<l);
			}
			CAN_FilterDeActivate(rx_can.FMI);
		}
		CAN_FIFORelease(CAN1, 0);	                   //release fifo
	}	

	void CAN1_RX1_IRQHandler()                 //CAN1 FIFO0 ISR 
	{
		CAN_Receive(CAN1,1,&rx_can);
		NVIC_ClearPendingIRQ(CAN1_RX1_IRQn);         //clear pending interrupt .....best at the start of ISR 
		uint8_t i,j,k,l;
		if(IGN_ON==0)
		{
			IGN_ON=1;
			if(upload_active == 1 || IFT_active == 1)
			{
				for(uint32_t x = 0; x < 2000000; x++);
				RETFROM_SLEEP();
			}
			NVIC_EnableIRQ(USART2_IRQn);
			NVIC_EnableIRQ(USART6_IRQn);
			NVIC_EnableIRQ(TIM7_IRQn);
			NVIC_EnableIRQ(CAN1_RX0_IRQn);
			NVIC_EnableIRQ(CAN1_RX1_IRQn);
			//Watchdog Initialised
				TM_WATCHDOG_Init(TM_WATCHDOG_Timeout_32s);
				IWDG_SET=1;
			//Variables Reset
				write_once = 0;
				do_once = 0;
				IFT_complete = 0;
				IFT_fail = 0;
				tracker_brd = 0;
				tracker_req = 0;

		}
		if(STM_SLEEP)
		{
			//WAKE Up procedure from sleep 
			RETFROM_SLEEP();
		}
		can_inactive=0;
		if( (rx_can.ExtId==response_can_id[0]) || (rx_can.StdId==response_can_id[0])
		 		|| (rx_can.ExtId==response_can_id[1]) || (rx_can.StdId==response_can_id[1]) )
		{
			//Update the params
			i = last_req;
					if(rx_can.Data[1]==(arr_req_param[i].service+0x40))
					{
						if(rx_can.Data[2]==arr_req_param[i].PID)
						{
							if(arr_req_param[i].can_extn_param)
							{
								if(rx_can.Data[3]==arr_req_param[i].PID1)
								{
									for(j=0;j<arr_req_param[i].length;j++)
									{
										Data[last_req+broadcast_params][j] =rx_can.Data[j+4];
									}
									tracker_req |= (1<<last_req);
									last_req++;
								}

							}
							else
							{
								for(j=0;j<arr_req_param[i].length;j++)
								{
									Data[last_req+broadcast_params][j] =rx_can.Data[j+3];
								}
								tracker_req |= (1<<last_req);
								last_req++;
							}
						}
					}
				
			//Request Params 
				if(last_req<req_params)
				{
					if(arr_req_param[last_req].log_freq==0)
					{
						if(arr_req_param[last_req].can_extn_param)
							tx_can.Data[0]=0x03;	
						else
							tx_can.Data[0]=0x02;			

						tx_can.Data[1]=arr_req_param[last_req].service;
						tx_can.Data[2]=arr_req_param[last_req].PID;
					
						if(arr_req_param[last_req].can_extn_param)
							tx_can.Data[3]=arr_req_param[last_req].PID1;

						tx_can.ExtId = request_can_id[arr_req_param[last_req].can_id_no];

						while(CAN_Which_MailboxIsEmpty(CAN1)==0x0F)
						{}
						CAN_Transmit(CAN1,&tx_can,CAN_Which_MailboxIsEmpty(CAN1));
					}
				}
		}
		else
		{
			j=0;
			while(j<pgn[rx_can.FMI+fifo1_pgn].num_params)
			{
				l=pgn[rx_can.FMI+fifo1_pgn].param_strt+j;
				i=arr_brd_param[l].position-1;
				k=0;
				while(i<arr_brd_param[l].position+arr_brd_param[l].length-1)
				{
					Data[l][k] = rx_can.Data[i];
					i++;
					k++;
				}
				j++;
				tracker_brd |= (1<<l);
			}
			CAN_FilterDeActivate(rx_can.FMI);
		}
		CAN_FIFORelease(CAN1,1);	                   //release fifo
	}
	void LOW_POWER_MODE(void)
	{
		wifi_power_off();
		//DISABLE ALL OTHER INTERRUPTS
		NVIC_DisableIRQ(USART2_IRQn);
		NVIC_DisableIRQ(USART3_IRQn);
		NVIC_DisableIRQ(TIM6_DAC_IRQn);
		NVIC_DisableIRQ(TIM7_IRQn);
		
		NVIC_EnableIRQ(CAN1_RX1_IRQn);
		NVIC_EnableIRQ(CAN1_RX0_IRQn);
		NVIC_EnableIRQ(RTC_WKUP_IRQn);
		//Power off Peripherals 
			// ESP
				GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 0);
				USART_Disable(USART3);
			//GPS Module 
				GPIO_Write_Bit(GPIOC, GPIO_PIN_3,0);	
				USART_Disable(USART2);
			//SD CARD
				GPIO_Write_Bit (GPIOA, GPIO_PIN_7,0);	
			
		/* Configure low-power mode */
   		 SCB->SCR &= ~( SCB_SCR_SLEEPDEEP_Msk );  // low-power mode = sleep mode
    	SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;     // reenter low-power mode after ISR
     
    	__WFI();  // enter low-power mode
	}

	void startup()
	{
		//Clock Setup
			InitializeClock(); 
			SystemCoreClockUpdate();
			
		//For delay
			uint32_t x= SysTick_Config(SystemCoreClock /1000);      /* Configure SysTick to generate an interrupt every millisecond */
			response_can_id[0]=0xFFFFFFFF;
			response_can_id[1]=0xFFFFFFFF;
			request_can_id[0]=0xFFFFFFFF;
			request_can_id[1]=0xFFFFFFFF;
		// Mount SD card and Check for Config file 
			wifi_init();
			wifi_power_on();
//			wifi_power_off();
			mount_card();
			config_file();
			load_dbc();
		//GPS
			//Initializes communincation with SIM28 at the configured baud rate and sets the USART2 Interrupt
			GPS_Init();
			delay(50);

			GPS_PMTKCmd(FULLCOLD_RESTART);
			delay(50);


			GPS_PMTKCmd(SET_BAUDRATE_9600);   //send the command  to only receive GPRMC string
			delay(50);
			GPS_PMTKCmd(UPDATE_1HZ);
			delay(50);
			while(gprmc_check_success == 0)
			{
				GPS_PMTKCmd(GPRMCGGA);   //send the command  to only receive GPRMC & GPGGA string
			    delay(50);
			}
			while(freq_check_success == 0)
			{
				GPS_PMTKCmd(UPDATE_1HZ);
			    delay(50);
			}
			//do z.altitude memset 
			memset(z.altitude,'0',sizeof(z.altitude));

		//Initiate the DAQ module 
			init_daq();

		//Initiate timers

			TIM7_Init();
			TIM7_Base_Start_IT();
			NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);   //clear any pending interrupt
			NVIC_SetPriority(RTC_WKUP_IRQn,6);
			NVIC_EnableIRQ(RTC_WKUP_IRQn);
		    rtc_timeout_gap = RTC_WAKEUP_TIMEOUT;
				//if(read_rtc_status())
					resrtc = RTC_Auto_Wakeup_Unit_Init(rtc_timeout_gap);
			//Conditions after watchdog system reset
				esp_counter = 0;
				esp_counter_active = 1;
				while(1)
				{
					if(IWDG_SET || (RTC_Wake == 1))
					{
						res = f_open(&RTC_file, "config.csv", FA_OPEN_ALWAYS | FA_WRITE | FA_READ);

						int k = 10;
						while (k)
						{
							memset(temp_buf, 0, sizeof(temp_buf));
							f_gets(temp_buf, 20, &RTC_file);
							k--;
						}
						res = f_write(&RTC_file, "0", 1, &bw);

						res = f_close(&RTC_file);

						esp_counter = 0;
						esp_counter_active = 0;
						break;
					}
					if(esp_counter > 30)
					{
						STM_SLEEP = 1;
						LOW_POWER_MODE();
					}
				}
		//Create a new Temp File
			filename_flag = 0;
			if(!RTC_Wake)
			{
				new_file();
			}

			ESP_Init();
			
	}
//File Handling 
	void write_csv()
	{
			/*
				Check if Rolling Buffer1 is nearing capacity and write it to the SD card
			*/
			if (rb_flag==1)
			{
				rb_flag=2;
				if(res == FR_OK)
					file_size = f_size(&file);
				res=f_write(&file,rb1,rb1_size,&bw);
				f_sync(&file);
				//If File Crashes close and open file and rewrite
				if ((file_size + rb1_size != f_size(&file)) || (res!=FR_OK) )
				{
					do{
						res = f_close(&file);
						GPIO_Write_Bit (GPIOA, GPIO_PIN_7,0);
						for(int p = 0;p < 100; p++);
						mount_card();
						res=f_open(&file,filename, FA_OPEN_EXISTING|FA_WRITE);
						res = f_lseek(&file, file_size);
						res=f_write(&file,rb1,rb1_size, &bw);
						f_sync(&file);
					}while(res!=FR_OK);
				}
				rb1_size=0;
				memset(rb1,0,sizeof(rb1));
			}
			/*
				Check if Rolling Buffer2 is nearing capacity and write it to the SD card
			*/
			else if (rb_flag==2)
			{
				rb_flag=1;
				if(res == FR_OK)
					file_size = f_size(&file);
				res=f_write(&file,rb2,rb2_size,&bw);
				f_sync(&file);
				//If File Crashes close and open file and rewrite
				if ((file_size + rb2_size != f_size(&file)) || (res!=FR_OK) )
				{
					do{
						res = f_close(&file);
						GPIO_Write_Bit (GPIOA, GPIO_PIN_7,0);
						for(int p = 0;p < 100; p++);
						mount_card();
						res=f_open(&file,filename, FA_OPEN_EXISTING|FA_WRITE);
						res = f_lseek(&file, file_size);
						res=f_write(&file,rb2,rb2_size, &bw);
						f_sync(&file);
					}while(res!=FR_OK);
				}
				rb2_size=0;
				memset(rb2,0,sizeof(rb2));
			}
	}	

	void RETFROM_SLEEP(void)
	{
   		SCB->SCR &= 0x00000000;     // reenter low-power mode after ISR
		SCB->AIRCR=0x05fa0004;  
	}

int main()
{
	startup();

	int q = 0, l = 0;
	char config_buffer[20];

/*	Add these to start updates check - MZ
	ESP_CMD_SEND(CHECK_UPDATES);
	IFT_Start = 0;

*/
	while(1)
	{

		if(update_checked == 1)
		{
			while(1)
			{
				if(down_status)
				{
					down_status = 0;
					Update_Check();
				}

				if(counter > download_timeout)
				{
					error = 5;
				}

				if(error >= 3)
				{
					update_checked = 3;
					timer7_mode = 1;
					counter = 0;
					NVIC_EnableIRQ(USART6_IRQn);
					NVIC_EnableIRQ(USART2_IRQn);
					NVIC_EnableIRQ(TIM7_IRQn);
					NVIC_EnableIRQ(TIM6_DAC_IRQn);
					NVIC_EnableIRQ(CAN1_RX1_IRQn);
					NVIC_EnableIRQ(CAN1_RX0_IRQn);
					NVIC_SetPriority(USART3_IRQn,7);
					break;
				}

				if(IWDG_SET)
					TM_WATCHDOG_Reset();

				if(its_done == 1)
				{
				    // Update config file here
					if(file1 || file2 || file3 || file4 || file5)
					{
						char temp_version[5];
						res = f_open(&filex, "config.csv", FA_WRITE|FA_READ);

						memset(config_buffer,0,sizeof(config_buffer));
						f_gets(config_buffer,20,&filex);
						memset(config_buffer,0,sizeof(config_buffer));
						f_gets(config_buffer,20,&filex);

						if(file5)
							res = f_write(&filex, "1\nSTM.bin\n",10, &bw);
						else
						{
							memset(config_buffer,0,sizeof(config_buffer));
							f_gets(config_buffer,20,&filex);
							memset(config_buffer,0,sizeof(config_buffer));
							f_gets(config_buffer,20,&filex);
						}

						if(file1)
							res = f_write(&filex, "1\n", 2, &bw);
						else
						{
							memset(config_buffer,0,sizeof(config_buffer));
							f_gets(config_buffer,20,&filex);
						}


						memset(temp_version,0,sizeof(temp_version));
						sprintf(temp_version, "%02d", stat1);
						strncat(temp_version,"\n",1);

						res = f_write(&filex, temp_version , 3, &bw);

						memset(temp_version,0,sizeof(temp_version));
						sprintf(temp_version, "%02d", stat2);
						strncat(temp_version,"\n",1);

						res = f_write(&filex, temp_version , 3, &bw);

						memset(temp_version,0,sizeof(temp_version));
						sprintf(temp_version, "%02d", stat3);
						strncat(temp_version,"\n",1);

						res = f_write(&filex, temp_version , 3, &bw);

						memset(temp_version,0,sizeof(temp_version));
						sprintf(temp_version, "%02d", stat4);
						strncat(temp_version,"\n",1);

						res = f_write(&filex, temp_version , 3, &bw);

						memset(temp_version,0,sizeof(temp_version));
						sprintf(temp_version, "%02d", stat5);
						strncat(temp_version,"\n",1);

						res = f_write(&filex, temp_version , 3, &bw);

						f_close(&filex);

						if(file2 || file5)
						{
							// Software Reset here
							if(filename_flag == 1)
							{
								memset(file_ending,0,sizeof(file_ending));
								sprintf(file_ending, "CODE UPDATED-STM:%02d & DBC:%02d\n",stat5,stat2);
								res = f_write(&file,file_ending,30,&bw);

								l = 0;
								do
								{
									res = f_close(&file);
									l++;
								}while(res != FR_OK && l < 3);
							}
							SCB->AIRCR=0x05fa0004;
						}
					}
					update_checked = 2;
					timer7_mode = 1;

					NVIC_EnableIRQ(USART6_IRQn);
					NVIC_EnableIRQ(USART2_IRQn);
					NVIC_EnableIRQ(TIM7_IRQn);
					NVIC_EnableIRQ(TIM6_DAC_IRQn);
					NVIC_EnableIRQ(CAN1_RX1_IRQn);
					NVIC_EnableIRQ(CAN1_RX0_IRQn);

					NVIC_SetPriority(USART3_IRQn,7);

					GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 0);
					for(int i=0; i<1000; i++);
					GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 1);

					break;
				}
			}

		}
		if(data_log_mode != copy_of_data_log_mode || data_log_mode > 3 || broadcast_params != copy_of_broadcast_params)
		{
			memset(file_ending,0,sizeof(file_ending));
			memcpy(file_ending,z.time,6);
			strncat(file_ending,",",1);
			strncat(file_ending,"CORRUPTED\n",10);
			res = f_write(&file,file_ending,17,&bw);

			if(gps_log_flag)
			{
				memset(file_ending,0,sizeof(file_ending));
				strncat(file_ending,"START DATE: ",12);
				strncat(file_ending,Start_Date,6);
				strncat(file_ending,"\n",1);
				res = f_write(&file, file_ending, 19, &bw);

				memset(file_ending,0,sizeof(file_ending));
				strncat(file_ending,"END DATE: ",10);
				strncat(file_ending,z.date,6);
				strncat(file_ending,"\n",1);
				res = f_write(&file, file_ending, 17, &bw);
			}
			memset(version_log,0,sizeof(version_log));
			sprintf(version_log, "Payload:%02d,%02d\nCMD:%02d,%02d\nECU:%02d,%02d\nSTM:%02d\nDriver:01\n",pay_ver, pay_ver, cmd_ver, cmd_ver, ecu_ver, ecu_ver, stm_ver);
			res = f_write(&file, version_log, 51, &bw);
			memset(file_ending,0,sizeof(file_ending));
			sprintf(file_ending, "DBC:%02d,", dbc_ver);
			strncat(file_ending,dbc_ver,11);
			strncat(file_ending,",\n",2);
			res = f_write(&file, file_ending,20, &bw);

			memset(version_log,0,sizeof(version_log));
			sprintf(version_log, "DLM:%02d,NCI:%02d,RCI:%08x,RsC:%08x,RF:%02d,CS:%02d,PGN:%02d,F1:%02d,NBP:%02d,NRP:%02d,RPM:%02d,PGN:%08x,PGN:%08x,rb1:%04d,rb2:%04d,fs:%09d",
					data_log_mode, num_can_ids, request_can_id[0], response_can_id[0], response_fifo, can_speed, num_pgns, fifo1_pgn, broadcast_params, req_params, rpm_mode, pgn[0].pgn_id, pgn[1].pgn_id,rb1_size,rb2_size,file_size);
			res = f_write(&file, version_log, 145, &bw);

			l = 0;
			do
			{
				res = f_close(&file);
				l++;
			}while(res != FR_OK && l < 3);

			SCB->AIRCR=0x05fa0004;
		}
//		if(heartbeat > 18000)
//		{
//			memset(file_ending,0,sizeof(file_ending));
//			memcpy(file_ending,z.time,6);
//			strncat(file_ending,",",1);
//			strncat(file_ending,"SYS RESET\n",10);
//			res = f_write(&file,file_ending,17,&bw);
//
//			for(int s = 0; s < wh_counter; s++)
//			{
//				memset(file_ending,0,sizeof(file_ending));
//				memcpy(file_ending,wh[s].time,6);
//				strncat(file_ending,",",1);
//				sprintf(wh_temp,"%1x",wh[s].type);
//				strncat(file_ending,wh_temp,1);
//				strncat(file_ending,"\n",1);
//				for(int q=0;q<500;q++)
//				{}
//				res = f_write(&file, file_ending, 9, &bw);
//				for(int q=0;q<500;q++)
//				{}
//			}
//			if(gps_log_flag)
//			{
//				memset(file_ending,0,sizeof(file_ending));
//				strncat(file_ending,"START DATE: ",12);
//				strncat(file_ending,Start_Date,6);
//				strncat(file_ending,"\n",1);
//				res = f_write(&file, file_ending, 19, &bw);
//
//				memset(file_ending,0,sizeof(file_ending));
//				strncat(file_ending,"END DATE: ",10);
//				strncat(file_ending,z.date,6);
//				strncat(file_ending,"\n",1);
//				res = f_write(&file, file_ending, 17, &bw);
//			}
//			memset(version_log,0,sizeof(version_log));
//			sprintf(version_log, "Payload:%02d,%02d\nCMD:%02d,%02d\nECU:%02d,%02d\nSTM:%02d\nDriver:01\n",pay_ver, pay_ver, cmd_ver, cmd_ver, ecu_ver, ecu_ver, stm_ver);
//			res = f_write(&file, version_log, 51, &bw);
//			memset(file_ending,0,sizeof(file_ending));
//			sprintf(file_ending, "DBC:%02d,", dbc_ver);
//			strncat(file_ending,dbc_ver,11);
//			strncat(file_ending,",\n",2);
//			res = f_write(&file, file_ending,20, &bw);
//
//			memset(version_log,0,sizeof(version_log));
//			sprintf(version_log, "DLM:%02d,NCI:%02d,RCI:%08x,RsC:%08x,RF:%02d,CS:%02d,PGN:%02d,F1:%02d,NBP:%02d,NRP:%02d,RPM:%02d,PGN:%08x,PGN:%08x,rb1:%04d,rb2:%04d,fs:%09d",
//					data_log_mode, num_can_ids, request_can_id[0], response_can_id[0], response_fifo, can_speed, num_pgns, fifo1_pgn, broadcast_params, req_params, rpm_mode, pgn[0].pgn_id, pgn[1].pgn_id,rb1_size,rb2_size,file_size);
//			res = f_write(&file, version_log, 145, &bw);
//
//			l = 0;
//			do
//			{
//				res = f_close(&file);
//				l++;
//			}while(res != FR_OK && l < 3);
//
//			SCB->AIRCR=0x05fa0004;
//		}

		if(IGN_ON == 1 && ENG_RUNNING == 0 && Update_payload == 1)
		{
			if(GPIO_Read_Bit(GPIOB, GPIO_PIN_12) == 0)
			{
				while(1)
				{
					if(GPIO_Read_Bit(GPIOB, GPIO_PIN_12) == 1)
						break;
				}
				NVIC_DisableIRQ(USART3_IRQn);
				NVIC_DisableIRQ(TIM7_IRQn);
				NVIC_DisableIRQ(USART6_IRQn);
				NVIC_DisableIRQ(USART2_IRQn);
				NVIC_DisableIRQ(TIM6_DAC_IRQn);
				NVIC_DisableIRQ(CAN1_RX1_IRQn);
				NVIC_DisableIRQ(CAN1_RX0_IRQn);

				res = f_open(&boot_config, "config.csv", FA_WRITE|FA_READ);
				q = 0;
				while(q < 2)
				{
					memset(config_buffer,0,sizeof(config_buffer));
					f_gets(config_buffer,20,&boot_config);
					q++;
				}
				memset(config_buffer,0,sizeof(config_buffer));
				strncpy(config_buffer, "1\nECU.bin", 9);

				res = f_write(&boot_config, config_buffer,9, &bw);
				f_close(&boot_config);
				//reset
				SCB->AIRCR=0x05fa0004;
			}
		}
		if(IGN_ON && ENG_RUNNING && (rb1_size > 1900 || rb2_size > 1900))
			write_csv();
		//Reset IWDG	
		if(IWDG_SET)
			TM_WATCHDOG_Reset();	
		//Check if file size is greater than 1MB

		if(file_size>0xF4240)
		{		
			write_csv();
			memset(file_ending,0,sizeof(file_ending));
			memcpy(file_ending,z.time,6);
			strncat(file_ending,",",1);
			strncat(file_ending,"FILE OVERFLOW\n",14);
			res = f_write(&file,file_ending,21,&bw);

			if(gps_log_flag)
			{
				memset(file_ending,0,sizeof(file_ending));
				strncat(file_ending,"START DATE: ",12);
				strncat(file_ending,Start_Date,6);
				strncat(file_ending,"\n",1);
				res = f_write(&file, file_ending, 19, &bw);

				memset(file_ending,0,sizeof(file_ending));
				strncat(file_ending,"END DATE: ",10);
				strncat(file_ending,z.date,6);
				strncat(file_ending,"\n",1);
				res = f_write(&file, file_ending, 17, &bw);
			}

			memset(version_log,0,sizeof(version_log));
			sprintf(version_log, "Payload:%02d,%02d\nCMD:%02d,%02d\nECU:%02d,%02d\nSTM:%02d\nDriver:01\n",pay_ver, pay_ver, cmd_ver, cmd_ver, ecu_ver, ecu_ver, stm_ver);
			res = f_write(&file, version_log, 51, &bw);
			memset(file_ending,0,sizeof(file_ending));
			sprintf(file_ending, "DBC:%02d,", dbc_ver);
			strncat(file_ending,dbc_ver,11);
			strncat(file_ending,",\n",2);
			res = f_write(&file, file_ending,20, &bw);

			memset(version_log,0,sizeof(version_log));
			sprintf(version_log, "DLM:%02d,NCI:%02d,RCI:%08x,RsC:%08x,RF:%02d,CS:%02d,PGN:%02d,F1:%02d,NBP:%02d,NRP:%02d,RPM:%02d,PGN:%08x,PGN:%08x,rb1:%04d,rb2:%04d,fs:%09d",
					data_log_mode, num_can_ids, request_can_id[0], response_can_id[0], response_fifo, can_speed, num_pgns, fifo1_pgn, broadcast_params, req_params, rpm_mode, pgn[0].pgn_id, pgn[1].pgn_id,rb1_size,rb2_size,file_size);
			res = f_write(&file, version_log, 145, &bw);

			l = 0;
			do
			{
				res = f_close(&file);
				l++;
			}while(res != FR_OK && l < 3);
			
			if(file_timer < 1800)
				SCB->AIRCR=0x05fa0004;

			filename_flag = 0;
			get_date = 0;
			new_file();		
		}
		if (( RTC_Wake == 2 ) && ( esp_counter > 12))
		{

			esp_counter=0;
			esp_counter_active=0;
			RTC_Fail++;

			if(RTC_Fail < 3)
			{
				GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 0);
				for(int i=0; i<1000; i++);
				GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 1);
			}


		}
		if(esp_counter > ESP_COUNTER_TIMEOUT)
		{
			if(update_checked == 0)
			{
				esp_counter=0;
				esp_counter_active=0;
				wifi_fail++;

				if(wifi_fail < 3)
				{
					GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 0);
					for(int i=0; i<1000; i++);
					GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 1);
				}
				else
				{
					error = 5;
					update_checked = 3;
				}
			}
			else if(esp_filename_wait)
			{
				IFT_fail++;
				esp_counter=0;
				esp_counter_active=0;

				GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 0);
				delay(1);
				GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 1);

			}
			//try to send rtc heartbeat for 2 times this happens if esp doesn't connect to wifi

			else
			{
				wifi_fail++;
				upload_active = 0;
				upload_complete = 0;
				esp_counter_active = 0;
				esp_counter=0;

				memcpy(wh[wh_counter].time,z.time,6);
				wh[wh_counter].type = 5;
				wh_counter++;
				if(wifi_fail <= 3)
				{
					GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 0);
					for(int i=0; i<1000; i++);
					GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 1);
				}
			}
		}

		if(upload_timer > UPLOAD_TIMER_TIMEOUT)
		{
			wifi_fail++;
			upload_active = 0;
			upload_complete = 0;
			esp_counter_active = 0;
			esp_counter=0;
			upload_timer = 0;
			upload_timer_active = 0;

			memcpy(wh[wh_counter].time,z.time,6);
			wh[wh_counter].type = 5;
			wh_counter++;
			if(wifi_fail <= 3)
			{
				GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 0);
				delay(1);
				GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 1);
			}


		}

		// Change IGN ON Flag after 20 secs of DAQ Inactivity 
		if(IGN_ON)
		{
			switch(data_log_mode)
			{
				case 0:
				{
					if(can_inactive>20)
						{
							IGN_ON=0;
							ENG_RUNNING=0;
							write_csv();
							memset(temp_buf,0,sizeof(temp_buf));
							memcpy(temp_buf,z.time,6);
							strncat(temp_buf,",",1);
							strncat(temp_buf,"CAN INACTIVE\n",13);

							res=f_write(&file,temp_buf,20, &bw);
							f_sync(&file);
						}
					break;
				}
				case 1:
				{
					if(can_inactive>20)
					{
						IGN_ON=0;
						ENG_RUNNING=0;
						write_csv();
						memset(temp_buf,0,sizeof(temp_buf));
						memcpy(temp_buf,z.time,6);
						strncat(temp_buf,",",1);
						strncat(temp_buf,"CAN INACTIVE\n",13);

						res=f_write(&file,temp_buf,20, &bw);
						f_sync(&file);
					}

					break;					
				}
				case 2:
				{
					if(can_inactive>20 && kline_inactive>20)
					{
						IGN_ON=0;
						ENG_RUNNING=0;
						write_csv();
						memset(temp_buf,0,sizeof(temp_buf));
						memcpy(temp_buf,z.time,6);
						strncat(temp_buf,",",1);
						strncat(temp_buf,"C K INACTIVE\n",13);

						res=f_write(&file,temp_buf,20, &bw);
						f_sync(&file);
					}

					break;
				}
				case 3:
				{
					if(can_inactive>20 && kline_inactive>20)
					{
						IGN_ON=0;
						ENG_RUNNING=0;
						write_csv();
						memset(temp_buf,0,sizeof(temp_buf));
						memcpy(temp_buf,z.time,6);
						strncat(temp_buf,",",1);
						strncat(temp_buf,"C K INACTIVE\n",13);

						res=f_write(&file,temp_buf,20, &bw);
						f_sync(&file);
					}

					break;
				}
			}
			if ((upload_complete == 1 || wifi_fail > 3) && update_checked > 1)
			{
				wifi_power_off();
			}
		}

		if(stop_logging > 30)
		{
			ENG_RUNNING = 0;
			stop_logging=0;
			write_csv();
			memset(file_ending,0,sizeof(file_ending));
			memcpy(file_ending,z.time,6);
			strncat(file_ending,",",1);
			strncat(file_ending,"ENG STOPPED\n",12);

			res=f_write(&file,file_ending,19, &bw);
			f_sync(&file);
		}
	
		if(IGN_ON == 0)
		{
			wifi_power_on();

			if(write_once == 0)
			{
				write_once = 1;
				for(int s = 0; s < wh_counter; s++)
				{
					memset(file_ending,0,sizeof(file_ending));
					memcpy(file_ending,wh[s].time,6);
					strncat(file_ending,",",1);
					sprintf(wh_temp,"%1x",wh[s].type);
					strncat(file_ending,wh_temp,1);
					strncat(file_ending,"\n",1);
					for(int q=0;q<500;q++)
					{}
					res = f_write(&file, file_ending, 9, &bw);
					for(int q=0;q<500;q++)
					{}
				}
				if(gps_log_flag)
				{
					memset(file_ending,0,sizeof(file_ending));
					strncat(file_ending,"START DATE: ",12);
					strncat(file_ending,Start_Date,6);
					strncat(file_ending,"\n",1);
					res = f_write(&file, file_ending, 19, &bw);

					memset(file_ending,0,sizeof(file_ending));
					strncat(file_ending,"END DATE: ",10);
					strncat(file_ending,z.date,6);
					strncat(file_ending,"\n",1);
					res = f_write(&file, file_ending, 17, &bw);
				}
				memset(version_log,0,sizeof(version_log));
				sprintf(version_log, "Payload:%02d,%02d\nCMD:%02d,%02d\nECU:%02d,%02d\nSTM:%02d\nDriver:01\n",pay_ver, pay_ver, cmd_ver, cmd_ver, ecu_ver, ecu_ver, stm_ver);
				res = f_write(&file, version_log, 51, &bw);
				memset(file_ending,0,sizeof(file_ending));
				sprintf(file_ending, "DBC:%02d,", dbc_ver);
				strncat(file_ending,dbc_ver,11);
				strncat(file_ending,",\n",2);
				res = f_write(&file, file_ending,20, &bw);

				memset(version_log,0,sizeof(version_log));
				sprintf(version_log, "DLM:%02d,NCI:%02d,RCI:%08x,RsC:%08x,RF:%02d,CS:%02d,PGN:%02d,F1:%02d,NBP:%02d,NRP:%02d,RPM:%02d,PGN:%08x,PGN:%08x,rb1:%04d,rb2:%04d,fs:%09d",
						data_log_mode, num_can_ids, request_can_id[0], response_can_id[0], response_fifo, can_speed, num_pgns, fifo1_pgn, broadcast_params, req_params, rpm_mode, pgn[0].pgn_id, pgn[1].pgn_id,rb1_size,rb2_size,file_size);
				res = f_write(&file, version_log, 145, &bw);

				l = 0;
				do
				{
					res = f_close(&file);
					l++;
				}while(res != FR_OK && l < 3);
				get_date = 0;
				filename_flag = 0;

			}

			//Ensure this is done only once
			if(!do_once)
			{
				do_once = 1;
				its_done = 0;
				error = 0;
				wifi_fail = 0;
				update_checked = 0;
				play_once = 0;
				GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 0);
				for(int i=0; i<1000; i++);
				GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 1);
			}

			if(upload_active != 1 && (upload_complete == 1 || wifi_fail >= 3) && !(IFT_complete == 1 || IFT_fail>=3) && (its_done == 1 || error >= 3))
			{
				memset(wh,0,100*sizeof(wifi_history));
				wh_counter = 0;

				file_size=0;
				filename_flag = 0;

				// Start Server upload
				upload_complete = 0;
				wifi_fail = 0;
				upload_active = 0;
				update_checked = 3;
				GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 0);
				delay(1);
				GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 1);

				//15 sec delay
				delay_timer = 0;
				delay_timer_active = 1;
				while(delay_timer < 15);
				delay_timer = 0;
				delay_timer_active = 0;

				while(/*upload_active == 1 && */upload_complete != 1 && wifi_fail < 3)
				{
					if(IWDG_SET)
						TM_WATCHDOG_Reset();
					if(esp_counter > ESP_COUNTER_TIMEOUT)
					{

						wifi_fail++;
						upload_active = 0;
						upload_complete = 0;
						esp_counter_active = 0;
						esp_counter=0;

						memcpy(wh[wh_counter].time,z.time,6);
						wh[wh_counter].type = 5;
						wh_counter++;
						if(wifi_fail <= 3)
						{
							GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 0);
							delay(1);
							GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 1);
						}
					}

					if(upload_timer > UPLOAD_TIMER_TIMEOUT)
					{
						wifi_fail++;
						upload_active = 0;
						upload_complete = 0;
						esp_counter_active = 0;
						esp_counter=0;
						upload_timer = 0;
						upload_timer_active = 0;

						memcpy(wh[wh_counter].time,z.time,6);
						wh[wh_counter].type = 5;
						wh_counter++;
						if(wifi_fail <= 3)
						{
							GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 0);
							delay(1);
							GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 1);
						}
					}
				}

				if(filename_flag==0)
				{
					if (IGN_ON==0)
					{
						IFT_fail=0;
						update_config_temp_counter();

						//10sec delay
						delay_timer = 0;
						delay_timer_active = 1;
						while(delay_timer < 10);
						delay_timer = 0;
						delay_timer_active = 0;

						ESPTRANSMITx();
						IFT_active = 0;
						NVIC_EnableIRQ(USART3_IRQn);
						NVIC_EnableIRQ(TIM7_IRQn);
						upload_complete = 0;
						update_checked = 3;
						wifi_fail = 0;
						delay(600);
						ESP_CMD_SEND(UPLDFXS);
					}
				}
			}
		}
		if(ps && gps_log_flag && (RTC_Wake == 0))
		{

			ps = 0;
			if(upload_active != 1 && (upload_complete == 1 || wifi_fail >= 3) && update_checked > 1 )
			{
				if(IGN_ON == 0 && (IFT_complete == 1 || IFT_fail>=3))
				{
					GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 0);
    				for(int i=0; i<1000; i++);
     				GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 1);
     				for(int i=0; i<10000; i++);
					RTC_Heartbeat(1);
				}

				else if(IGN_ON == 1)
				{
					GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 0);
					for(int i=0; i<1000; i++);
					GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 1);
					for(int i=0; i<10000; i++);
					RTC_Heartbeat(1);
				}

			}
		}
        if(RTC_Wake == 1)
        {
    		EXTI->PR |= EXTI_PR_PR22;
    		NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);

       	//IF ESP UPLOAD IS COMPLETED OR TRIED MORE THAN 3 TIMES AND IFT COMPLETED OR TRIED MORE THAN 3 TIMES AND UPDATE CHECK IS DONE
        	if(upload_active != 1 && (upload_complete == 1 || wifi_fail >= 3) && update_checked > 1 )
        	{
        		if(IGN_ON == 0 && (IFT_complete == 1 || IFT_fail>=3))
        		{
        			RTC_Wake = 2;
    				GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 0);
    				for(int i=0; i<1000; i++);
    				GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 1);


        		}
        		else if(IGN_ON == 1)
        		{
        			RTC_Wake = 2;

    				GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 0);
    				for(int i=0; i<1000; i++);
    				GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 1);


        		}



        	}


        }
		if(IGN_ON == 0 && upload_active != 1 && 
			(upload_complete == 1 || wifi_fail >= 3) && (IFT_complete == 1 || IFT_fail>=3) && RTC_Wake == 0	)
		{
			STM_SLEEP = 1;
			LOW_POWER_MODE();
		}


	}
}
