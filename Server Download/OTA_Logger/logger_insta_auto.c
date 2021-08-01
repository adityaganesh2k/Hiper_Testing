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

//Timeout Macros
#define ESP_COUNTER_TIMEOUT 40
#define UPLOAD_TIMER_TIMEOUT 1000

	//FATFS Variables
	FATFS fs;            	  	/* Filesystem object */
	FIL file, boot_config, file_down, IFT, filex;           			/* File object */
	FRESULT res;        			/* API result code */
	FILINFO fno;							/* File information */
	UINT bw, br;            	/* Bytes written */

	//GPS Variables
	GPS_Data z;
	// New file
	uint32_t temp_file_counter=0;
	uint8_t filename_flag = 0;
	char filename[15]="t_";
	char temp_buf[40];
	char temp_buffer[10];
	//Load DBC
	uint8_t data_log_mode, num_can_ids, can_speed;
	uint8_t req_params, ecu_params;
	uint32_t request_can_id[2], response_can_id[2];
	uint32_t heartbeat=0, file_timer=0;

	//CAN
	request_param arr_req_param[50];
	request_param arr_ecu_param[20];
	
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
	char log_ecu[256];

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
	char s_buf[256], h_buf[10];

	//Flags & Counters
	uint8_t IGN_ON=0;

	//ESP
	int esp_counter = 0, upload_timer = 0;
	uint8_t esp_counter_active = 0, upload_timer_active = 0, upload_active = 0, upload_complete = 0;
	uint8_t delay_timer = 0, delay_timer_active = 0;
	uint8_t wifi_fail = 0, esp_filename_start = 0, esp_filename_counter = 0;
	uint8_t esp_filename_wait = 0, esp_lastfilename[15];
	char* IFT_filename = "";
	_Bool IFT_complete = 0,IFT_active = 0, STM_SLEEP = 0, exception_01 = 0, write_once = 0;
	uint8_t	IFT_fail=0, logging = 1;
	char wh_temp[2];

	uint8_t file_transfer_ready = 0;

	char version_log[145];
	uint8_t file1, file2, file3, file4, file5;
	uint8_t ver_1, ver_2, ver_3, ver_4, ver_5;			// 1 - Payload version  2 - DBC
														// 3 - CMD  4 - ECU   5 - STM

	int stat1 = 0,stat2 = 0, stat3 = 0, stat4 = 0, stat5 = 0;
	uint8_t ecu_ver, pay_ver, cmd_ver;
	uint8_t message_start = 0, abort_file = 0, set_variable = 1, update_checked = 0, error = 0, do_once = 0, play_once = 0, Update_payload = 0;   // Use set variable before ymodem_receive
	uint8_t Data_IFT[PACKET_1K_SIZE + PACKET_OVERHEAD];
	uint16_t packet_size=0, count1=0, index_IFT=0;
	char orig_name[25] = {'\0'};
	char size_IFT[256] = {'\0'};
	uint32_t filesize_IFT = 0, filesize_check = 0;
	uint8_t a_bit, rename_flag = 0, IFT_Start = 0, download_fail = 0, down_status = 0, its_done = 0, down_done = 0;
	char Start_Date[6], dbc_ver[12];
	int get_date = 0, timer7_mode = 1, counter = 0, download_timeout = 60;

	uint8_t broadcast_done = 0, req_done = 0;
	uint32_t tracker_req = 0, tracker_req1 = 0;

	uint8_t CAN_Multiframe_Length;
	char CAN_Frame[256];
	uint8_t frame_index = 0;
	uint8_t cycle[17];					// Cycle counter for each frame number
	uint8_t ctrl=0;
	uint64_t set_frames_received;		// Each bit is set when a frame number has been recived 
	uint8_t can_process_flag=1;			// Ensure set to 1 before sending out a request
	uint8_t count;

	//Config File
	void config_file()
	{
//		unsigned int device_id_val;
		char config_buf[20];
		char s[5];
		uint8_t k,i;
		//Create a new Config File
		if (f_stat("config.csv", &fno)== FR_NO_FILE)
		{
		
			/* Create a file as new */
			res = f_open(&file, "config.csv", FA_CREATE_ALWAYS | FA_WRITE|FA_READ);
			memset(config_buf,0,20);
			
			/* Temp File # */
			sprintf(config_buf,"BLANK");
			strncat(config_buf,"\n",1);
			
			res=f_write(&file, config_buf, 6, &bw);
			memset(config_buf,0,20);	
			
			/* Temp File # */
			sprintf(config_buf,"000");
			strncat(config_buf,"\n",1);											
			
			res=f_write(&file, config_buf, 4, &bw);
			memset(config_buf,0,20);	
			
//			/* Write a HIPER UNIQUE ID */
//			device_id_val = get_device_id();
//			sprintf(config_buf, "%8X", device_id_val);
//			strncat(config_buf,"\n",1); 										//Pointer is in position 9
//
//			res=f_write(&file, config_buf, 9, &bw);
//			memset(config_buf,0,20);
//
//			/* VIN No. */
//			sprintf(config_buf,"9629799271xxxxxxx");
//			strncat(config_buf,"\n",1);											//Pointer is in position 38
//
//			res=f_write(&file, config_buf, 18, &bw);
//			memset(config_buf,0,20);
			
			/* Close the file */
			do
			{
				res = f_close(&file);
			}while(res!=FR_OK);
		}
		//Check for the temp_file_counter
		else
		{
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
			ver_1=atoi(s);

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
			ver_2=atoi(s);

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
			ver_3=atoi(s);

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
			ver_4=atoi(s);


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
			ver_5=atoi(s);

			do
			{
				res = f_close(&file);
			}while(res!=FR_OK);
		}
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
							sscanf(s,"%x",&p);
							arr_req_param[j].service=p;
							memset(s,0,sizeof(s));
							break;															
						}
						case 4:
						{
							s[n]='\0';
							sscanf(s,"%x",&p);
							arr_req_param[j].PID=p;
							memset(s,0,sizeof(s));
							break;				
						}
						case 5:
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

			memset(s,0,4);
			memset(temp_buff,0,sizeof(temp_buff));
			k=1;
			i=0;
				
		//Read the # of ECU messages
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
			ecu_params=(uint8_t)atoi(s);
			memset(s,0,4);
			memset(temp_buff,0,sizeof(temp_buff));
			k=1;
			i=0;
			j=0;

		//Loading req params	
		while(j<ecu_params) 
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
							arr_ecu_param[j].can_id_no = p;
							memset(s,0,sizeof(s));
							break;			
						}

						case 1:
						{
							s[n]='\0';																	// Add into daq 
							sscanf(s,"%d",&p);
							arr_ecu_param[j].can_extn_param=p;
							memset(s,0,sizeof(s));
							break;			
						}

						case 2:
						{
							s[n]='\0';
							sscanf(s,"%d",&p);
							arr_ecu_param[j].length=p;
							memset(s,0,sizeof(s));
							break;			
						}
						case 3:
						{
							s[n]='\0';
							sscanf(s,"%x",&p);
							arr_ecu_param[j].service=p;
							memset(s,0,sizeof(s));
							break;															
						}
						case 4:
						{
							s[n]='\0';
							sscanf(s,"%x",&p);
							arr_ecu_param[j].PID=p;
							memset(s,0,sizeof(s));
							break;				
						}
						case 5:
						{
							if(arr_ecu_param[j].can_extn_param)
							{
								s[n]='\0';
								sscanf(s,"%x",&p);
								arr_ecu_param[j].PID1=p;								// add into daq
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

/*
		CAN Filter Init 
	*/
	void can_filt_init()
	{
		CAN_FilterInit_SlaveBankStart(1+num_can_ids);
		int i,j;
 		
		switch(data_log_mode)
		{
			case 0:						//Pure CAN 2.0
			{			
				CAN_FilterInit_Req(0);
				CAN_FilterInit_FilterMode(0,CAN_FILTERMODE_IDMASK );
				CAN_FilterInit_FilterScale(0,CAN_FILTERSCALE_32BIT );					
				CAN_Filter_IDMaskModify(0,1,((response_can_id[0]<<3)|0X04));
				CAN_Filter_IDMaskModify(0,2,0);                             			//mask for PGN
				CAN_FilterInit_FilterAssign(0,1);
				CAN_FilterActivate(0);

			//Assign filter for responses for requested params	
				j=0;
				i=1;
				while(j<num_can_ids)
				{
					CAN_FilterInit_Req(i);
					CAN_FilterInit_FilterMode(i,CAN_FILTERMODE_IDMASK );
					CAN_FilterInit_FilterScale(i,CAN_FILTERSCALE_32BIT );					
					CAN_Filter_IDMaskModify(i,1,((response_can_id[j]<<3)|0X04)); 
					CAN_Filter_IDMaskModify(i,2,((0xFFFF00<<3)));                             			//mask for PGN
					CAN_FilterInit_FilterAssign(i,0);
					CAN_FilterActivate(i);
					j++;
					i++;
				}

				break;
			}
			case 1:					//Pure CAN 1.0
			{
				CAN_FilterInit_Req(0);
				CAN_FilterInit_FilterMode(0,CAN_FILTERMODE_IDMASK );
				CAN_FilterInit_FilterScale(0,CAN_FILTERSCALE_32BIT );					
				CAN_Filter_IDMaskModify(0,1,(response_can_id[0]<<21));
				CAN_Filter_IDMaskModify(0,2,0);                             			//mask for PGN
				CAN_FilterInit_FilterAssign(0,1);
				CAN_FilterActivate(0);

				j=0;
				i=1;

				while(j<num_can_ids)
				{
					CAN_FilterInit_Req(i);
					CAN_FilterInit_FilterMode(i,CAN_FILTERMODE_IDMASK );
					CAN_FilterInit_FilterScale(i,CAN_FILTERSCALE_32BIT );					
					CAN_Filter_IDMaskModify(i,1,(response_can_id[0]<<21));
					CAN_Filter_IDMaskModify(i,2,0xFFE00000);                             			//mask for PGN
					CAN_FilterInit_FilterAssign(i,0);
					CAN_FilterActivate(i);
					j++;
					i++;
				}

				break;
			}
			case 2:					//CAN 2.0 & Kline
			{
				CAN_FilterInit_Req(0);
				CAN_FilterInit_FilterMode(0,CAN_FILTERMODE_IDMASK );
				CAN_FilterInit_FilterScale(0,CAN_FILTERSCALE_32BIT );					
				CAN_Filter_IDMaskModify(0,1,((response_can_id[0]<<3)|0X04));
				CAN_Filter_IDMaskModify(0,2,0);                             			//mask for PGN
				CAN_FilterInit_FilterAssign(0,1);
				CAN_FilterActivate(0);
				break;
			}
			case 3:					//CAN 1.0 & Kline
			{			
				CAN_FilterInit_Req(0);
				CAN_FilterInit_FilterMode(0,CAN_FILTERMODE_IDMASK );
				CAN_FilterInit_FilterScale(0,CAN_FILTERSCALE_32BIT );					
				CAN_Filter_IDMaskModify(0,1,(response_can_id[0]<<21));
				CAN_Filter_IDMaskModify(0,2,0);                             			//mask for PGN
				CAN_FilterInit_FilterAssign(0,1);
				CAN_FilterActivate(0);	
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
		
		TIM6_param.Period = 49;
		TIM6_param.Prescaler = 47999;
		TIM6_Base_Init(&TIM6_param);
		
		NVIC_SetPriority(TIM6_DAC_IRQn,2);
		NVIC_ClearPendingIRQ(TIM6_DAC_IRQn); 
		NVIC_EnableIRQ(TIM6_DAC_IRQn);
	}

	void TIM6_Init_can_daq()
	{
		RCC_TIM6_CLOCK_ENABLE_CLK();
		
		TIM6_param.Period = 49;
		TIM6_param.Prescaler = 47999;
		TIM6_Base_Init(&TIM6_param);
		
		NVIC_SetPriority(TIM6_DAC_IRQn,2);
		NVIC_ClearPendingIRQ(TIM6_DAC_IRQn); 
		NVIC_EnableIRQ(TIM6_DAC_IRQn);
	}
	void TIM6_Init_ecu_daq()
	{
		RCC_TIM6_CLOCK_ENABLE_CLK();

		TIM6_param.Period = 99;
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
					//Init DAQ Timer
						TIM6_Init_can_daq();
						NVIC_DisableIRQ(TIM6_DAC_IRQn);
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
					//Init DAQ Timer
						TIM6_Init_can_daq();
						NVIC_DisableIRQ(TIM6_DAC_IRQn);
					//Timer Mode
						timer_mode=1;

					break;
				}
				case 2:
				{
					//K Line Init
						KLINE_INIT_SEQ();
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
			NVIC_DisableIRQ(CAN1_RX1_IRQn);
		
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
		NVIC_EnableIRQ(CAN1_RX1_IRQn);

	}
	/*
		K-Line Init sequence will be implemented by this function 
	*/
	void KLINE_REINIT()
	{
		NVIC_DisableIRQ(USART6_IRQn);
		NVIC_DisableIRQ(TIM6_DAC_IRQn);
		NVIC_DisableIRQ(TIM7_IRQn);
		NVIC_DisableIRQ(CAN1_RX1_IRQn);
		NVIC_DisableIRQ(CAN1_RX0_IRQn);
		NVIC_DisableIRQ(USART3_IRQn);
		KLINE_INIT_SEQ();
		TIM6_Init_kline_daq();
		NVIC_EnableIRQ(USART6_IRQn);
		NVIC_EnableIRQ(TIM6_DAC_IRQn);
		NVIC_EnableIRQ(TIM7_IRQn);
		NVIC_EnableIRQ(CAN1_RX1_IRQn);
		NVIC_EnableIRQ(CAN1_RX0_IRQn);
		NVIC_EnableIRQ(USART3_IRQn);
	}

	void update_rb() //only request parameters are logged, in rb1
	{
		char temp_buf[20];
		char buf[2];
		int j=0;
		
		for (int k=0;k<req_params;k++)
		{
			if(arr_req_param[k].length > 12)
			{
				for(j=0; (j<arr_req_param[k].length) && (j<=3); j++)
				{
					sprintf(temp_buf,"%02x",Data[k][j]);
					strncat(rb1,temp_buf,2);
					rb1_size=rb1_size+2;
				}
			}
			else
			{
				for(j=0; (j<arr_req_param[k].length) && (j<=1); j++)
				{
					sprintf(temp_buf,"%02x",Data[k][j]);
					strncat(rb1,temp_buf,2);
					rb1_size=rb1_size+2;
				}
			}

			memset(temp_buf,0,sizeof(temp_buf));
			strncpy(temp_buf, ",", 1);
			strncat(rb1,temp_buf,1);
			rb1_size=rb1_size+1;
			memset(temp_buf,0,sizeof(temp_buf));

		}
		memset(temp_buf,0,sizeof(temp_buf));
		if(req_params > 32)
		{
			sprintf(temp_buf,"%08x",tracker_req1);
			strncat(rb1,temp_buf,8);	
			rb1_size+=8;
			tracker_req1 = 0;
		}
		sprintf(temp_buf,"%08x",tracker_req);
		strncat(rb1,temp_buf,8);
		strncat(rb1,",",1);
		rb1_size=rb1_size+9;
		strncat(rb1,"\r\n",2);
		rb1_size=rb1_size+2;
		tracker_req = 0;
		tracker_req1 = 0;
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

				if(data_log_mode == 0 || data_log_mode == 1)
				{
					last_req++;

					if(req_done == 2)
					{
						if(last_req >= ecu_params)
						{
							last_req=0;
							req_done=3;
							NVIC_DisableIRQ(TIM6_DAC_IRQn);
							break;
						}
						can_process_flag = 1;
						buff_length = 0;
						if(arr_ecu_param[last_req].can_extn_param)
							tx_can.Data[0]=0x03;
						else
							tx_can.Data[0]=0x02;

						tx_can.Data[1]=arr_ecu_param[last_req].service;
						tx_can.Data[2]=arr_ecu_param[last_req].PID;

						if(arr_ecu_param[last_req].can_extn_param)
							tx_can.Data[3]=arr_ecu_param[last_req].PID1;

						tx_can.ExtId = request_can_id[arr_ecu_param[last_req].can_id_no];
		
						if(CAN_Which_MailboxIsEmpty(CAN1) != 0x0F)
							CAN_Transmit(CAN1,&tx_can,CAN_Which_MailboxIsEmpty(CAN1));

						break;

					}

					if(last_req >= req_params)
					{
						last_req=0;
						req_done=1;
						NVIC_DisableIRQ(TIM6_DAC_IRQn);
						break;
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

					if(CAN_Which_MailboxIsEmpty(CAN1) != 0x0F)
						CAN_Transmit(CAN1,&tx_can,CAN_Which_MailboxIsEmpty(CAN1));

				}

				if(data_log_mode == 2 || data_log_mode == 3)
				{
					if (kline_init == 4)
					{	
						last_req++;
					
						if(req_done == 2)
						{
							if(last_req >= ecu_params)
							{
								last_req=0;
								req_done=3;
								NVIC_DisableIRQ(TIM6_DAC_IRQn);
								break;
							}

							if (kline_init == 4)
							{
							 	kline_buf[0]=0x82;
								kline_buf[1]=0x10;
								kline_buf[2]=0xF1;
								kline_buf[3]=arr_ecu_param[last_req].service;
								kline_buf[4]=arr_ecu_param[last_req].PID;	
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
							}
							break;
						}

						if(last_req >= req_params)
						{
							last_req=0;
							req_done=1;
							NVIC_DisableIRQ(TIM6_DAC_IRQn);
							break;
						}

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
					if(logging == 1)
						switch(heartbeat)
						{
							case 10:
								ESP_CMD_SEND(PRESS_CLUTCH);
								break;
							case 20:
								ESP_CMD_SEND(RELEASE_CLUTCH);
								break;
							case 30:
								ESP_CMD_SEND(PRESS_CLUTCH);
								break;
							case 40:
								ESP_CMD_SEND(RELEASE_CLUTCH);
								break;
							case 50:
								ESP_CMD_SEND(PRESS_BRAKE);
								break;
							case 60:
								ESP_CMD_SEND(RELEASE_BRAKE);
								break;
							case 70:
								ESP_CMD_SEND(PRESS_BRAKE);
								break;
							case 80:
								ESP_CMD_SEND(RELEASE_BRAKE);
								break;
							case 90:
								ESP_CMD_SEND(FIRST_GEAR);
								break;
							case 100:
								ESP_CMD_SEND(NEUTRAL);
								break;
							case 110:
								ESP_CMD_SEND(ENG_ON);
								break;
							case 120:
								ESP_CMD_SEND(PRESS_THROTTLE);
								break;
							case 130:
								ESP_CMD_SEND(RELEASE_THROTTLE);
								break;
							case 140:
								ESP_CMD_SEND(ENG_OFF);
								break;
							default:
								break;
						}
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

					if(heartbeat > 150 && !broadcast_done)
					{

						// Collected BRD Params, move on to REQ
						broadcast_done = 1;
						heartbeat = 0;
						CAN_FilterDeActivate(0);
						NVIC_DisableIRQ(CAN1_RX1_IRQn);
						write_csv();
						rb_flag = 1;

						//Msg that brd is done; Req going to start
						strncat(rb1, "BRD DONE; REQ STARTING\n",23);
						rb1_size=rb1_size+23;
						ESP_CMD_SEND(BRD_DONE);

						char temp_buf[10];

						for (int k=0;k<req_params;k++)
						{
							memset(temp_buf,0,sizeof(temp_buf));
							sprintf(temp_buf,"%02x",arr_req_param[k].PID);

							if(arr_req_param[k].can_extn_param)
							{
								strncat(rb1,temp_buf,2);
								rb1_size+=2;
								memset(temp_buf,0,sizeof(temp_buf));
								sprintf(temp_buf,"%02x",arr_req_param[k].PID1);
							}

							strncat(temp_buf,",",1);
							strncat(rb1,temp_buf,3);
							rb1_size=rb1_size+3;
						}
						strncat(rb1,"\r\n",2);
						rb1_size=rb1_size+2;


						switch(data_log_mode)
						{
							case 0:
							case 1:
									last_req = 0;
									if(arr_req_param[last_req].can_extn_param)
											tx_can.Data[0]=0x03;
									else
										tx_can.Data[0]=0x02;

									tx_can.Data[1]=arr_req_param[last_req].service;
									tx_can.Data[2]=arr_req_param[last_req].PID;

									if(arr_req_param[last_req].can_extn_param)
										tx_can.Data[3]=arr_req_param[last_req].PID1;

									tx_can.ExtId = request_can_id[arr_req_param[last_req].can_id_no];
			
									if(CAN_Which_MailboxIsEmpty(CAN1) != 0x0F)
										CAN_Transmit(CAN1,&tx_can,CAN_Which_MailboxIsEmpty(CAN1));

									break;
							case 2:
							case 3:
									if (kline_init == 4)
									{
										last_req = 0;

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
									}
									break;
						}
						TIM6_Init_can_daq();
						TIM6_Base_Start_IT();
					}

					switch(data_log_mode)
					{
						case 0:
						
						case 1:
								can_inactive++;
								break;
						
						case 2:
						
						case 3:
						{
							can_inactive++;
							kline_inactive++;

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
							break;
						}
						default:
					    	break;
					}
					break;
			}
		
		/* TIM Update event */
		if((TIM7->SR & (TIMER_UPDATE_FLAG)) == TIMER_UPDATE_FLAG)
		{
			TIM7->SR = ~(TIMER_INTERRUPT);  
		}
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

			file1 = (stat1 != ver_1)?1:0;
			file2 = (stat2 != ver_2)?1:0;
			file3 = (stat3 != ver_3)?1:0;
			file4 = (stat4 != ver_4)?1:0;
			file5 = (stat5 != ver_5)?1:0;

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
						if(upload_active == 0 && upload_complete==0)
						{
							ESP_CMD_SEND(UPLDFXS);
						}
						else if (upload_active ==1 && upload_complete==0) // Initial
						{
							wifi_fail++;
							upload_active = 0;
							upload_complete = 0;
							esp_counter_active = 0;

							GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 0);
							for(int i = 0; i<1000; i++);
							GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 1);
						}
					}
					break;
				}
				case '%':   //No file to upload
				{
					upload_complete = 1;
					break;
				}
				case '$':
				{
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
								    count1 = 133;

								    break;

						  case STX:
								    packet_size = PACKET_1K_SIZE;
								    message_start = 1;
								    Data_IFT[0] = STX;
								    count1 = 1029;

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

						if(count1 == index_IFT)
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

		while (file_transfer_ready==0 && esp_counter< ESP_COUNTER_TIMEOUT)
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

			}while(exception_01==0 && IGN_ON==0 && IFT_fail<3);

			if(exception_01 == 1)
				IFT_complete = 1;
		}
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
				{
					kline_init = 4;
					NVIC_DisableIRQ(TIM6_DAC_IRQn);

				}
				else
				{
					kline_init = 0; break;
				}
			}
			if (kline_init != 4)
				KLINE_REINIT();

			kline_inactive=0;
			kline_len=0;
		}

		if(kline_process_flag == 1 && kline_init == 4)
		{
			kline_process_flag = 0;
			i=last_req;

			if(req_done == 2)
			{
				for(int ti = 0; ti < buff_length; ti++)
				{
					sprintf(h_buf, "%02x", Kline_RxBuff[ti]);
					strncat(s_buf,h_buf,2);
				}
				res = f_write(&file, s_buf, (buff_length*2), &bw);
				res = f_write(&file, ",", 1, &bw);
				memset(s_buf, 0, sizeof(s_buf));

				tracker_req |= (1<<last_req);
			}
			else
			{
				for (int k=0;k<buff_length;k++)
				{
					sprintf(h_buf,"%02x",Kline_RxBuff[k]);
					strncat(rb1,h_buf,2);
					rb1_size=rb1_size+2;
				}
				memset(h_buf,0,sizeof(h_buf));
				strncat(rb1,",",1);
				rb1_size=rb1_size+1;
				
				if(last_req > 31)
					tracker_req1 |= (1<<(last_req-32));
				else
					tracker_req |= (1<<last_req);
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
		uint8_t i,j,k,l,ti;
		char temp_buf[20], test_buf[256];

		can_inactive=0;

		if(req_done == 2)
		{
			/* Log Multiframe messages */		
			if(rx_can.Data[0] == 0x10 || ((rx_can.Data[0] & 0xF0) == 0x20))
			{
				if(rx_can.Data[0] == 0x10)
				{
					CAN_Multiframe_Length = rx_can.Data[1];						// Total length of Data bytes in all the frames
					buff_length = rx_can.Data[1];

					memset(CAN_Frame,0,sizeof(CAN_Frame));		

					for(int i=2; i<8; i++)
						CAN_Frame[i-2] = rx_can.Data[i];							// Log first frame message separately
					
					CAN_Multiframe_Length -= 6;

					tx_can.Data[0]=0x30;												// Acknowledge for multiframe message 
					for(int i=1; i<8; i++)
						tx_can.Data[i]=0x00;							
					
					while(CAN_Which_MailboxIsEmpty(CAN1)==0x0F)
					{}
					CAN_Transmit(CAN1,&tx_can,CAN_Which_MailboxIsEmpty(CAN1));
		
					for(int i=0; i<17; i++)											// Reset cycle count when the next multiframe message is started 
						cycle[i] = 0;	

					set_frames_received = 0;
					can_process_flag = 0;
				}
				else
				{
					ctrl = rx_can.Data[0];
					ctrl -= 0x20;

					if(ctrl == 0x0)				// Frame Number 0 is assumed as 17 for ease of computation
						ctrl = 0x10;

					switch(cycle[ctrl])						// Update cycle counter value of a frame after 5 frames are received
					{
						case 0:
								if(ctrl >= 6)
									cycle[ctrl-5] = 1;
								break;

						case 1:
								if(ctrl >= 6)
									cycle[ctrl-5] = 2;
								else
									cycle[0xB+ctrl] = 1;
								break;

						case 2:
								break;
					}
					count = (ctrl-1) + cycle[ctrl]*16;

					frame_index = 6 + count*7;

					if((set_frames_received & (1<<count)) == 0)
					{
						if(CAN_Multiframe_Length > 7)
							for(int i=1; i<8; i++)
							{
								CAN_Frame[frame_index++] = rx_can.Data[i];
								CAN_Multiframe_Length--;
							}
						else
						{
							for(int i=1; i<=CAN_Multiframe_Length; i++)
							{
								CAN_Frame[frame_index++] = rx_can.Data[i];
							}
							CAN_Multiframe_Length=0;
						}
						set_frames_received |= (1<<count);
					}

					if(CAN_Multiframe_Length == 0)
						can_process_flag = 1;
				}
			}	

		/* Check if received message is same as expected message */ 
			if(can_process_flag)
			{
				can_process_flag = 0;

				//log ecu msgs
				if(buff_length != 0)
				{
					res = f_write(&file, CAN_Frame, buff_length, &bw);
					res = f_write(&file, ",", 1, &bw);
				}
				else
				{
					for(ti = 0; ti < 8; ti++)
					{
						sprintf(temp_buf, "%02x", rx_can.Data[ti]);
						strncat(test_buf,temp_buf,2);
					}
					res = f_write(&file, test_buf, 16, &bw);
					res = f_write(&file, ",", 1, &bw);
					memset(test_buf, 0, sizeof(test_buf));
				}
				tracker_req |= (1<<last_req);
			}

		}
		else
		{
			if( (rx_can.ExtId==response_can_id[0]) || (rx_can.StdId==response_can_id[0]) || (rx_can.ExtId==response_can_id[1]) || (rx_can.StdId==response_can_id[1]) )
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
									Data[last_req][j] = rx_can.Data[j+4];
								}
								if(last_req > 31)
									tracker_req1 |= (1<<(last_req-32));
								else
									tracker_req |= (1<<last_req);
							}

						}
						else
						{
							for(j=0;j<arr_req_param[i].length;j++)
							{
								Data[last_req][j] = rx_can.Data[j+3];
							}
							if(last_req > 31)
								tracker_req1 |= (1<<(last_req-32));
							else
								tracker_req |= (1<<last_req);						}
					}
				}
			}
		}
		CAN_FIFORelease(CAN1, 0);	                   //release fifo
	}	

	void CAN1_RX1_IRQHandler()                 //CAN1 FIFO0 ISR 
	{
		CAN_Receive(CAN1,1,&rx_can);
		NVIC_ClearPendingIRQ(CAN1_RX1_IRQn);         //clear pending interrupt .....best at the start of ISR 
		char temp_buf[20];
		NVIC_DisableIRQ(CAN1_RX1_IRQn);
		can_inactive=0;
		int ti = 0;
	//Log the ID
		switch(rb_flag)
		{
			case 1:
					switch (rx_can.IDE)
					{
						case 0:
						{
							sprintf(temp_buf, "%04x",rx_can.StdId);
							strncat(temp_buf,",",1);
							strncat(rb1,temp_buf,5);
							for(ti = 0; ti < 8; ti++)
							{
								sprintf(temp_buf, "%02x", rx_can.Data[ti]);
								strncat(temp_buf,",",1);
								strncat(rb1,temp_buf,3);
							}
							
							rb1_size=rb1_size+29;
							memset(temp_buf,0,sizeof(temp_buf));
							break;	
						}
						case 1:
						{
							sprintf(temp_buf, "%08x",rx_can.ExtId);
							strncat(temp_buf,",",1);
							strncat(rb1,temp_buf,9);
							for(ti = 0; ti < 8; ti++)
							{
								sprintf(temp_buf, "%02x", rx_can.Data[ti]);
								strncat(temp_buf,",",1);
								strncat(rb1,temp_buf,3);
							}
							
							rb1_size=rb1_size+33;
							memset(temp_buf,0,sizeof(temp_buf));
							break;
						}
					}
				
					strncat(rb1,"\r\n",2);
					rb1_size=rb1_size+2;

					break;

			case 2:
				switch (rx_can.IDE)
				{
					case 0:
					{
						sprintf(temp_buf, "%04x",rx_can.StdId);
						strncat(temp_buf,",",1);
						strncat(rb2,temp_buf,5);
						for(ti = 0; ti < 8; ti++)
						{
							sprintf(temp_buf, "%02x", rx_can.Data[ti]);
							strncat(temp_buf,",",1);
							strncat(rb2,temp_buf,3);
						}

						rb2_size=rb2_size+29;
						memset(temp_buf,0,sizeof(temp_buf));
						break;
					}
					case 1:
					{
						sprintf(temp_buf, "%08x",rx_can.ExtId);
						strncat(temp_buf,",",1);
						strncat(rb2,temp_buf,9);
						for(ti = 0; ti < 8; ti++)
						{
							sprintf(temp_buf, "%02x", rx_can.Data[ti]);
							strncat(temp_buf,",",1);
							strncat(rb2,temp_buf,3);
						}

						rb2_size=rb2_size+33;
						memset(temp_buf,0,sizeof(temp_buf));
						break;
					}
				}

				strncat(rb2,"\r\n",2);
				rb2_size=rb2_size+2;

				break;

		}

		CAN_FIFORelease(CAN1,1);	                   //release fifo
		if(rb1_size < 2100 && rb2_size < 2100)
			NVIC_EnableIRQ(CAN1_RX1_IRQn);
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
			mount_card();
			config_file();
			load_dbc();

		//Initiate the DAQ module 
			init_daq();

		//Initiate timers
			TIM7_Init();
			TIM7_Base_Start_IT();

		//Create a new Temp File
			filename_flag = 0;
			new_file();

			ESP_Init();
			LED_init();
			
	}
//File Handling 
	void write_csv()
	{
			/*
				Check if Rolling Buffer1 is nearing capacity and write it to the SD card
			*/
			NVIC_DisableIRQ(CAN1_RX1_IRQn);
			NVIC_DisableIRQ(CAN1_RX0_IRQn);

			if (rb_flag==1)
			{
				rb_flag = 2;
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
				rb_flag = 1;
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
			NVIC_EnableIRQ(CAN1_RX1_IRQn);
			NVIC_EnableIRQ(CAN1_RX0_IRQn);

	}	

	void RETFROM_SLEEP(void)
	{
   		SCB->SCR &= 0x00000000;     // reenter low-power mode after ISR
		SCB->AIRCR=0x05fa0004;  
	}

	void LED_init()
	{
		RCC_GPIOC_CLOCK_ENABLE_CLK();
		GPIO_Config_Mode (GPIOC,GPIO_PIN_9,GPIO_MODE_OUT);        //LED Connected
		GPIO_Config_OType (GPIOC, GPIO_PIN_9, GPIO_OTYPE_PP);
		GPIO_Config_Speed (GPIOC, GPIO_PIN_9, GPIO_LOW_SPEED);
  	}

int main()
{
	startup();
	update_checked = 2;
	upload_complete = 1;

	int q = 0, l = 0;
	char config_buffer[40];

	//Watchdog Initialised
	TM_WATCHDOG_Init(TM_WATCHDOG_Timeout_32s);
	IWDG_SET=1;

	while(1)
	{
		if(req_done == 1)
		{
			if(heartbeat < 150)	// When one cycle of request is completed, send req again if time elapsed is less than 150s
			{
				req_done = 0;
				last_req = 0;
				if(data_log_mode == 0 || data_log_mode == 1)
				 	update_rb();
				 else
				 {
				 	sprintf(h_buf,"%08x",tracker_req);
				 	strncat(h_buf,",\n",2);
				 	strncat(rb1,h_buf,10);
				 }
				if(rb1_size > 1500)
					write_csv();

				rb_flag = 1;

				switch(data_log_mode)
				{
					case 0:
					case 1: 
							if(arr_req_param[last_req].can_extn_param)
								tx_can.Data[0]=0x03;
							else
								tx_can.Data[0]=0x02;

							tx_can.Data[1]=arr_req_param[last_req].service;
							tx_can.Data[2]=arr_req_param[last_req].PID;

							if(arr_req_param[last_req].can_extn_param)
								tx_can.Data[3]=arr_req_param[last_req].PID1;

							tx_can.ExtId = request_can_id[arr_req_param[last_req].can_id_no];

							if(CAN_Which_MailboxIsEmpty(CAN1) != 0x0F)
								CAN_Transmit(CAN1,&tx_can,CAN_Which_MailboxIsEmpty(CAN1));
							
							break;
					case 2:
					case 3:
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
							}
							break;
				}
				TIM6_Init_can_daq();
				TIM6_Base_Start_IT();

			}
			else  					// Send ECU Parmams requests after 150s (Currently only for CAN)
			{
				req_done = 2;
				last_req = 0;
				can_process_flag = 1;
				buff_length = 0;
				
				if(data_log_mode == 0 || data_log_mode == 1)
				 	update_rb();
				 else
				 {
				 	sprintf(h_buf,"%08x",tracker_req);
				 	strncat(h_buf,",\n",2);
				 	strncat(rb1,h_buf,10);
				 }
	
				write_csv();

				res = f_write(&file, "REQ DONE; ECU PARAMS START\n", 27, &bw);			
				ESP_CMD_SEND(REQ_DONE);
				rb_flag = 1;
				logging = 0;

				if(ecu_params)
				{
					TIM6_Init_ecu_daq();

					switch(data_log_mode)
					{
						case 0:
						case 1:
								if(arr_ecu_param[last_req].can_extn_param)
									tx_can.Data[0]=0x03;
								else
									tx_can.Data[0]=0x02;

								tx_can.Data[1]=arr_ecu_param[last_req].service;
								tx_can.Data[2]=arr_ecu_param[last_req].PID;

								if(arr_ecu_param[last_req].can_extn_param)
									tx_can.Data[3]=arr_ecu_param[last_req].PID1;

								tx_can.ExtId = request_can_id[arr_ecu_param[last_req].can_id_no];

								if(CAN_Which_MailboxIsEmpty(CAN1) != 0x0F)
									CAN_Transmit(CAN1,&tx_can,CAN_Which_MailboxIsEmpty(CAN1));

								break;
						case 2:
						case 3:
								if (kline_init == 4)
								{
								 	kline_buf[0]=0x82;
									kline_buf[1]=0x10;
									kline_buf[2]=0xF1;
									kline_buf[3]=arr_ecu_param[last_req].service;
									kline_buf[4]=arr_ecu_param[last_req].PID;	
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
								}

								break;

					}

					TIM6_Base_Start_IT();
					NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
					NVIC_EnableIRQ(TIM6_DAC_IRQn);

				}
				else if(!ecu_params)
				{
					req_done = 3;
				}
				

			}
				
		}
		
		if(((rb1_size > 2000) || (rb2_size > 2000)) && !broadcast_done)			// Log broadcast params onto SD Card
			write_csv();

		//Reset IWDG	
		if(IWDG_SET)
			TM_WATCHDOG_Reset();	
		
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
			else
			{
				wifi_fail++;
				upload_active = 0;
				upload_complete = 0;
				esp_counter_active = 0;
				esp_counter=0;

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

			if(wifi_fail <= 3)
			{
				GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 0);
				delay(1);
				GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 1);
			}


		}
	
		if(req_done == 3 && file_size != 0)
		{
			sprintf(config_buffer,"%08x\nLOG COMPLETED\nSTM:%d\nDBC:%d\n",tracker_req, ver_5,ver_2);
			res = f_write(&file,config_buffer,40,&bw);
			f_close(&file);
			logging = 2;
			file_size=0;
			filename_flag = 0;

			IFT_fail=0;
			update_config_temp_counter();

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

		if((upload_complete == 1 || wifi_fail >= 3) && (IFT_complete == 1 || IFT_fail>=3) && req_done == 3)
		{
			GPIO_Write_Bit (GPIOC, GPIO_PIN_9,1);		// GREEN LED ON
			GPIO_Write_Bit (GPIOB, GPIO_PIN_15,0);		// BLUE LED OFF


			NVIC_DisableIRQ(TIM6_DAC_IRQn);
			// NVIC_DisableIRQ(TIM7_IRQn);
			NVIC_DisableIRQ(CAN1_RX1_IRQn);
			NVIC_DisableIRQ(CAN1_RX0_IRQn);

			CAN_FilterDeActivate(2);
			CAN_FilterDeActivate(1);
			CAN_FilterDeActivate(0);
			update_checked = 0;
			heartbeat = 0;

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
							NVIC_EnableIRQ(TIM7_IRQn);
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
									SCB->AIRCR=0x05fa0004;
								}
							}
							update_checked = 2;
							timer7_mode = 1;
							
							NVIC_EnableIRQ(TIM7_IRQn);
							NVIC_SetPriority(USART3_IRQn,7);
							break;
						}
					}

				}
				if(heartbeat > 200)
				{
					update_checked = 0;
					heartbeat = 0;
					wifi_fail = 0;
					GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 0);
					for(int i=0; i<1000; i++);
					GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 1);
				}
				if(IWDG_SET)
					TM_WATCHDOG_Reset();

			}
		}

	}
}	
