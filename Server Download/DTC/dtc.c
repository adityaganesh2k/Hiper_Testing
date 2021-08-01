#include "stm32f446xx.h"
#include "gpio_driver.h"
#include "can_driver2.h"
#include "delay.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "rcc_clock.h"
#include "sdio_driver.h"
#include "ff.h"
#include "timer_driver.h"
#include "math.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

	//Struct for Cmds & Response
		struct cmd_format
		{
			uint8_t size;
			uint8_t type;
			uint8_t message[15];
			uint8_t sucess_index;
			uint8_t failure_index;
		};
		struct code_area_tasks
		{
			uint32_t start_address;
			uint32_t finish_address;
			uint32_t payload_pointer;
			uint8_t start_response_counter;
			uint32_t end_frame;
			uint32_t checksum_end_frame;
		};
		struct ret_val
		{
			uint8_t num_frames;
			uint16_t checksum;
		};

	// Variables for Cmd & Response 
		struct cmd_format cmd_resp[100];
		struct code_area_tasks bin_area_task[6];
		uint8_t bin_area_size=0;
		uint8_t bin_area_counter=0;
		uint8_t cmd_resp_counter=0;
		uint8_t baud_change_counter=0;
		uint8_t baud_changes_size=0,buff_length=0;
		uint8_t cmd_resp_size=0;
		uint8_t full_checksum_error=0;
		uint32_t seed, response;
		uint8_t temp_buf_frame[2];

	//TIMER Parameters
		TIM_Base_InitTypeDef TIM6_param;
		TIM_Base_InitTypeDef TIM7_param;
		uint8_t timer_mode=0;	
	//Code Area variables
		uint8_t check_code = 0;
		uint16_t counter_code_area = 1;
		uint32_t payload_read=0;

	//System Variables 
		uint8_t LED_ON=0;

	//CAN Variables 
		CAN_ClockParamTypeDef ClkArg;        //bit timing configuration
		CAN_ConfigParamTypeDef ConfArg;      //additional configuration 
		CAN_TxMsgType tx_can;

		CAN_RxMsgType rx_can;

		uint32_t ECU_CAN_ID;
		uint32_t Response_CAN_ID;
		uint8_t cmd_status_flag=0;
		uint8_t CAN_Multiframe_Length;
		char CAN_Frame[256],CAN_Frame1[8];
		uint8_t frame_index = 0;
		uint8_t cycle[17];					// Cycle counter for each frame number
		uint8_t ctrl=0;
		uint64_t set_frames_received;		// Each bit is set when a frame number has been recived 
		uint8_t can_process_flag=1;			// Ensure set to 1 before sending out a request
		uint8_t count;
		uint8_t can_inactive=0;
		uint8_t Active_TxFlag = 1;   //either 1 or 2
		uint8_t ensure_1=0;
		_Bool  tx1_empty_flag = 1, tx2_empty_flag = 1;
		uint8_t kline_process_flag = 0;
		uint16_t payload_size_1, payload_size_2, full_checksum = 0;
		uint16_t pay_check_calc = 0;
		uint8_t pay_check_read = 0;
		uint32_t pay_sent_bytes = 0; //total no.of payload bytes sent
		uint8_t send_multiframes=0;

	// Buffers
		uint8_t Data1[40][8], Data2[40][8], Data3[40][8], Data4[8];			// Data1 - Payload Buffer 1
																			// Data2 - Payload Buffer 2
																			// Data3 - CMD Multiframe Buffer
																			// Data4 - 4 Byte Payload Buffer
	//	Track number of frames in payload
		uint8_t Tx1_num_frames;
		uint8_t Tx2_num_frames;

	// Timing control
		uint8_t wait=0;
		uint8_t wait_30s = 0;  // 0 - Normal Operation; 1 - Waiting for 30s under process; 2 - Completed waiting for 30s

	//FATFS Variables
		FATFS fs;            	  	/* Filesystem object */
   		FIL file,bin;           			/* File object */
    	FRESULT res;        			/* API result code */
		FILINFO fno;							/* File information */
		UINT bw, br;            	/* Bytes written */	
		FSIZE_t payload_pointer;
		_Bool task_status=0;
		uint16_t frame_number_1 = 0, frame_number_2 = 0, frames_confirmed_sent=0, frame_number_temp=0;

	// New file
		uint32_t temp_file_counter=0;
		uint8_t filename_flag = 0;
		char filename[15]="t_";
		char temp_buf[20], test_buf[512];
		uint32_t heartbeat = 0;
		
	/*
		Init the SDIO & mount the card for further use 
	*/
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
	}

	/*
		Load the DBC data to required pointers
	*/
	void load_cmd()
	{
		//Required variable for this funct
			char temp_buff[80];
			uint8_t i,j,k,m,n,p;
			k=1;
			i=0;
			j=0;
			char s[9];
		
		//Open required file
			res= f_open(&file, "dtc.csv", FA_OPEN_EXISTING|FA_READ);
	
		//First line has some additional charc attached to what's in the file. This is a workaround the bug
			f_gets (temp_buff,70,&file);
			memset(temp_buff,0,70);

		//Read the total # of cmd_resp lines 
			f_gets (temp_buff,70,&file);
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
			cmd_resp_size=atoi(s);				//total Number of cmd_response 
			memset(s,0,sizeof(s));
			memset(temp_buff,0,sizeof(temp_buff));
			k=1;
			i=0;

	//Loading CMDs & Responses 
		j=0;			
		while(j<cmd_resp_size)
		{
			memset(temp_buff,0,sizeof(temp_buff));
			memset(s,0,sizeof(s));			
			f_gets(temp_buff,70,&file);
			
			k=1;
			i=0;
			m=0;
			while(k)
			{
				if(temp_buff[i] == ',') //Process buffer s  
				{
					switch(m)
					{
						case 0:							// Read cmd type
						{
							s[n]='\0';
							cmd_resp[j].type=atoi(s);
							memset(s,0,sizeof(s));
							n=0;
							p=0;
							m=1;
							i++;						
							break;
						}
						case 1:  						// Read Response size 
						{
							s[n]='\0';
							cmd_resp[j].size=atoi(s);
							memset(s,0,sizeof(s));
							n=0;
							p=0;
							m=2;
							i++;
							break;
						}
						case 2:							// Read Response message
						{
							cmd_resp[j].message[p]= (ascii_to_hex(s[0])<<4) + ascii_to_hex(s[1]);
							memset(s,0,sizeof(s));
							p++;
							n=0;
							i++;
							if(p>cmd_resp[j].size-1)
							{
								m=3;
							}
							break;
						}
						case 3:							// Read next cmd index (if response sucessful )
						{
							s[n]='\0';
							cmd_resp[j].sucess_index=atoi(s);
							memset(s,0,sizeof(s));
							n=0;
							p=0;
							i++;
							m=4;									
							break;
						}
						case 4:						// Read prev cmd index (if response unsucessful )
						{
							s[n]='\0';
							cmd_resp[j].failure_index=atoi(s);
							memset(s,0,sizeof(s));
							n=0;
							p=0;
							m=5;
							i++;
							k=0;
							break;
						}								
						default :
							break;
					}
				}
				else 									//Add current char to buffer s  
				{
					s[n]=temp_buff[i];
					n++;
					i++;
				}
			}
			j++;	
		}

	// Read the CAN ID of ECU
		k=1; 
		i=0;
		j=0;
		memset(s,0,sizeof(s));
		memset(temp_buff,0,sizeof(temp_buff));
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
		sscanf(s,"%8x",&ECU_CAN_ID);
		memset(s,0,sizeof(s));
		memset(temp_buff,0,sizeof(temp_buff));
		k=1;
		i=0;

	// Read the response CAN ID
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
		sscanf(s,"%8x",&Response_CAN_ID);
		
	//Close Command file
		res= f_close(&file);		

	}

	/*
		TIM_6 K-Line Init 
		Priority - 2
		Timer Period -25ms
	*/
	void TIM6_Init_kline_init()
	{
		RCC_TIM6_CLOCK_ENABLE_CLK();
		
		TIM6_param.Period = 40;
		TIM6_param.Prescaler = 47999;
		TIM6_Base_Init(&TIM6_param);
		
		NVIC_SetPriority(TIM6_DAC_IRQn,3);
		NVIC_ClearPendingIRQ(TIM6_DAC_IRQn); 
		NVIC_EnableIRQ(TIM6_DAC_IRQn);
	}

	/*
		TIM_7 Init 
		Priority - 2
		Timer Period -1s
	*/
	void TIM7_Init()
	{
		RCC_TIM7_CLOCK_ENABLE_CLK();	
		TIM7_param.Period = 999;
		TIM7_param.Prescaler = 47999;
		TIM7_Base_Init(&TIM7_param);		
		NVIC_SetPriority(TIM7_IRQn,4);
		NVIC_ClearPendingIRQ(TIM7_IRQn); 
		NVIC_EnableIRQ(TIM7_IRQn);
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
		
		//////////////////////
		//can_stb
		/*FOR can_stb enable (got from Initialization file)*/	
		GPIO_Config_Mode (GPIOB,GPIO_PIN_7,GPIO_MODE_OUT);        
		GPIO_Config_OType (GPIOB, GPIO_PIN_7, GPIO_OTYPE_PP); 
		GPIO_Config_Speed (GPIOB, GPIO_PIN_7, GPIO_LOW_SPEED);    
		GPIO_Write_Bit (GPIOB, GPIO_PIN_7,0);				
	}

	void can_filt_init()
	{

		CAN_FilterInit_SlaveBankStart(2);

	//Assign filter for responses for requested params
		CAN_FilterInit_Req(0);
		CAN_FilterInit_FilterMode(0,CAN_FILTERMODE_IDMASK );
		CAN_FilterInit_FilterScale(0,CAN_FILTERSCALE_32BIT );
		CAN_Filter_IDMaskModify(0,1,((ECU_CAN_ID<<3)|0X04));
		CAN_Filter_IDMaskModify(0,2,((0xFFFFFF00<<3)|0X04));                             			//mask for PGN
		CAN_FilterInit_FilterAssign(0,0);
		CAN_FilterActivate(0);
		CAN_FilterInit_Quit();

	}

	/*
		CAN Setup 
		Baud Rate - 250 kbps
		Interrupt Priority - 2 & 3	
	*/
	void setup_can()
	{
		ClkArg.BS1=CAN_BS1_13TQ;
		ClkArg.BS2=CAN_BS2_2TQ;                             //bit timing
		ClkArg.Mode=CAN_MODE_NORMAL;                        //NORMAL MODE................
		ClkArg.Prescaler=6;                                //for 250kbps-6 For 500kbps 3
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
		
		can_filt_init();
		
		CAN_IntMaskEnable(CAN1, CAN_IT_FMP0 );  //set the interrupt    Rx interrupt FIFO0
				
	//set the interrupt    Rx interrupt FIFO0
		NVIC_ClearPendingIRQ(CAN1_RX0_IRQn);   //clear any pending interrupt
		NVIC_SetPriority(CAN1_RX0_IRQn,1);
		NVIC_EnableIRQ(CAN1_RX0_IRQn);         //enable from NVIC
		
	}
	

	void Send_Multiframes(uint8_t Data[][8], uint8_t num_frames)
	{
		
		send_multiframes = 0;	

		for(int i=0; i<8; i++)
			tx_can.Data[i] = Data[0][i];

		while(CAN_Which_MailboxIsEmpty(CAN1)==0x0F)
		{}
		CAN_Transmit(CAN1,&tx_can,CAN_Which_MailboxIsEmpty(CAN1));

		while(send_multiframes == 0)
		{}

		for(int i=1; i<=num_frames; i++)						// Add number of frames to be sent here
		{
			for(int j=0; j<8; j++)
				tx_can.Data[j] = Data[i][j];
			
			while(CAN_Which_MailboxIsEmpty(CAN1)==0x0F)
			{}
			CAN_Transmit(CAN1,&tx_can,CAN_Which_MailboxIsEmpty(CAN1));
		}
		can_process_flag = 1;

	}

	struct ret_val CAN_PACK_MULTIFRAMES(uint8_t Data[][8], uint8_t *Buffer, uint8_t length)   // Buffer - bytes after length byte
	{
		struct ret_val obj;
		obj.checksum=0;
		obj.num_frames=0;

		uint8_t i=0, k=1,j=1;
		int b;

		memset(Data,0,sizeof(Data));

		Data[0][0] = 0x10;


		for(b = 1; b < 38; b++)						// Initialize frame numbers for all Data Frames
		{
			Data[b][0] =  (b & 0x0F) + 0x20;
		}

		Data[0][1] = length;					// Store total length of multiframe

		for(i=0;i<6;i++)							// Store first frame separately
		{
			Data[0][i+2] = Buffer[i];
			obj.checksum += Buffer[i];
		}
		
		length -= 6; // Control variable to pack bytes
		
		while(i<Data[0][1])							// Receive remaining bytes
		{
			if(length >= 7)
			{
				for(j=1; j<8; j++)					// Pack 7 bytes into each frame except last 
				{
					Data[k][j] = Buffer[i];
					obj.checksum += Buffer[i];
					i++;
				}
				length -= 7;
			}
			else									// Pack remaining bytes into last frame
				for(j=1; j<=length; j++)
				{
					Data[k][j] = Buffer[i];
					obj.checksum += Buffer[i];
					i++;
				}

			k++;
			obj.num_frames++;
		}

		return obj;
	}


	/*
		CAN IRQ Handler for the FIFO-1
	*/
	void CAN1_RX0_IRQHandler()                 //CAN1 FIFO1 ISR 
	{
		CAN_Receive(CAN1,0,&rx_can);
		NVIC_ClearPendingIRQ(CAN1_RX0_IRQn);         //clear pending interrupt .....best at the start of ISR 
		
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
	/* ---- Logged Multiframe message ---- */
		else if(rx_can.Data[0] == 0x30)
		{
			send_multiframes = 1;
			can_process_flag = 0;
		}

	/* Check if received message is same as expected message */ 
		if(can_process_flag)
		{
			can_process_flag = 0;

			if(buff_length != 0)
			{
				memset(test_buf, 0, sizeof(test_buf));
				for(int i = 0; i < buff_length; i++)
				{
					sprintf(temp_buf, "%02x", CAN_Frame[i]);
					strncat(test_buf,temp_buf,2);
				}
				res = f_write(&file, test_buf, buff_length*2, &bw);
				res = f_write(&file, ",", 1, &bw);
				buff_length = 0;
			}
			else
			{
				memset(test_buf, 0, sizeof(test_buf));
				for(int i = 0; i < 8; i++)
				{
					sprintf(temp_buf, "%02x", rx_can.Data[i]);
					strncat(test_buf,temp_buf,2);
				}
				res = f_write(&file, test_buf, 16, &bw);
				res = f_write(&file, ",", 1, &bw);
			}

			//Check if mesasge is similar to what we expect 
			if(cmd_resp[cmd_resp_counter].size > 7)
				for(int r = 0; r < cmd_resp[cmd_resp_counter].size; r++)
				{
					if(CAN_Frame[r] == cmd_resp[cmd_resp_counter].message[r])
						cmd_status_flag = 2;
					else
					{
						cmd_status_flag = 0;
						break;
					}
				}
			else 
				for(int r = 0; r < cmd_resp[cmd_resp_counter].size; r++)
				{
					if(rx_can.Data[r] == cmd_resp[cmd_resp_counter].message[r])
						cmd_status_flag = 2;
					else
					{
						cmd_status_flag = 0;
						break;
					}
				}

		// Process the response
			if(cmd_status_flag == 2)	// If expected replies
			{
				cmd_status_flag = 0;

				switch (cmd_resp[cmd_resp_counter].type)
				{
					case 8:
					{
						NVIC_DisableIRQ(TIM6_DAC_IRQn);
						NVIC_DisableIRQ(TIM7_IRQn);
						GPIO_Write_Bit (GPIOC, GPIO_PIN_9,1);
						LED_ON=7;
						break;
					}
				}

				if((cmd_resp[cmd_resp_counter].type != 8))
				{
					cmd_resp_counter=cmd_resp[cmd_resp_counter].sucess_index;
				}
			}
			else // IF unexpected replies
			{
				// if((cmd_resp[cmd_resp_counter].type != 8))				// Use if neg response if followed by expected response (DTC Check cmd)
					cmd_resp_counter=cmd_resp[cmd_resp_counter].failure_index;
			}

			//Activate TIM6 to send the next cmd
				can_inactive=0;

				if(cmd_resp[cmd_resp_counter].type != 8)
				{
					timer_mode=1;
					NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
					NVIC_EnableIRQ(TIM6_DAC_IRQn);
					TIM6_Base_Start_IT();
				}
		}

	CAN_FIFORelease(CAN1, 0);	                   //release fifo
}
	/*
	TIM6 IRQ handler 
	*/	
	void TIM6_DAC_IRQHandler()                 //IRQ FOR TIMER=25ms
	{
		struct ret_val obj;

		/* TIM Update event */
		if((TIM6->SR & (TIMER_UPDATE_FLAG)) == TIMER_UPDATE_FLAG)
		{
			TIM6->SR = ~(TIMER_INTERRUPT);  
		}
		
		NVIC_DisableIRQ(TIM6_DAC_IRQn);
		can_process_flag = 1;

		if(cmd_resp[cmd_resp_counter].type==0)
		{
			if(cmd_resp[cmd_resp_counter].size > 7)					
			{
				obj = CAN_PACK_MULTIFRAMES(Data3, cmd_resp[cmd_resp_counter].message, cmd_resp[cmd_resp_counter].size);	// Define it. packing_can file
			
																						// Get number of frames to be sent also from this func
				cmd_resp_counter=cmd_resp[cmd_resp_counter].sucess_index;
				Send_Multiframes(Data3, obj.num_frames);		
				
			}
			else
			{
				for(int i=0; i<cmd_resp[cmd_resp_counter].size; i++)
					tx_can.Data[i] = cmd_resp[cmd_resp_counter].message[i];

				cmd_resp_counter=cmd_resp[cmd_resp_counter].sucess_index;

				while(CAN_Which_MailboxIsEmpty(CAN1)==0x0F)
				{}
				CAN_Transmit(CAN1,&tx_can,CAN_Which_MailboxIsEmpty(CAN1));
			}
		}
	}

	void TIM7_IRQHandler()                 //IRQ FOR TIMER=500ms
	{		
		/* TIM Update event */
		if((TIM7->SR & (TIMER_UPDATE_FLAG)) == TIMER_UPDATE_FLAG)
		{
			TIM7->SR = ~(TIMER_INTERRUPT);
		}

		if(LED_ON)
		{
			LED_ON=0;	
		}
		else 
		{
			LED_ON=1;			
		}	

		can_inactive++;
		heartbeat++;

	}

  /*
		Using LED as a debug tool to check load on uC
	*/
	void LED_init()
	{
		RCC_GPIOC_CLOCK_ENABLE_CLK();
		GPIO_Config_Mode (GPIOC,GPIO_PIN_9,GPIO_MODE_OUT);        //LED Connected
		GPIO_Config_OType (GPIOC, GPIO_PIN_9, GPIO_OTYPE_PP); 
		GPIO_Config_Speed (GPIOC, GPIO_PIN_9, GPIO_LOW_SPEED);
  	}


/*
	Main Loop 
*/
int main()
{
	
	//Clock Setup
		InitializeClock();
		SystemCoreClockUpdate();
		
	//For delay
		uint32_t x= SysTick_Config(SystemCoreClock /1000);
	
	//Initiate timers USART
		mount_card();
		load_cmd();

	//Open required file
		new_file();
		
	//CAN 
		Init_CAN();
		setup_can();
	
		TIM7_Init();
		TIM6_Init_kline_init();
		LED_init();
		
		tx_can.IDE = CAN_ID_EXT;
		tx_can.DLC = 8;
		tx_can.ExtId = Response_CAN_ID;

		TIM6_Base_Start_IT();
		TIM7_Base_Start_IT();

	while(1)
    {
		if(LED_ON == 1)
		{
			GPIO_Write_Bit (GPIOC, GPIO_PIN_9,0);			
		}
		else if(LED_ON == 0)
		{
			GPIO_Write_Bit (GPIOC, GPIO_PIN_9,1);					
		}

		if(LED_ON == 7 || heartbeat == 150)
		{
			res = f_close(&file);

			char s[5], config_buffer[20];

			res = f_open(&file, "config.csv", FA_WRITE|FA_READ);

			memset(config_buffer,0,sizeof(config_buffer));
			f_gets(config_buffer,20,&file);

			memset(config_buffer,0,sizeof(config_buffer));
			f_gets(config_buffer,20,&file);

			memset(config_buffer,0,sizeof(config_buffer));
			strncpy(config_buffer, "1\nSTM.bin\n", 10);
			res = f_write(&file, config_buffer,10, &bw);

			res = f_close(&file);

			SCB->AIRCR=0x05fa0004;

		}

    }
}


