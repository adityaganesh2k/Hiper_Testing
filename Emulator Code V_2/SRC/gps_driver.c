#include "delay.h"
#include "uart_driver.h"
#include "gps_driver.h"
#include <string.h>


USART_Type param2;

char GPS_RxBuff[]={'0'};//String of GPRMC starting from "MC....... "  //see its extern
volatile int gps_log_flag = 0, msg_process_flag = 0, star_flag = 0, altitude_flag = 0, valid_bit = 0;
char gps_temp[10];
uint8_t com_cnt = 0, gps_cnt = 0;
int g_count = 0;
uint8_t cs_count, checksum_uart = 0, checksum_calc = 0, message_status = 0;
volatile uint8_t freq_check_success = 0, gprmc_check_success = 0;
uint32_t tracker_brd = 0;
uint8_t broadcast_params;
/*Enable or disable the Output sentences */
char  *GPRMC="$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n";
char  *GPRMCGGA="$PMTK314,0,1,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*2C\r\n";
/*Baud rate setting of serial port of the GPS*/	

char *SET_BAUDRATE_9600="$PMTK251,9600*17\r\n";

	
/*Restart the GPS (Factory Reset) */		
char *FULLCOLD_RESTART="$PMTK104*37\r\n";

/*Go to standby mode to save power , (Software on Host side sends any byte to wake up from standby mode.) */	
char *STANDBYMODE="$PMTK161,0*28\r\n";

/*Set Update rate*/

char *UPDATE_5HZ="$PMTK220,200*2C\r\n";
char *UPDATE_1HZ="$PMTK220,1000*1F\r\n";
char *UPDATE_2HZ="$PMTK220,500*2B\r\n";

//ASCII to HEX converter

uint8_t ascii_to_hex(uint8_t ch)
{
	uint8_t cs, ca;
	cs = ch;
	ca = ch;
	cs &= NUMBER;
	ca &= ALPHABET;
	if(cs == 0x30)
	{
		ch &= CONVERT;
		return ch;

	}
	else if(ca == 0x40)
	{
		ch &= CONVERT;
		ch += ALPH_OFFSET;
		return ch;
	}

}

/**
	*Enables and Initializes every thing related to GPS 
  *Parameter :
  *Return: None
	*/
void GPS_Init()
{

//GPIO clock enable
RCC_GPIOC_CLOCK_ENABLE_CLK();
	
//GPIO clock enable
RCC_GPIOB_CLOCK_ENABLE_CLK();
	
param2.USART_BaudRate=9600;
param2.USART_WordLength=USART_WORDLENGTH_8B;
param2.USART_HardwareFlowControl=USART_HARDWAREFLOWCONTROL_NONE;
param2.USART_Mode=USART_MODE_Rx_Tx;
param2.USART_StopBits=USART_STOPBITS_1;
param2.USART_Parity=USART_PARITY_NO;
	
GPIO_Config_Mode (GPIOC,GPIO_PIN_3,GPIO_MODE_OUT);        //GPS enable Connected
GPIO_Config_OType (GPIOC, GPIO_PIN_3, GPIO_OTYPE_PP); 
GPIO_Config_Speed (GPIOC, GPIO_PIN_3, GPIO_LOW_SPEED);
GPIO_Write_Bit(GPIOC, GPIO_PIN_3,1);	

//PB14.GPIO_Label=GPS_TIMEMARK	also
GPIO_Config_Mode (GPIOB,GPIO_PIN_14,GPIO_MODE_IN);        //GPS TIMEMARK
GPIO_Config_Speed (GPIOB, GPIO_PIN_14, GPIO_LOW_SPEED);    
GPIO_Config_PuPd	(GPIOB,GPIO_PIN_14,GPIO_PUPD_NOPULL);
	

USART2_Init(&param2);
USART_Enable(USART2);	 

USART_IntMaskEnable(USART2,USART_IT_RXNE);
NVIC_SetPriority(USART2_IRQn,6);	
NVIC_ClearPendingIRQ(USART2_IRQn);             //clear any pending interrupt
NVIC_EnableIRQ(USART2_IRQn);   

}


/**
	*Send PMTK Cmd to Configure the GPS 
  *Parameter :PMTK Cmd String
  *Return: None
	*/
void GPS_PMTKCmd(char *s)
{

	int i=0;
	while(s[i]!='\0')          //Send the entire cmd string
	{	
		USART_OutChar(USART2,s[i]);
		i++;
	}

}

//sample
void GPS_NMEAString()
{
	char rxch;
	rxch = USART_InChar(USART2); //grab the rx char
	msg_process_flag = 0;
	switch(message_status)
	{
		case 0:
			if(rxch == '$')
			{
				message_status = 1;
				gps_cnt = 0;

			}
			break;

		case 1:
			GPS_RxBuff[gps_cnt++] = rxch;

			if(rxch != '*')
				checksum_calc ^= rxch;
			else
				message_status = 2;
			break;

		case 2:
			GPS_RxBuff[gps_cnt++] = rxch;

			if(rxch == CR)
			{
				gps_cnt-=3;
				checksum_uart = ascii_to_hex(GPS_RxBuff[gps_cnt]) << 4;
				gps_cnt+=1;
				checksum_uart += ascii_to_hex(GPS_RxBuff[gps_cnt]);

				if(checksum_uart == checksum_calc)
				{
					msg_process_flag = 1;
				}
				else
				{
					tracker_brd |= (1<<(broadcast_params + 2));
					gps_cnt = 0;
					message_status = 0;
					checksum_calc = 0;
					checksum_uart = 0;
					msg_process_flag = 2;
					memset(GPS_RxBuff,0,sizeof(GPS_RxBuff));
				}
			}
			break;

		default:
			break;
	}

	if(gps_cnt > 72)
	{
		tracker_brd |= (1<<(broadcast_params + 3));
		gps_cnt = 0;
		message_status = 0;
		checksum_calc = 0;
		checksum_uart = 0;
		msg_process_flag = 3;
		memset(GPS_RxBuff,0,sizeof(GPS_RxBuff));
	}


}

void GPS_NMEAData(GPS_Data *data)
{

		uint8_t gpsi = 0;
		int i = 0;

		if(GPS_RxBuff[3] == 'M')
		{
			for (gpsi = 0; gpsi <= 72; gpsi++)
			{
				switch(GPS_RxBuff[gpsi])
				{
					case ',':
					{
						switch(com_cnt)
						{

							case 1:
							{
								if(g_count==10)
								{
									strncpy(data->time,gps_temp,6);
								}
								break;
							}

							case 3:
							{
								if(g_count==9 && gps_log_flag)
								{
									strncpy(data->lat,gps_temp,9);
								}
								break;
							}
							case 5:
							{
								if(g_count==10 && gps_log_flag)
								{
									strncpy(data->longi,gps_temp,10);
								}
								break;
							}
							case 9:
							{
								if (g_count==6 && gps_log_flag)
								{
									strncpy(data->date,gps_temp,g_count);
								}
								break;
							}
							default:
								break;
						}
						com_cnt++;
						g_count=0;
						memset(gps_temp,0,10);
						break;
					}
					case '\0':
					{
						com_cnt=0;
	//					Add if needed
	//					g_count = 0;
	//					memset(gps_temp, 0 , sizeof(gps_temp));
						memset(GPS_RxBuff, 0, sizeof(GPS_RxBuff));
						break;
					}
					default :
					{
						if(com_cnt==2 && GPS_RxBuff[gpsi] == 'A')
							gps_log_flag=1;
						else
							gps_temp[g_count++] = GPS_RxBuff[gpsi];

						break;
					}
				}
			}
			g_count = 0;
			memset(gps_temp, 0 , sizeof(gps_temp));

		}
		else if(GPS_RxBuff[3] == 'G')
		{
			for (gpsi = 0; gpsi <= 72; gpsi++)
			{
				switch(GPS_RxBuff[gpsi])
				{
					case ',':
					{
						switch(com_cnt)
						{
							case 9:
							{
								if(altitude_flag)
								{
									//strncpy(data->altitude,gps_temp,g_count);

									memset(data->altitude,'0',6);

									g_count--;  // -1 for index

									for(i = 5; g_count>=0 && i>=0; i--, g_count--)
									{
										data->altitude[i] = gps_temp[g_count];
									}

								}
								break;
							}
							default:
								break;
						}
						com_cnt++;
						g_count=0;
						memset(gps_temp,0,10);
						break;
					}
					case '\0':
					{
						com_cnt=0;
	//					Add if needed
	//					g_count = 0;
	//					memset(gps_temp, 0 , sizeof(gps_temp));
						memset(GPS_RxBuff, 0, sizeof(GPS_RxBuff));
						break;
					}
					default :
					{
						if(com_cnt==6)
						{
							if(GPS_RxBuff[gpsi] == '1')
								altitude_flag=1;
							valid_bit = GPS_RxBuff[gpsi];
						}
						else
							gps_temp[g_count++] = GPS_RxBuff[gpsi];

						break;
					}
				}
			}
			g_count = 0;
			memset(gps_temp, 0 , sizeof(gps_temp));
		}
		memset(GPS_RxBuff, 0, sizeof(GPS_RxBuff));
		gpsi = 0;
		gps_cnt = 0;
		message_status = 0;
		checksum_calc = 0;
		msg_process_flag = 0;
//		cs_count = 0;

}

void FREQUENCY_CHECK()
{		
	char rxch;
	rxch = USART_InChar(USART2); //grab the rx char

	switch(message_status)
	{
		case 0:
			if(rxch == '$')
			{
				message_status = 1;
				gps_cnt = 0;

			}
			break;

		case 1:
			GPS_RxBuff[gps_cnt++] = rxch;

			if(rxch != '*')
				checksum_calc ^= rxch;
			else
				message_status = 2;
			break;

		case 2:

			if(GPS_RxBuff[gps_cnt-4] == '0')
			{
				if(GPS_RxBuff[gps_cnt-2] == '3')
				{
					freq_check_success = 1;

				}
				else
					freq_check_success = 0;
			}
			else if(GPS_RxBuff[gps_cnt-4] == '4')
			{
				if(GPS_RxBuff[gps_cnt-2] == '3')
				{
					gprmc_check_success = 1;

				}
				else
					gprmc_check_success = 0;
			}

			gps_cnt = 0;
			message_status = 0;
			checksum_calc = 0;
			checksum_uart = 0;
			memset(GPS_RxBuff,0,sizeof(GPS_RxBuff));
				
			
			break;

		default:
			break;
	}
}
