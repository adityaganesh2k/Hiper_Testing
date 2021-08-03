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
#include "gps_driver.h"
#include "timer_driver.h"
#include "math.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "k_line.h"

//CAN Variables
CAN_ClockParamTypeDef ClkArg;	//bit timing configuration
CAN_ConfigParamTypeDef ConfArg; //additional configuration
CAN_TxMsgType tx_can[15], reply;
CAN_RxMsgType rx_can;
uint8_t i, j, k;
uint8_t ENG_RUNNING = 0;
uint8_t IGN_ON = 0;
char s[3];

//GPS
GPS_Data z;

//K-Line
USART_Type param6;
uint8_t kline_buf[45];
uint8_t Kline_RxBuff[20];
uint8_t kline_len = 0;
uint8_t kline_crc;
uint8_t last_req = 0;
// uint8_t message_strt=0;
uint8_t b;
uint8_t x = 0;
uint8_t msg_status = 0, start_exec = 0, flag = 0;
uint8_t buff_length = 0, kline_cnt = 0, check_calc = 0, check_uart = 0;
volatile int kline_process_flag = 0;
uint8_t start_comm_response[] = {0x83, 0x83, 0xF1, 0x10, 0xC1, 0xDF, 0x8F, 0xB3};

/*
		SIM28 Usart interrupt handler - stores the incoming GPRMC message in a string 
		When a complete message is recieved stores the data in z(GPS DATA) 
	*/
uint16_t tim = 0;
uint16_t TIMEOUT = 0;
uint8_t do_once=0;
TIM_Base_InitTypeDef TIM7_param;
void USART2_IRQHandler()
{

	GPS_NMEAString();
	if (msg_process_flag == 1)
		GPS_NMEAData(&z);
	USART_IntFlagClear(USART2, USART_IT_RXNE);
	NVIC_ClearPendingIRQ(USART2_IRQn); //clear pending interrupt .....best at the start of ISR
}
/*
		Intitalize CAN Peripheral & Interrupts
	*/
void Init_CAN()
{

	RCC_GPIOB_CLOCK_ENABLE_CLK(); //GPIO CLOCK
	RCC_CAN1_CLOCK_ENABLE_CLK();  // CAN clock enabling

	GPIO_Config_Mode(GPIOB, GPIO_PIN_8, GPIO_MODE_AF); //Alternate function  for CAN Rx
	GPIO_Config_PuPd(GPIOB, GPIO_PIN_8, GPIO_PUPD_UP);
	GPIO_Config_Speed(GPIOB, GPIO_PIN_8, GPIO_HIGH_SPEED);

	GPIO_Config_Mode(GPIOB, GPIO_PIN_9, GPIO_MODE_AF); //Alternate function  for CAN Tx
	GPIO_Config_OType(GPIOB, GPIO_PIN_9, GPIO_OTYPE_PP);
	GPIO_Config_PuPd(GPIOB, GPIO_PIN_9, GPIO_PUPD_UP);
	GPIO_Config_Speed(GPIOB, GPIO_PIN_9, GPIO_HIGH_SPEED);

	GPIO_Config_AF(GPIOB, GPIO_PIN_9, 9); /*Alt function configuration function */
	GPIO_Config_AF(GPIOB, GPIO_PIN_8, 9); /*Alt function configuration function */

	//////////////////////
	//can_stb
	/*FOR can_stb enable (got from Initialization file)*/
	GPIO_Config_Mode(GPIOB, GPIO_PIN_7, GPIO_MODE_OUT);
	GPIO_Config_OType(GPIOB, GPIO_PIN_7, GPIO_OTYPE_PP);
	GPIO_Config_Speed(GPIOB, GPIO_PIN_7, GPIO_LOW_SPEED);
	GPIO_Write_Bit(GPIOB, GPIO_PIN_7, 0);
}

/*
		CAN Setup 
		Baud Rate - 250 kbps
		Interrupt Priority - 2 & 3	
	*/
void setup_can()
{
	ClkArg.BS1 = CAN_BS1_13TQ;
	ClkArg.BS2 = CAN_BS2_2TQ;	   //bit timing
	ClkArg.Mode = CAN_MODE_NORMAL; //NORMAL MODE................
	ClkArg.Prescaler = 6;		   //for 250kbps-6 For 500kbps 3
	ClkArg.SJW = CAN_SJW_1TQ;

	ConfArg.AutoBusOff = CAN_ABOM_ENABLE;
	ConfArg.AutoWakeUp = CAN_AWUM_ENABLE;
	ConfArg.NoAutoRetransmission = CAN_NART_DISABLE; // additional features
	ConfArg.ReceiveFifoLocked = CAN_RXFIFO_OVERRUN_LOCKED;
	ConfArg.TimeTriggeredMode = CAN_TTCM_DISABLE;
	ConfArg.TransmitFifoPriority = CAN_TXFIFO_PRIO_BY_ID;

	CAN_Init_Req(CAN1);
	CAN_Init_Clock(CAN1, &ClkArg); //initialization and configuration
	CAN_Init_Config(CAN1, &ConfArg);
	CAN_DBGFreeze(CAN1, DISABLE);
	CAN_Init_Quit(CAN1);

	CAN_FilterInit_Req(0);
	CAN_FilterInit_FilterMode(0, CAN_FILTERMODE_IDMASK);
	CAN_FilterInit_FilterScale(0, CAN_FILTERSCALE_32BIT);
	CAN_Filter_IDMaskModify(0, 1, ((0x18DB33F1 << 3)));
	CAN_Filter_IDMaskModify(0, 2, ((0x00 << 3)));
	CAN_FilterInit_FilterAssign(0, 0);
	CAN_FilterActivate(0);

	CAN_FilterInit_Req(1);
	CAN_FilterInit_FilterMode(1, CAN_FILTERMODE_IDMASK);
	CAN_FilterInit_FilterScale(1, CAN_FILTERSCALE_32BIT);
	CAN_Filter_IDMaskModify(1, 1, ((0x18DB33F1 << 3)));
	CAN_Filter_IDMaskModify(1, 2, ((0x00 << 3)));
	CAN_FilterInit_FilterAssign(1, 1);
	CAN_FilterActivate(1);

	CAN_FilterInit_Quit();

	CAN_IntMaskEnable(CAN1, CAN_IT_FMP0); //set the interrupt    Rx interrupt FIFO0

	CAN_IntMaskEnable(CAN1, CAN_IT_FMP1); //set the interrupt    Rx interrupt FIFO1

	//set the interrupt    Rx interrupt FIFO0
	NVIC_ClearPendingIRQ(CAN1_RX0_IRQn); //clear any pending interrupt
	NVIC_SetPriority(CAN1_RX0_IRQn, 2);
	//NVIC_EnableIRQ(CAN1_RX0_IRQn); //enable from NVIC

	//set the interrupt    Rx interrupt FIFO1
	NVIC_ClearPendingIRQ(CAN1_RX1_IRQn); //clear any pending interrupt
	NVIC_SetPriority(CAN1_RX1_IRQn, 3);
	//NVIC_EnableIRQ(CAN1_RX1_IRQn); //enable from NVIC
}

/*
		CAN IRQ Handler for the FIFO-1
	*/
void CAN1_RX0_IRQHandler() //CAN1 FIFO1 ISR
{
	CAN_Receive(CAN1, 0, &rx_can);
	NVIC_ClearPendingIRQ(CAN1_RX0_IRQn); //clear pending interrupt .....best at the start of ISR
	if (rx_can.ExtId == 0x18DA00FA)
	{
		reply.ExtId = 0x18DAFA00;
		reply.Data[0] = 0x07;
		reply.Data[1] = rx_can.Data[1] + 0x40;
		reply.Data[2] = rx_can.Data[2];
		reply.Data[3] = rx_can.Data[3];
		reply.Data[4] = tx_can[0].Data[2];
		reply.Data[5] = tx_can[0].Data[3];
		reply.Data[6] = tx_can[0].Data[4];
		reply.Data[7] = tx_can[0].Data[5];
		while (CAN_Which_MailboxIsEmpty(CAN1) == 0x0F)
		{
		}
		CAN_Transmit(CAN1, &reply, CAN_Which_MailboxIsEmpty(CAN1));
	}
	else if (rx_can.ExtId == 0x18DA01FA)
	{
		reply.ExtId = 0x18DAFA01;
		reply.Data[0] = 0x07;
		reply.Data[1] = rx_can.Data[1] + 0x40;
		reply.Data[2] = rx_can.Data[2];
		reply.Data[3] = rx_can.Data[3];
		reply.Data[4] = tx_can[0].Data[2];
		reply.Data[5] = tx_can[0].Data[3];
		reply.Data[6] = tx_can[0].Data[4];
		reply.Data[7] = tx_can[0].Data[5];
		while (CAN_Which_MailboxIsEmpty(CAN1) == 0x0F)
		{
		}
		CAN_Transmit(CAN1, &reply, CAN_Which_MailboxIsEmpty(CAN1));
	}
	if (rx_can.ExtId == 0xCF11100)
	{
		if (ENG_RUNNING)
			ENG_RUNNING = 0;
		else
			ENG_RUNNING = 1;
	}
	CAN_FIFORelease(CAN1, 0); //release fifo
}
/*
		K-Line Handler
		Top Priority - 1
	*/
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

void USART6_IRQHandler() //
{
	//for rx handler
	b = USART_InHex(USART6);
	USART_IntFlagClear(USART6, USART_IT_RXNE);
	NVIC_ClearPendingIRQ(USART6_IRQn);

	if (b == 0x81 && start_exec == 0)
		start_exec = 1;

	if (start_exec)
	{
		switch (msg_status)
		{
		case 0:
		{
			if ((b & 0xC0) == 0x80 || (b & 0xC0) == 0xC0)
			{
				kline_len = b & 0x3F;		 //length of data
				buff_length = kline_len + 4; //total length of msg
				Kline_RxBuff[kline_cnt++] = b;
				check_calc += b;
				msg_status = 1;
			}
			break;
		}

		case 1:
		{
			if ((buff_length - 1) != kline_cnt)
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
				if (check_calc == check_uart)
					kline_process_flag = 1;
				else
					memset(Kline_RxBuff, 0, sizeof(Kline_RxBuff));
				check_calc = 0;
				check_uart = 0;
			}
			break;
		}
		default:
			break;
		}
		if (kline_process_flag == 1 && flag == 1)
		{
			flag = 2;
			kline_process_flag = 0;
			buff_length = 0;
			kline_len = 0;
			KLINE_SEND_CMD(start_comm_response, 8);
			memset(Kline_RxBuff, 0, sizeof(Kline_RxBuff));
		}
		else if (kline_process_flag == 1 && flag == 2)
		{
			kline_process_flag = 0;
			buff_length = 0;
			kline_len = 0;
			if (Kline_RxBuff[4] == 0x24)
			{
				kline_buf[0] = 0xA2;
				kline_buf[1] = 0xF1;
				kline_buf[2] = 0x11;
				kline_buf[3] = Kline_RxBuff[3] + 0x40;
				kline_buf[4] = Kline_RxBuff[4];

				for (int i = 0; i < 32; i++)
				{
					kline_buf[i + 5] = tx_can[0].Data[1] + i;
					//								kline_buf[6]=tx_can[0].Data[0];
				}

				kline_crc = 0;
				for (int i = 0; i < 37; i++)
				{
					kline_crc = kline_crc + kline_buf[i];
				}
				kline_buf[37] = kline_crc;
				KLINE_SEND_CMD(kline_buf, 38);
			}

			else if (Kline_RxBuff[4] == 0x9D)
			{
				kline_buf[0] = 0x92;
				kline_buf[1] = 0xF1;
				kline_buf[2] = 0x11;
				kline_buf[3] = Kline_RxBuff[3] + 0x40;
				kline_buf[4] = Kline_RxBuff[4];

				for (int i = 0; i < 16; i++)
				{
					kline_buf[i + 5] = tx_can[0].Data[1] + i;
					//								kline_buf[6]=tx_can[0].Data[0];
				}

				kline_crc = 0;
				for (int i = 0; i < 21; i++)
				{
					kline_crc = kline_crc + kline_buf[i];
				}
				kline_buf[21] = kline_crc;
				KLINE_SEND_CMD(kline_buf, 22);
			}

			else
			{
				kline_buf[0] = 0x84;
				kline_buf[1] = 0xF1;
				kline_buf[2] = 0x11;
				kline_buf[3] = Kline_RxBuff[3] + 0x40;
				kline_buf[4] = Kline_RxBuff[4];
				kline_buf[5] = tx_can[0].Data[1];
				kline_buf[6] = tx_can[0].Data[0];
				kline_crc = 0;
				for (int i = 0; i < 7; i++)
				{
					kline_crc = kline_crc + kline_buf[i];
				}
				kline_buf[7] = kline_crc;
				kline_buf[8] = '\0';
				KLINE_SEND_CMD(kline_buf, 8);
			}
			memset(Kline_RxBuff, 0, sizeof(Kline_RxBuff));
			kline_crc = 0;
			memset(kline_buf, 0, sizeof(kline_buf));
		}
	}
}
void WiFi_Button_Init()
{
	RCC_GPIOB_CLOCK_ENABLE_CLK();
	GPIO_Config_Mode(GPIOB, GPIO_PIN_12, GPIO_MODE_IN); //WiFi Button
	GPIO_Config_Speed(GPIOB, GPIO_PIN_12, GPIO_LOW_SPEED);
	GPIO_Config_Mode(GPIOB, GPIO_PIN_15, GPIO_MODE_OUT); //LED Connected
	GPIO_Config_OType(GPIOB, GPIO_PIN_15, GPIO_OTYPE_PP);
	GPIO_Config_Speed(GPIOB, GPIO_PIN_15, GPIO_LOW_SPEED);
}
void TIM7_IRQHandler()                 //IRQ FOR TIMER=500ms
{
	tim++;
	if((TIM7->SR & (TIMER_UPDATE_FLAG)) == TIMER_UPDATE_FLAG)
	{
		TIM7->SR = ~(TIMER_INTERRUPT);
	}

}
/*
	Main Loop 
*/
int main()
{



	//Clock Setup
	InitializeClock();
	SystemCoreClockUpdate();

	TIM7_Init();
	TIM7_Base_Start_IT();
	//For delay
	uint32_t x = SysTick_Config(SystemCoreClock / 1000);

	//GPS
	//Initializes communincation with SIM28 at the configured baud rate and sets the USART2 Interrupt
	GPS_Init();
	delay(50);

	// Configure SIM28 to send only GPRMC message
	GPS_PMTKCmd(UPDATE_5HZ); //First command goes corrupted hence this line(known bug)
	GPS_PMTKCmd(GPRMC);		 //send the command  to only receive GPRMC string
	GPS_PMTKCmd(UPDATE_5HZ);

	//Initiate TX_CAN messages
	{
		tx_can[0].ExtId = 0xCF00400;
		tx_can[0].DLC = 8;
		tx_can[0].IDE = CAN_ID_EXT;

		tx_can[1].ExtId = 0xCF00300;
		tx_can[1].DLC = 8;
		tx_can[1].IDE = CAN_ID_EXT;

		tx_can[2].ExtId = 0x18FEF100;
		tx_can[2].DLC = 8;
		tx_can[2].IDE = CAN_ID_EXT;

		tx_can[3].ExtId = 0x18FEF200;
		tx_can[3].DLC = 8;
		tx_can[3].IDE = CAN_ID_EXT;

		tx_can[4].ExtId = 0x18FEF500;
		tx_can[4].DLC = 8;
		tx_can[4].IDE = CAN_ID_EXT;

		tx_can[5].ExtId = 0x18FEF600;
		tx_can[5].DLC = 8;
		tx_can[5].IDE = CAN_ID_EXT;

		tx_can[6].ExtId = 0x18FEEF00;
		tx_can[6].DLC = 8;
		tx_can[6].IDE = CAN_ID_EXT;

		tx_can[7].ExtId = 0x18FEEE00;
		tx_can[7].DLC = 8;
		tx_can[7].IDE = CAN_ID_EXT;

		tx_can[8].ExtId = 0x18FF0001;
		tx_can[8].DLC = 8;
		tx_can[8].IDE = CAN_ID_EXT;

		tx_can[9].ExtId = 0x18F0010B;
		tx_can[9].DLC = 8;
		tx_can[9].IDE = CAN_ID_EXT;

		tx_can[10].ExtId = 0x18FEDF00;
		tx_can[10].DLC = 8;
		tx_can[10].IDE = CAN_ID_EXT;

		tx_can[11].ExtId = 0x18FECA00;
		tx_can[11].DLC = 8;
		tx_can[11].IDE = CAN_ID_EXT;

		tx_can[12].ExtId = 0x18FEE000;
		tx_can[12].DLC = 8;
		tx_can[12].IDE = CAN_ID_EXT;

		tx_can[13].ExtId = 0x18FECA0B;
		tx_can[13].DLC = 8;
		tx_can[13].IDE = CAN_ID_EXT;

		tx_can[14].ExtId = 0x18FEBF0B;
		tx_can[14].DLC = 8;
		tx_can[14].IDE = CAN_ID_EXT;

		reply.IDE = CAN_ID_EXT;
		reply.DLC = 8;
		reply.ExtId = 0x18DAFA00;
	}

	//CAN
	Init_CAN();
	setup_can();

	//Init as USART6
	param6.USART_BaudRate = 10400;
	param6.USART_WordLength = USART_WORDLENGTH_8B;
	param6.USART_HardwareFlowControl = USART_HARDWAREFLOWCONTROL_NONE;
	param6.USART_Mode = USART_MODE_Rx_Tx;
	param6.USART_StopBits = USART_STOPBITS_1;
	param6.USART_Parity = USART_PARITY_NO;
	USART6_Init(&param6);
	USART_Enable(USART6);
	flag = 1;
	//Init IRQ for USART6
	USART_IntMaskEnable(USART6, USART_IT_RXNE);
	NVIC_SetPriority(USART6_IRQn, 1);
	NVIC_ClearPendingIRQ(USART6_IRQn); //clear any pending interrupt
	NVIC_EnableIRQ(USART6_IRQn);	   //enable from NVIC

	WiFi_Button_Init();
    TIMEOUT = 5400;
    ENG_RUNNING = 1;
	while (1)
	{

		delay(2);


		while (CAN_Which_MailboxIsEmpty(CAN1) != 0x0F)
		{
			if(ENG_RUNNING && !do_once)
			{
				do_once = 1;
				GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 1);
			}
			if(!ENG_RUNNING && do_once)
			{
				do_once = 0;
				GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 0);
			}

			// if((GPIO_Read_Bit (GPIOB, GPIO_PIN_12))== 0)
			// {
			// 	if(ENG_RUNNING)
			// 	{
			// 		ENG_RUNNING=0;
			// 		GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 0);
			// 	}
			// 	else
			// 	{
			// 		ENG_RUNNING=1;
			// 		GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 1);
			// 	}
			// 	delay(100);
			// }
			if (tim < TIMEOUT && ENG_RUNNING)
			{
				if (i == 0)
				{
					if (ENG_RUNNING)
					{
						s[0] = z.time[4];
						s[1] = z.time[5];
						s[2] = '\0';
						k = atoi(s);
						for (j = 0; j < 8; j++)
						{
							tx_can[i].Data[j] = k + j;
						}
					}
					else
					{
						for (j = 0; j < 8; j++)
						{
							tx_can[i].Data[j] = 0;
						}
					}
					
				}
				else
				{
					s[0] = z.time[4];
					s[1] = z.time[5];
					s[2] = '\0';
					k = atoi(s);
					for (j = 0; j < 8; j++)
					{
						tx_can[i].Data[j] = k + j + i;
					}
				}
				CAN_Transmit(CAN1, &tx_can[i], CAN_Which_MailboxIsEmpty(CAN1));
				i++;
				if (i > 14)
				{
					i = 0;
				}
			}
			else if(tim >= TIMEOUT && ENG_RUNNING )
			{
				if(TIMEOUT == 5400)
				{
					tim = 0;
					TIMEOUT = 2700;
					ENG_RUNNING = 0;

				}
				else if(TIMEOUT == 2700)
				{
					tim = 0;
					TIMEOUT = 300;
					ENG_RUNNING = 0;

				}
				

			}
			else if(tim > TIMEOUT && !ENG_RUNNING)
			{
				if(TIMEOUT == 2700)
				{
					tim = 0;
					TIMEOUT = 2700;
					ENG_RUNNING = 1;

				}
				else if(TIMEOUT == 300)
				{
					tim = 0;
					TIMEOUT = 5400;
					ENG_RUNNING = 1;

				}
			}
		}
	}
}
