#include "esp.h"
#include "uart_driver.h"
#include "gpio_driver.h"
#include "delay.h"

char  *FILE_TRANSFER="$0A\n";
char  *FILETXDF="$0D\n";
char  *LAST_FILE_SIZE="$A1\n";
char  *LAST_FILE="$BB\n";
char  *LAST_DATED_FILE="$BD\n";
char  *UPLDFXS="$UX\n";

char *CHECK_UPDATES="$MZ\n";

char  *KEEP_IGN_ON = "$M1\n";
char  *TURN_VEHCILE_ON = "$M2\n";
char  *TURN_VEHCILE_OFF = "$M3\n";
char  *FLASHING_COMPLETE = "$M4\n";
char  *PRESS_BUTTON = "$M5\n";
char  *ECU_UPDATE_AVAILABLE = "$M6\n";
char  *Backup_Audio = "$M7\n";

char  *DOWN0="$D0\n";
char  *DOWN1="$D1\n";
char  *DOWN2="$D2\n";
char  *DOWN3="$D3\n";
char  *DOWN4="$D4\n";
char  *DOWN5="$D5\n";
char  *DOWN6="$D6\n";
char  *DOWN7="$D7\n";

char  *FILE0="$F0\n";
char  *FILE1="$F1\n";
char  *FILE2="$F2\n";
char  *FILE3="$F3\n";
char  *FILE4="$F4\n";
char  *FILE5="$F5\n";
char  *FILE6="$F6\n";
char  *FILE7="$F7\n";

char  *PRESS_CLUTCH ="$M8\n";
char  *RELEASE_CLUTCH ="$M9\n";
char  *PRESS_BRAKE ="$MA\n";
char  *RELEASE_BRAKE ="$MB\n";
char  *FIRST_GEAR ="$MC\n";
char  *NEUTRAL ="$MD\n";
char  *ENG_ON ="$ME\n";
char  *PRESS_THROTTLE ="$MF\n";
char  *RELEASE_THROTTLE ="$MG\n";
char  *ENG_OFF ="$MH\n";
char  *BRD_DONE = "$MI\n";
char  *REQ_DONE = "$MJ\n";

char  *WIFI_ON="$AA\n";

char  *FIRSTb="zzz\n";
USART_Type param3;


void ESP_CMD_SEND(char *s)
{

	int i=0;
	while(s[i]!='\n')          //Send the entire cmd string
	{	
		USART_OutChar(USART3,s[i]);
		i++;
	}

}

void ESP_Init()
	{
		RCC_GPIOA_CLOCK_ENABLE_CLK();
		RCC_GPIOB_CLOCK_ENABLE_CLK();
		RCC_GPIOC_CLOCK_ENABLE_CLK();

		GPIO_Config_Mode (GPIOB,GPIO_PIN_15,GPIO_MODE_OUT);        //LED Connected
		GPIO_Config_OType (GPIOB, GPIO_PIN_15, GPIO_OTYPE_PP); 
		GPIO_Config_Speed (GPIOB, GPIO_PIN_15, GPIO_LOW_SPEED);
		GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 0);
		delay(1000);
		GPIO_Write_Bit(GPIOB,GPIO_PIN_15, 1);
		
		GPIO_Config_Mode (GPIOA,GPIO_PIN_6,GPIO_MODE_OUT);        //Amp Connected
		GPIO_Config_OType (GPIOA, GPIO_PIN_6, GPIO_OTYPE_PP); 
		GPIO_Config_Speed (GPIOA, GPIO_PIN_6, GPIO_LOW_SPEED);
		GPIO_Write_Bit(GPIOA,GPIO_PIN_6, 0);

		param3.USART_BaudRate=115200;
		param3.USART_WordLength=USART_WORDLENGTH_8B;
		param3.USART_HardwareFlowControl=USART_HARDWAREFLOWCONTROL_NONE;
		param3.USART_Mode=USART_MODE_Rx_Tx;
		param3.USART_StopBits=USART_STOPBITS_1;
		param3.USART_Parity=USART_PARITY_NO;
		USART3_Init(&param3);

		USART_Enable(USART3);
		USART_IntFlagClear(USART3,USART_IT_RXNE);
		ESP_CMD_SEND(FIRSTb);
		delay(10);
			
		USART_IntMaskEnable(USART3,USART_IT_RXNE);
		NVIC_SetPriority(USART3_IRQn,7);	
		NVIC_ClearPendingIRQ(USART3_IRQn);             //clear any pending interrupt
		NVIC_EnableIRQ(USART3_IRQn); 

//USART_IntFlagClear(USART3,USART_IT_RXNE);
//		ESP_CMD_SEND(UPLDFXS);
		delay(10);
	}
