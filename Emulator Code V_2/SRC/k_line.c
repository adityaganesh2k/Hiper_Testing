#include "k_line.h"
#include "gpio_driver.h"
#include "uart_driver.h"
#include "timer_driver.h"
#include <stdint.h>

uint8_t START_COMM_REQ[]={0x81, 0x10, 0xF1, 0x81, 0x03, 0x0 };
uint8_t START_COMM_REQ_1[]={0x81, 0x10, 0xF1, 0x81, 0x03, 0x0 };
uint8_t	POS_RESPONSE[] = { 0x83, 0xF1, 0x10, 0xC1, 0xDF, 0x8F, 0xB3 };

uint8_t tester_present[]={0xC2, 0x33, 0xF1, 0x3E, 0x01, 0x25};
uint8_t SUPPORTED_PIDS_20[]={0xC2, 0x33, 0xF1, 0x01, 0x0C, 0xF3};
uint8_t COM_START_RESP[]={0x83, 0xF1, 0x11, 0xC1};


void KLINE_SEND_CMD(uint8_t* kl, uint16_t size_msg)
{
	NVIC_DisableIRQ(USART6_IRQn);
	USART_DisableRx(USART6);
  int i=0;
	while(size_msg!=0)          //Send the entire cmd string
	{	
		USART_OutHex(USART6,kl[i]);
		i++;
		size_msg--;
	}
	NVIC_EnableIRQ(USART6_IRQn);
	USART_IntFlagClear(USART6,USART_IT_RXNE);
	NVIC_ClearPendingIRQ(USART6_IRQn);
	USART_EnableRx(USART6);
}
