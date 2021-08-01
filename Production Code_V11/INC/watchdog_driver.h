#ifndef WATCHDOG_TIMER_H
#define WATCHDOG_TIMER_H

#include "stm32f446xx.h"
extern uint8_t IWDG_SET;
typedef enum {
	TM_WATCHDOG_Timeout_5ms = 0x00,   /*!< System reset called every 5ms */
	TM_WATCHDOG_Timeout_10ms = 0x01,  /*!< System reset called every 10ms */
	TM_WATCHDOG_Timeout_15ms = 0x02,  /*!< System reset called every 15ms */
	TM_WATCHDOG_Timeout_30ms = 0x03,  /*!< System reset called every 30ms */
	TM_WATCHDOG_Timeout_60ms = 0x04,  /*!< System reset called every 60ms */
	TM_WATCHDOG_Timeout_120ms = 0x05, /*!< System reset called every 120ms */
	TM_WATCHDOG_Timeout_250ms = 0x06, /*!< System reset called every 250ms */
	TM_WATCHDOG_Timeout_500ms = 0x07, /*!< System reset called every 500ms */
	TM_WATCHDOG_Timeout_1s = 0x08,    /*!< System reset called every 1s */
	TM_WATCHDOG_Timeout_2s = 0x09,    /*!< System reset called every 2s */
	TM_WATCHDOG_Timeout_4s = 0x0A,    /*!< System reset called every 4s */
	TM_WATCHDOG_Timeout_8s = 0x0B,    /*!< System reset called every 8s */
	TM_WATCHDOG_Timeout_16s = 0x0C,   /*!< System reset called every 16s */
	TM_WATCHDOG_Timeout_32s = 0x0D    /*!< System reset called every 32s. This is maximum value allowed with IWDG timer */
} TM_WATCHDOG_Timeout_t;

uint8_t TM_WATCHDOG_Init(TM_WATCHDOG_Timeout_t timeout);

#define TM_WATCHDOG_Reset()     (IWDG->KR = 0xAAAA)

#endif WATCHDOG_TIMER_H