#ifndef _K_LINE_H
#define _K_LINE_H

#include "stm32f446xx.h"

void k_line_fast_init(void);
void k_line_request(char *CMD);

/*Set Command for K-LINE  */

extern uint8_t START_COMM_REQ[];
extern uint8_t tester_present[];
extern uint8_t START_COMM_REQ_1[];
extern uint8_t POS_RESPONSE[];
//extern uint8_t RPM_REQ[];

void KLINE_SEND_CMD(uint8_t* s, uint16_t size_msg);
void KLINE_INIT_SEQ(void);

#endif

/*end of _K_LINE_H*/




