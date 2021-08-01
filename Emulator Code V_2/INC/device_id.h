#ifndef DEVICE_ID_H
#define DEVICE_ID_H

#include "stm32f446xx.h"


#define TM_ID_GetUnique32(x)	((x >= 0 && x < 3) ? (*(uint32_t *) (UID_BASE + 4 * (x))) : 0)

#define TM_ID_GetSignature()	((*(uint16_t *) (ID_DBGMCU_IDCODE)) & 0x0FFF) 

int get_device_id(void);

#endif