#ifndef _RCC_CLOCK_H
#define _RCC_CLOCK_H

/*board specific header file (stm32f446re nucleo) */
#include "stm32f446xx.h"
#include <stdint.h>


//for External oscillator
void InitializeClock(void);

//for internal oscilltaor
void InitializeClockHSI(void);



#endif

/* end */

