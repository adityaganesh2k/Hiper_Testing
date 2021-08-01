#include "delay.h"
#include <stdint.h>

volatile uint32_t mstick=0;

void SysTick_Handler(void)
{
mstick++;
}

/*returns current Tick count*/
uint32_t GetSysTick(void)      
{
return mstick;
}

/*   delay in ms                     */
void delay(uint32_t ms)
{
uint32_t start,end;
start =GetSysTick();
end = start+ms;	
	

if (start < end) { 
  	while ((GetSysTick() >= start) && (GetSysTick() < end)) 
    { } 
                 }

else { 
    while ((GetSysTick() >= start) || (GetSysTick() < end)) {};    //for interger value overflow conditions
     }                                                          //e.g. if start=Max(uint32_t), then end=start+ms <start (more condtions.) 

}


	

