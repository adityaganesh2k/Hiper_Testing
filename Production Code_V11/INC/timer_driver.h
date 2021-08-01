#ifndef TIMER_DRIVER_H
#define TIMER_DRIVER_H

/*board specific header file (stm32f446re nucleo) */
#include "stm32f446xx.h"
#include <stdint.h>
#include "rcc_clock.h"


#define RCC_TIM6_CLOCK_ENABLE_CLK()   (RCC->APB1ENR |= (1<<4))

#define RCC_TIM7_CLOCK_ENABLE_CLK()   (RCC->APB1ENR |= (1<<5))


typedef struct
{
  uint32_t Prescaler;         /*!< Specifies the prescaler value used to divide the TIM clock.
                                   This parameter can be a number between Min_Data = 0x0000U and Max_Data = 0xFFFFU */

  uint32_t CounterMode;       /*!< Specifies the counter mode.
                                   This parameter can be a value of @ref TIM_Counter_Mode */

  uint32_t Period;            /*!< Specifies the period value to be loaded into the active
                                   Auto-Reload Register at the next update event.
                                   This parameter can be a number between Min_Data = 0x0000U and Max_Data = 0xFFFF.  */

  uint32_t ClockDivision;     /*!< Specifies the clock division.
                                   This parameter can be a value of @ref TIM_ClockDivision */

  uint32_t RepetitionCounter;  /*!< Specifies the repetition counter value. Each time the RCR downcounter
                                    reaches zero, an update event is generated and counting restarts
                                    from the RCR value (N).
                                    This means in PWM mode that (N+1) corresponds to:
                                        - the number of PWM periods in edge-aligned mode
                                        - the number of half PWM period in center-aligned mode
                                     This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF. 
                                     @note This parameter is valid only for TIM1 and TIM8. */
} TIM_Base_InitTypeDef;




void TIM6_Base_Init(TIM_Base_InitTypeDef* TIMx);
void TIM7_Base_Init(TIM_Base_InitTypeDef* TIMx);

void TIM6_Base_Start_IT(void);
void TIM7_Base_Start_IT(void);



#define TIMER_INTERRUPT                 TIM_DIER_UIE
#define TIMER_CNTL_ENABLE               TIM_CR1_CEN
#define TIMER_UPDATE_FLAG               TIM_SR_UIF
#define TIMER_SR_TRIGGER_FLAG           TIM_SR_TIF
#define TIMER_DIER_TRIGGER_INTERRUPT    TIM_DIER_TIE


#endif