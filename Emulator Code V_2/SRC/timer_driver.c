#include "timer_driver.h"

TIM_Base_InitTypeDef TIM6_param_kline;

void TIM6_Base_Init(TIM_Base_InitTypeDef* TIMx)
{
	uint32_t tmpcr1 = 0U;
	tmpcr1 = TIM6->CR1;
	
  TIM6->CR1 = tmpcr1;
	

  /* Set the Auto-reload value */
  TIM6->ARR = (uint32_t)TIMx->Period ;
 
  /* Set the Prescaler value */
  TIM6->PSC = (uint32_t)TIMx->Prescaler;
  
	return;
}

void TIM7_Base_Init(TIM_Base_InitTypeDef* TIMx)
{
	uint32_t tmpcr1 = 0U;
	tmpcr1 = TIM7->CR1;
	
  TIM7->CR1 = tmpcr1;
	

  /* Set the Auto-reload value */
  TIM7->ARR = (uint32_t)TIMx->Period ;
 
  /* Set the Prescaler value */
  TIM7->PSC = (uint32_t)TIMx->Prescaler;
  
	return;
}


void TIM6_Base_Start_IT(void)
{
	uint32_t tmpcr1 = 0U;
	TIM6->DIER |= TIMER_INTERRUPT;
	TIM6->CNT = tmpcr1;
	
	TIM6->CR1|= TIM_CR1_CEN;
	
}


void TIM7_Base_Start_IT(void)
{
	uint32_t tmpcr1 = 0U;
	TIM7->DIER |= TIMER_INTERRUPT;
	TIM7->CNT = tmpcr1;

	TIM7->CR1|= TIM_CR1_CEN;
	
}


