#include "rcc_clock.h"



void RTC_ClocksPowerInit(void)
{
  RCC->APB1ENR |=RCC_APB1ENR_PWREN;
	RCC->AHB1ENR |=RCC_AHB1ENR_BKPSRAMEN;              
	
	PWR->CR |=PWR_CR_DBP;
	
	uint32_t tempreg=0;
	tempreg |=RCC_BDCR_RTCEN;
	//select HSE for RTC
	tempreg |=RCC_BDCR_RTCSEL;
//	//Enable RTC Clock

	RCC->BDCR|=tempreg;

	PWR->CR &=~PWR_CR_DBP;
	

}	


void InitializeClock(void)
{

  RCC->CFGR = 0x00000000;         //Reset Clock Configuration Register
  RCC->CR &= 0xFEF6FFFF;          //Reset HSEON, CSSON and PLLON Bits
  RCC->CR |= RCC_CR_HSEON;        //Turn on HSE clock
  
	//Wait until HSE is ready
	while((RCC->CR & RCC_CR_HSERDY) == 0)
  {}	

	  //SET the APB1 and APB2 (AHB/2) and AHB (SYSCLK/1)	
		uint32_t tempreg=0;
	  //SET the APB1 and APB2 (AHB/2) and AHB (SYSCLK/1)	
		tempreg |=RCC_CFGR_HPRE_DIV1;     // AHB=SYSCLK/1
		tempreg |=RCC_CFGR_PPRE1_DIV2;    // APB1=AHB/2
		tempreg |=RCC_CFGR_PPRE2_DIV1;    // APB2=AHB/1
		tempreg |=(0x00080000);           //for rtc HSE/8
		RCC->CFGR |=tempreg;
		RCC->CFGR |= RCC_CFGR_RTCPRE_0 ;
		RCC->CFGR |= RCC_CFGR_RTCPRE_1 ;
		RCC->CFGR |= RCC_CFGR_RTCPRE_2 ;
		RCC->CFGR |= RCC_CFGR_RTCPRE_3 ;
		RCC->CFGR |= RCC_CFGR_RTCPRE_4 ;
		

		
		
		 int i;
		 for(i=0;i<5;i++) //some delay   //some fault was coming once or twice ..//mainly happens in Clock peripherals (got from internet)
		 {}
		
	  
			 
		//Set PLLP = Div_8 , PLLN = 192, PLLM = 4, PLLQ = Div_8, PLLR=Div_2, PLL Src = HSE
    RCC->PLLCFGR = (uint32_t)0;               //CLEAR 
		RCC->PLLCFGR |= (uint32_t)(0x04 << 0);    //PLLM = 4
		RCC->PLLCFGR |= (uint32_t)(0x03 << 12);   //PLLN=192
		RCC->PLLCFGR |= (uint32_t)(0x03 << 16);   //PLLP = Div_8
		RCC->PLLCFGR |= (uint32_t)(0x01 << 22);   //PLL Src = HSE
		RCC->PLLCFGR |= (uint32_t)(0x08 << 24);   //PLLQ = Div_8
		RCC->PLLCFGR |= (uint32_t)(0x02 << 28);   //PLLR=Div_2
 
		 int j;
		 for(j=0;j<5;j++) //some delay   //some fault was coming once or twice ...
		 {}
		
		//Enable PLL on	 
		RCC->CR |= RCC_CR_PLLON;                            
 
		
		//Wait for PLL to lock on	 
	  while((RCC->CR & RCC_CR_PLLRDY) == 0)
    {}	                                               
	
	
		//select the pll oscillator as clock source
	  RCC->CFGR &= ~RCC_CFGR_SW;    //<<<<<<<<<<<<<,,------------------make change here in previous versions
	  RCC->CFGR |=RCC_CFGR_SW_PLL;
	
		//Set flash wait states to 5
	  FLASH->ACR &= 0xFFFFFFF8;       
	  FLASH->ACR |= 0x5;

		 int k;
		 for(k=0;k<5;k++) //some delay   //some fault was coming once or twice ...
		 {}
			 
		RTC_ClocksPowerInit();	

}


void InitializeClockHSI(void)
{

   RCC->CR |= RCC_CR_HSION;           //Turn on HSI clock
  
	 // Wait until HSI is ready
   while((RCC->CR & RCC_CR_HSIRDY) == 0)
   {}	
   uint32_t tempreg=0;
	  //SET the APB1 and APB2 (AHB/2) and AHB (SYSCLK/1)	
		tempreg |=RCC_CFGR_HPRE_DIV1;     // AHB=SYSCLK/1
		tempreg |=RCC_CFGR_PPRE1_DIV2;    // APB1=AHB/2
		tempreg |=RCC_CFGR_PPRE2_DIV1;    // APB2=AHB/1
		RCC->CFGR |=tempreg;
		 int i;
		 for(i=0;i<5;i++) //some delay   //some fault was coming once or twice ...
		 {}
		
		//Set PLLP = Div_8 , PLLN = 192, PLLM = 8, PLLQ = Div_8, PLLR=Div_2, PLL Src = HSE
    RCC->PLLCFGR = (uint32_t)0x00;            //CLEAR 
		RCC->PLLCFGR |= (uint32_t)(0x08 << 0);    //PLLM = 8
		RCC->PLLCFGR |= (uint32_t)(0x03 << 12);   //PLLN=192
		RCC->PLLCFGR |= (uint32_t)(0x03 << 16);   //PLLP = Div_8
		RCC->PLLCFGR |= (uint32_t)(0x00 << 22);   //PLL Src = HSI
		RCC->PLLCFGR |= (uint32_t)(0x08 << 24);   //PLLQ = Div_8
		RCC->PLLCFGR |= (uint32_t)(0x02 << 28);   //PLLR=Div_2
 
		int j;
		 for(j=0;j<5;j++) //some delay
		 {}
		 
		//Enable PLL on	 
		RCC->CR |= RCC_CR_PLLON;           
 
		//Wait for PLL to lock on	 
	  while((RCC->CR & RCC_CR_PLLRDY) == 0)
    {}	
	
	  //select the pll oscillator as clock source
		RCC->CFGR &=~RCC_CFGR_SW;                         /////////<<<<<<<,-------------make corrections here in all previous versions
	  RCC->CFGR |=RCC_CFGR_SW_PLL;
	
	  //Set flash wait states to 5	
	  FLASH->ACR &= 0xFFFFFFF8;            //saw it somewhere ..will see later
	  FLASH->ACR |= 0x5;
		
	  int k; 	
	  for(k=0;k<5;k++)        //some delay
	  {}	

}
