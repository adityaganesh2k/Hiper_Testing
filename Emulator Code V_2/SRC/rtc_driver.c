#include "delay.h"
#include "rtc_driver.h"


RTCError err;

#define RTC_TR_RESERVED_MASK    ((uint32_t)0x003F7F7F)
#define RTC_DR_RESERVED_MASK    ((uint32_t)0x00FFFF3F) 
#define INITMODE_TIMEOUT         ((uint32_t) 0x00010000)
#define SYNCHRO_TIMEOUT          ((uint32_t) 0x00020000)

////////////////////////////////////
//////////Helper Functions//////////

uint8_t BYTEToBCD(uint8_t Value)
{
  uint8_t bcdhigh = 0;
  
  while (Value >= 10)
  {
    bcdhigh++;
    Value -= 10;
  }
  
  return  ((uint8_t)(bcdhigh << 4) | Value);
}

uint8_t BCDToBYTE(uint8_t Value)
{
  uint8_t tmp = 0;
  tmp = (uint8_t)(Value & 0xF) + (((uint8_t)Value & 0xF0) >> 4)*10;             //((uint8_t)(Value & (uint8_t)0xF0) >> (uint8_t)0x4) * 10;
	return tmp;
//  return (tmp + (Value & (uint8_t)0x0F));
}

uint8_t ASCII_TO_BCD(unsigned char ascii_text[2])
{
        uint8_t bcd_value;
        ascii_text[0] &= 0x0F;     // set high nibble
        ascii_text[0] <<= 4;       // shift lower nibble to higher

        ascii_text[1] &= 0x0F;     // set high nibble
        bcd_value = ascii_text[0] | ascii_text[1];  // mix both to get BCD
        return bcd_value;
}

void BCD_TO_ASCII(unsigned char time, unsigned char ascii_val[])
{
	
	
	

	ascii_val[0]=time&0xF0;
  ascii_val[0]=(ascii_val[0]>>4)|0x30;
	ascii_val[1]=time&0x0F;
  ascii_val[1]=ascii_val[1]|0x30;
	
} 

/* */




//Enters the RTC Initialization mode.

//SUCCESS: RTC is in Init mode
//ERROR: RTC is not in Init mode  
RTCError RTC_EnterInitMode(void)
{
  __IO uint32_t initcounter = 0x00;
  RTCError status = ERROR;
  uint32_t initstatus = 0x00;
 
	
  /* Check if the Initialization mode is set */
  if ((RTC->ISR & RTC_ISR_INITF) == (uint32_t)RESET)
  {
		/*clear the RSF flag */
		//it is also cleared using below line
		
    /* Set the Initialization mode */
    RTC->ISR = (uint32_t) RTC_ISR_INIT;
    
    /* Wait till RTC is in INIT state and if Time out is reached exit */
    do
    {
      initstatus = RTC->ISR & RTC_ISR_INITF;
      initcounter++;  
    } while((initcounter != INITMODE_TIMEOUT) && (initstatus == 0x00));
    
    if ((RTC->ISR & RTC_ISR_INITF) != RESET)
    {
      status = SUCCESS;
    }
    else
    {
      status = ERROR;
    }        
  }
  else
  {
    status = SUCCESS;  
  } 
    
  return (status);  
}



//Exits the RTC Initialization mode.
  //When the initialization sequence is complete, the calendar restarts 
  //counting after 4 RTCCLK cycles.  
 
void RTC_ExitInitMode(void)
{ 
  /* Exit Initialization mode */
  RTC->ISR &= (uint32_t)~RTC_ISR_INIT;  
}



////////////////////////////////////////////////
////////////////////////////////////////////////





RTCError RTC_Init(RTC_TimeTypeDef* RTC_Time,RTC_DateTypeDef* RTC_Date)
{
 
	RTCError status = ERROR;
	
	
	//Enable RTC Register write Access
	PWR->CR |=PWR_CR_DBP;
	
  
	
	
	/* Disable the write protection for RTC registers */
  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
	
	
	/* Set Initialization mode */
  if (RTC_EnterInitMode() == ERROR)
  {
    status = ERROR;
  } 
  else
  {

    /* Configure the RTC PRER */
    RTC->PRER = 7999;                //for HSE.............
    RTC->PRER |= (uint32_t)(124 << 16);
		
		////////////////////////////////////////////////////////////////////////////
		///set time and date
		uint32_t tmpreg = 0;
		tmpreg = (uint32_t)(((uint32_t)BYTEToBCD(RTC_Time->RTC_Hours) << 16) | \
                   ((uint32_t)BYTEToBCD(RTC_Time->RTC_Minutes) << 8) | \
                   ((uint32_t)BYTEToBCD(RTC_Time->RTC_Seconds))) ;
		
		
		/* Set the RTC_TR register */
    RTC->TR = (uint32_t)(tmpreg & RTC_TR_RESERVED_MASK);
		
		//set date
		tmpreg = (((uint32_t)BYTEToBCD(RTC_Date->RTC_Year) << 16) | \
              ((uint32_t)BYTEToBCD(RTC_Date->RTC_Month) << 8) | \
              ((uint32_t)BYTEToBCD(RTC_Date->RTC_Date)) | \
              ((uint32_t)RTC_Date->RTC_WeekDay << 13));
							
			
     /* Set the RTC_DR register */
    RTC->DR = (uint32_t)(tmpreg & RTC_DR_RESERVED_MASK);			
    //////////////////////////////////////////////////////////////////////////////////
    
		//Exit Initialization mode 
    RTC_ExitInitMode();

    status = SUCCESS;    
  }
	
	//Waits until the RTC Time and Date registers (RTC_TR and RTC_DR) are 
  //synchronized with RTC APB clock.
	if(status==SUCCESS)
	{
	   __IO uint32_t synchrocounter = 0;
	 	 uint32_t synchrostatus = 0x00;
	
         /* Wait the registers to be synchronised */
       do
         {
          synchrostatus = RTC->ISR & RTC_ISR_RSF;
          synchrocounter++;  
          } while((synchrocounter != SYNCHRO_TIMEOUT) && (synchrostatus == 0x00));
    
       if ((RTC->ISR & RTC_ISR_RSF) != RESET)
       {
         status = SUCCESS;
       }
       else
       {
         status = ERROR;
       }        
	
	}
	//Disable RTC write access 
	PWR->CR &=~PWR_CR_DBP;
  /* Enable the write protection for RTC registers */
  RTC->WPR = 0xFF; 
  

  return status;

}
RTCError RTC_Auto_Wakeup_Unit_Init(uint16_t time_gap)
{
	RTCError status = ERROR;

	//Enable RTC Register write Access
	PWR->CR |=PWR_CR_DBP;

	/* Disable the write protection for RTC registers */
	RTC->WPR = 0xCA;
 	RTC->WPR = 0x53;
 	//Disable timer
 	RTC->CR &= ~RTC_CR_WUTE;
	/* Set Initialization mode */
  if (RTC_AWU_EnterInitMode() == ERROR)
  {
    status = ERROR;
  }
  else
  {
	  RTC->WUTR = time_gap;   //range for time_gap - 1s to 65536s

	  //WUCKSEL[2:0] to be in 10x configuration for the above range
	  RTC->CR &= ~RTC_CR_WUCKSEL_1;
	  RTC->CR |= RTC_CR_WUCKSEL_2;

	  EXTI->IMR |= EXTI_IMR_MR22;
	  EXTI->RTSR |= EXTI_RTSR_TR22;
//	  EXTI->PR |= EXTI_PR_PR22;

	  RTC->ISR &= ~RTC_ISR_WUTF;  //check if needed
	  //re enable timer
	  RTC->CR |= RTC_CR_WUTE;
	  //Enable periodic wakeup interrupt
	  RTC->CR |= RTC_CR_WUTIE;
	  //Disable RTC write access
	  PWR->CR &=~PWR_CR_DBP;
	  /* Enable the write protection for RTC registers */
	  RTC->WPR = 0xFF;

	  status = SUCCESS;

  }
  return status;

}

_Bool read_rtc_status()
{
	return ((RTC->CR & RTC_CR_WUTE) >> 10U);
}
RTCError RTC_Auto_Wakeup_Unit_Reset()
{
	RTCError status = ERROR;

	//Enable RTC Register write Access
	PWR->CR |=PWR_CR_DBP;

	/* Disable the write protection for RTC registers */
	RTC->WPR = 0xCA;
 	RTC->WPR = 0x53;
 	//Disable timer
 	RTC->CR &= ~RTC_CR_WUTE;
	/* Set Initialization mode */
  if (RTC_AWU_EnterInitMode() == ERROR)
  {
    status = ERROR;
  }
  else
  {


	  RTC->ISR &= ~RTC_ISR_WUTF;  //check if needed
	  //re enable timer
	  RTC->CR |= RTC_CR_WUTE;
//	  //Enable periodic wakeup interrupt
//	  RTC->CR |= RTC_CR_WUTIE;
	  //Disable RTC write access
	  PWR->CR &=~PWR_CR_DBP;
	  /* Enable the write protection for RTC registers */
	  RTC->WPR = 0xFF;

	  status = SUCCESS;

  }
  return status;

}

RTCError RTC_AWU_EnterInitMode(void)
{
  __IO uint32_t initcounter = 0x00;
  RTCError status = ERROR;
  uint32_t initstatus = 0x00;


  /* Check if the Initialization mode is set */
  if ((RTC->ISR & RTC_ISR_WUTWF) == (uint32_t)RESET)
  {

    /* Wait till RTC_ISR_WUTWF is set and if Time out is reached exit */
    do
    {
      initstatus = RTC->ISR & RTC_ISR_WUTWF;
      initcounter++;
    } while((initcounter != INITMODE_TIMEOUT) && (initstatus == 0x00));

    if ((RTC->ISR & RTC_ISR_WUTWF) != RESET)
    {
      status = SUCCESS;
    }
    else
    {
      status = ERROR;
    }
  }
  else
  {
    status = SUCCESS;
  }

  return (status);
}

void RTC_GetTimestamp_HMS(RTC_Timestamp* RTC_HMS)
{
	RTC_HMS->timestamp_HMS = (uint32_t)(RTC->TR & RTC_TR_RESERVED_MASK);
  (void) (RTC->DR);
	return;
}

void RTC_GetTimestamp_SS(RTC_Timestamp* RTC_SS)
{
	RTC_SS->timestamp_SS = (uint32_t)(RTC->SSR);
	return;
}

void RTC_GetTime(RTC_TimeTypeDef* RTC_Time)
{
 uint32_t tmpreg = 0;
	
 tmpreg = (uint32_t)(RTC->SSR);


  /* Get the RTC_TR register */
  tmpreg = (uint32_t)(RTC->TR & RTC_TR_RESERVED_MASK); 
  
  /* Fill the structure fields with the read parameters */
  RTC_Time->RTC_Hours = (uint8_t)((tmpreg & (RTC_TR_HT | RTC_TR_HU)) >> 16);
  RTC_Time->RTC_Minutes = (uint8_t)((tmpreg & (RTC_TR_MNT | RTC_TR_MNU)) >>8);
  RTC_Time->RTC_Seconds = (uint8_t)(tmpreg & (RTC_TR_ST | RTC_TR_SU));
	
	/* Convert the structure parameters to Binary format */
    RTC_Time->RTC_Hours = (uint8_t)BCDToBYTE(RTC_Time->RTC_Hours);
    RTC_Time->RTC_Minutes = (uint8_t)BCDToBYTE(RTC_Time->RTC_Minutes);
    RTC_Time->RTC_Seconds = (uint8_t)BCDToBYTE(RTC_Time->RTC_Seconds); 
	
	/* Get sub seconds values from the correspondent registers*/
  tmpreg = (uint32_t)(RTC->SSR);
  
  /* Read DR register to unfroze calendar registers */
  (void) (RTC->DR);
	
	RTC_Time->RTC_SubSec=tmpreg;

}	

	
void RTC_GetDate(RTC_DateTypeDef* RTC_Date)
{

uint32_t tmpreg = 0;
  
  /* Get the RTC_TR register */
  tmpreg = (uint32_t)(RTC->DR & RTC_DR_RESERVED_MASK); 

  /* Fill the structure fields with the read parameters */
  RTC_Date->RTC_Year = (uint8_t)((tmpreg & (RTC_DR_YT | RTC_DR_YU)) >> 16);
  RTC_Date->RTC_Month = (uint8_t)((tmpreg & (RTC_DR_MT | RTC_DR_MU)) >> 8);
  RTC_Date->RTC_Date = (uint8_t)(tmpreg & (RTC_DR_DT | RTC_DR_DU));
  RTC_Date->RTC_WeekDay = (uint8_t)((tmpreg & (RTC_DR_WDU)) >> 13);

  /* Check the input parameters format */
  
  
    /* Convert the structure parameters to Binary format */
    RTC_Date->RTC_Year = (uint8_t)BCDToBYTE(RTC_Date->RTC_Year);
    RTC_Date->RTC_Month = (uint8_t)BCDToBYTE(RTC_Date->RTC_Month);
    RTC_Date->RTC_Date = (uint8_t)BCDToBYTE(RTC_Date->RTC_Date);
  
}	

 
