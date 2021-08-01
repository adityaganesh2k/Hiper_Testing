#ifndef _RTC_DRIVER_H
#define _RTC_DRIVER_H


/*board specific header file (stm32f446re nucleo) */
#include "stm32f446xx.h"
#include <stdint.h>


/* typedef enum
{
ENABLE=0,
DISABLE=1	
}
RTCFunctionalState;  */


typedef enum
{

SUCCESS=0,
ERROR =1	
}	RTCError;


typedef enum
{
RESET=0,
SET=1	
}	RTCActiveState;
	
typedef struct
{
	uint32_t timestamp_HMS;
	
	uint32_t timestamp_SS;
	
}RTC_Timestamp;


//RTC Time structure definition  

typedef struct
{
  uint8_t RTC_Hours;    //Specifies the RTC Time Hour.
                        
  uint8_t RTC_Minutes;  //Specifies the RTC Time Minutes.
                        //This parameter must be set to a value in the 0-59 range. 
  
  uint8_t RTC_Seconds;  //Specifies the RTC Time Seconds.
                        //This parameter must be set to a value in the 0-59 range. 
	
	uint32_t RTC_SubSec;  //sub seconds

 
}RTC_TimeTypeDef; 



//RTC Date structure definition  

typedef struct
{
  uint8_t RTC_WeekDay;  //Specifies the RTC Date WeekDay.
                        
  
  uint8_t RTC_Month;   //RTC Date Month in BCD format
                       

  uint8_t RTC_Date;     //RTC Date.
                        
  
  uint8_t RTC_Year;     //RTC Date Year.
                        
}RTC_DateTypeDef;





//RTC time format
#define RTC_HourFormat_24              ((uint32_t)0x00000000)
#define RTC_HourFormat_12              ((uint32_t)0x00000040)


//Months of year
//Coded in BCD format 
#define RTC_Month_January              ((uint8_t)0x01)
#define RTC_Month_February             ((uint8_t)0x02)
#define RTC_Month_March                ((uint8_t)0x03)
#define RTC_Month_April                ((uint8_t)0x04)
#define RTC_Month_May                  ((uint8_t)0x05)
#define RTC_Month_June                 ((uint8_t)0x06)
#define RTC_Month_July                 ((uint8_t)0x07)
#define RTC_Month_August               ((uint8_t)0x08)
#define RTC_Month_September            ((uint8_t)0x09)
#define RTC_Month_October              ((uint8_t)0x10)
#define RTC_Month_November             ((uint8_t)0x11)
#define RTC_Month_December             ((uint8_t)0x12)


//WeekDay_Definitions 

#define RTC_Weekday_Monday             ((uint8_t)0x01)
#define RTC_Weekday_Tuesday            ((uint8_t)0x02)
#define RTC_Weekday_Wednesday          ((uint8_t)0x03)
#define RTC_Weekday_Thursday           ((uint8_t)0x04)
#define RTC_Weekday_Friday             ((uint8_t)0x05)
#define RTC_Weekday_Saturday           ((uint8_t)0x06)
#define RTC_Weekday_Sunday             ((uint8_t)0x07)



//RTC_Flag

#define RTC_FLAG_INITF                    ((uint32_t)0x00000040)
#define RTC_FLAG_RSF                      ((uint32_t)0x00000020)
#define RTC_FLAG_INITS                    ((uint32_t)0x00000010)


RTCError RTC_Init(RTC_TimeTypeDef* RTC_Time,RTC_DateTypeDef* RTC_Date);
RTCError RTC_Auto_Wakeup_Unit_Init(uint16_t time_gap);
RTCError RTC_Auto_Wakeup_Unit_Reset();
_Bool read_rtc_status();
void RTC_GetTime(RTC_TimeTypeDef* RTC_TimeStruct);
void RTC_GetDate(RTC_DateTypeDef* RTC_DateStruct);
RTCError RTC_EnterInitMode(void);
RTCError RTC_AWU_EnterInitMode(void);
void RTC_GetTimestamp_HMS(RTC_Timestamp* RTC_HMS);
void RTC_GetTimestamp_SS(RTC_Timestamp* RTC_SS);

uint8_t BYTEToBCD(uint8_t Value);
void BCD_TO_ASCII(unsigned char time, unsigned char ascii_val[]);
uint8_t ASCII_TO_BCD(unsigned char ascii_text[2]);
uint32_t ASCII_TO_BCD_ss(unsigned char ascii_text[3]);


#endif
