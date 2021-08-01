#ifndef _GPS_DRIVER_H
#define _GPS_DRIVER_H

#include "uart_driver.h"
#include "gpio_driver.h"

#define NUMBER   0x30
#define ALPHABET 0x40

#define CONVERT   0x0F
#define ALPH_OFFSET   0x09

extern char GPS_RxBuff[80];
extern volatile int gps_log_flag;
extern volatile int altitude_flag;
extern volatile int valid_bit;
extern volatile int star_flag;
extern volatile int msg_process_flag;
extern volatile char GPS_RxFlag;
extern volatile uint8_t gprmc_check_success;
extern volatile uint8_t freq_check_success;
extern uint32_t tracker_brd;
extern uint8_t broadcast_params;
/*Enable or disable the Output sentences */
extern char  *GPRMC;
extern char *GPRMCGGA;
/*Baud rate setting of serial port of the GPS*/	
extern char *SET_BAUDRATE_9600;
_Bool alt_toggle;
	
/*Restart the GPS (Factory Reset) */		
extern char *FULLCOLD_RESTART;

/* Go to standby mode to save power , (Software on Host side sends any byte to wake up from standby mode.) */	
extern char *STANDBYMODE;

extern char *UPDATE_1HZ;
extern char *UPDATE_2HZ;
extern char *UPDATE_5HZ;

typedef struct
{

char time[6];
char date[6];
char lat[9]; 
char longi[10];
char altitude1[6];
char altitude2[6];
}GPS_Data;



uint8_t ascii_to_hex(uint8_t ch);
void    GPS_Init(void);            //Enables and Initializes every thing related to GPS
void    GPS_PMTKCmd(char *s);
 
void    GPS_NMEAString();
void 	GPS_NMEAData(GPS_Data *data);
void	FREQUENCY_CHECK();

#endif
