#ifndef _DAQ_H_
#define _DAQ_H_

#include "stm32f446xx.h"

//Variables 
	//Struct for CAN 2.0 broadcast param 
		typedef struct
		{	
			uint8_t length;			//Lenght of the data field from start position common for both J1939 & CAN_OBD_PIDs

			uint8_t	log_freq;	/*!< Log_Period specifies the rate at which the parameter is logged 
																00 - Default High Speed 4Hz
																01 - Fluid Temperatures etc 0.2 Hz 
																02 - For every startup */
			
			uint8_t position;		//Position of the start byte of this parameter in the J1939 Message with the relevant PGN
			
			
		} broadcast_param_can;
		
		typedef struct
		{
			uint8_t can_id_no;
			
			uint8_t can_extn_param;

			uint8_t length;			//Lenght of the data field from start position common for both J1939 & CAN_OBD_PIDs

			uint8_t	log_freq;	/*!< Log_Period specifies the rate at which the parameter is logged 
																00 - Default High Speed 4Hz
																01 - Fluid Temperatures etc 0.2 Hz 
																02 - For every startup */
			
			uint8_t service;		//Corresponding Service Mode
						
			uint8_t PID;				//PID under service 

			uint8_t PID1;

			
		} request_param;
		
		typedef struct
		{
			uint32_t pgn_id;				//CAN_ID
			
			uint8_t num_params; 		//# of params in the PGN
			
			uint8_t param_strt;			//First param present in the PGN
			
		} can_pgn;
			


	
		
#endif  /* _FILE_HANDLING_H_ */