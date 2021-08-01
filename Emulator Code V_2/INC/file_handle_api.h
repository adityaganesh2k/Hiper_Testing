#ifndef _FILE_HANDLE_API
#define _FILE_HANDLE_API

#include "ff.h"

		typedef struct{
			uint16_t year, month, day;
			unsigned int fid;
		}loginfo;
		
		FRESULT fncheck(const char* fname);
		char* 	TEMP_FILENAME_HANDLE(char checker_filename[]);
		int  		ESPTRANSMIT(void);
		//FRESULT scan_file(char* path, const char* lastfile);
		int internal_file_tranfer(char* filename);
		
		char* fn_increment(char filename[], int fcount, int day);


#endif