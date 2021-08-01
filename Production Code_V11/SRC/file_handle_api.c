#include "stdio.h"
#include "string.h"
#include "file_handle_api.h"
#include "ff.h"
#include "esp.h"
#include "uart_driver.h"
#include "delay.h"
#include "ymodem.h"



char* fn_increment(char filename[12], int fcount, int day)
{
	if(fcount)
	{
		filename[7]+=0x01;
		if(filename[7]==0x3A)
		{
			filename[7] = 0x30;
			filename[6] += 0x01;
		}
  }

	else if(day)
	{
		filename[5]+=0x01;
		filename[7] = 0x30;
	  filename[6] = 0x30;
		if(filename[5]==0x3A)
		{
			filename[5]=0x30;
			filename[4]+=0x01;
		}
		if(filename[4]==0x33 && filename[5]==0x32)
		{
			filename[4]=0x30;
			filename[5]=0x31;
			filename[3]+=0x01;
		}
		if(filename[3]==0x3A)
		{
			filename[3]=0x30;
			filename[2]+=0x01;
		}
		if(filename[2]==0x31 && filename[3]==0x33)
		{
			filename[4]=0x30;
			filename[5]=0x31;
			filename[2]=0x30;
			filename[3]=0x31;
			filename[1]+=0x01;
		}
		if(filename[1]==0x3A)
		{
			filename[1]=0x30;
			filename[0]+=0x01;
		}
	}
	return filename;
}

char* TEMP_FILENAME_HANDLE(char* checker_filename)
{	
	checker_filename[7]+=0x01;
	if(checker_filename[7]==0x3A)
	{
		checker_filename[7]=0x30;
		checker_filename[6]+=0x01;
		if(checker_filename[6]==0x3A)
		{
			checker_filename[6]=0x30;
			checker_filename[5]+=0x01;
			if(checker_filename[5]==0x3A)
			{
				checker_filename[5]=0x30;
				checker_filename[4]+=0x01;
				if(checker_filename[4]==0x3A)
				{
					checker_filename[4]=0x30;
					checker_filename[3]+=0x01;
					if(checker_filename[3]==0x3A)
					{
						checker_filename[3]=0x30;
						checker_filename[2]+=0x01;
					}

				}
			}
		}
	}

	return checker_filename;
}



