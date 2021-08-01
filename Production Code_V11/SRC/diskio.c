/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2016        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */



#include "stm32f446xx.h"
#include "sdio_driver.h"

#define BLOCK_SIZE  512 /* Block Size in Bytes */


SDCard_TypeDef SD_CARD;

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat=0;
	return stat;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */

DSTATUS disk_initialize (
        BYTE drv                                /* Physical drive nmuber (0..) */
)
{
	
	DSTATUS stat = 0;

 /*-------------------------- SD Init ----------------------------- */
  if (SD_Init(&SD_CARD) !=SDR_Success)
  {
    stat |= STA_NOINIT;
  }

	return(stat);
}




/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{

	SDResult result;

	
	result = SD_ReadBlock(&SD_CARD,sector*512, (uint32_t *)buff, count*512);

	if (result == SDR_Success)
	{
			return(RES_OK);
	}
	else
		return(RES_ERROR);
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/



DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	SDResult result;

	
	result = SD_WriteBlock(&SD_CARD,sector*512, (uint32_t *)buff, count*512);

	if (result == SDR_Success)
	{
			return(RES_OK);
	}
	else
		return(RES_ERROR);
}




/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */

DRESULT disk_ioctl (
        BYTE drv,               /* Physical drive nmuber (0..) */
        BYTE ctrl,              /* Control code */
        void *buff              /* Buffer to send/receive control data */
)
{
       if (drv != 0)
{
return RES_PARERR;
}

switch (ctrl)
{
case CTRL_SYNC:
//do nothing. By calling SD_WaitReadOperation and
//SD_WaitWriteOperation we already ensure that operations
//complete in the read and write functions.
  return RES_OK;
  break;
default:
return RES_PARERR;
}
}

/*-----------------------------------------------------------------------*/
/* Get current time                                                      */
/*-----------------------------------------------------------------------*/
DWORD get_fattime(void){
        return 0;
}
