#ifndef _ESP_H
#define _ESP_H

extern char  *FILE_TRANSFER;
extern char  *LAST_FILE_SIZE;
extern char  *LAST_FILE;
extern char  *LAST_DATED_FILE;
extern char  *FILETXDF;
extern char  *UPLDFXS;
extern char  *GEAR_UP;
extern char  *GEAR_DOWN;
extern char  *HOTSPOTON;
extern char  *HOTSPOTOFF;
extern char  *WELCOME;
extern char  *IFT_SER;
extern char  *DOWN0;
extern char  *DOWN1;
extern char  *DOWN2;
extern char  *DOWN3;
extern char  *DOWN4;
extern char  *DOWN5;
extern char  *DOWN6;
extern char  *DOWN7;

extern char  *FILE0;
extern char  *FILE1;
extern char  *FILE2;
extern char  *FILE3;
extern char  *FILE4;
extern char  *FILE5;
extern char  *FILE6;
extern char  *FILE7;


extern char  *KEEP_IGN_ON;
extern char  *CHECK_UPDATES;
extern char  *TURN_VEHCILE_ON;
extern char  *TURN_VEHCILE_OFF;
extern char  *FLASHING_COMPLETE;
extern char  *ECU_UPDATE_AVAILABLE;
extern char  *Backup_Audio;
extern char  *PRESS_BUTTON;

extern char  *PRESS_CLUTCH;
extern char  *RELEASE_CLUTCH;
extern char  *PRESS_BRAKE;
extern char  *RELEASE_BRAKE;
extern char  *FIRST_GEAR;
extern char  *NEUTRAL;
extern char  *ENG_ON;
extern char  *PRESS_THROTTLE;
extern char  *RELEASE_THROTTLE;
extern char  *ENG_OFF;
extern char  *BRD_DONE;
extern char  *REQ_DONE;

extern char  *WIFI_ON;

extern char  *FIRSTb;

void ESP_CMD_SEND(char *s);
void ESP_Init(void);


#endif
