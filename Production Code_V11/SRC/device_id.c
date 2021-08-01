#include "device_id.h"
#include<stdio.h>
#include<string.h>


unsigned int device_id_buff;

int get_device_id(void)
{
 device_id_buff = *(volatile uint32_t *)UID_BASE;
 return device_id_buff;
}