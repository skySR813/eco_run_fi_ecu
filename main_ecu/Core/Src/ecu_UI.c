#include "liquidcrystal_i2c.h"
#include <stdio.h>
#include <math.h>

void UIprint_init(void){
	//osMutexWait(I2C_mutexHandle, osWaitForever);
	HD44780_Init(2);
	HD44780_Clear();
	//osMutexRelease(I2C_mutexHandle);
}

void UIprint_int(int com1,int *last_com1,char comm1[16],int yoko1,int tate1){
	if(com1 != *last_com1){
		*last_com1 = com1;
		sprintf(comm1,"%4d",com1);
		//osMutexWait(I2C_mutexHandle,osWaitForever);
		HD44780_SetCursor(yoko1,tate1);
		HD44780_PrintStr(comm1);
		//osDelay(100);
		//HD44780_PrintStr("    ");
		//osMutexRelease(I2C_mutexHandle);

	}
}

void UIprint_float(float com2,float *last_com2,char comm2[16],int yoko2,int tate2)
{
    if (fabsf(com2 - *last_com2) > 0.05f)
    {
        snprintf(comm2,16,"%4.1f",com2);
        HD44780_SetCursor(yoko2,tate2);
        HD44780_PrintStr(comm2);
        *last_com2 = com2;
    }
}

/*
void UIprint_float(float com2,float *last_com2,char comm2[16],int yoko2,int tate2){
	if(com2 != *last_com2){
		*last_com2 = com2;
		sprintf(comm2,"%4.1f",com2);
		osMutexWait(I2C_mutexHandle,osWaitForever);
		HD44780_SetCursor(yoko2,tate2);
		HD44780_PrintStr(comm2);
		osDelay(50);
		HD44780_PrintStr("    ");
		osMutexRelease(I2C_mutexHandle);

	}
}
*/
