#ifndef __MYMATH_H
#define __MYMATH_H


#include <stdbool.h>
#include <stdint.h>

#define T_SAMPLE 3

typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	float I_part;
	float e_pre;
	uint8_t time_sample;
	uint16_t pos;
	float pwm;
	
	bool pid_process;
	
	bool home;
	
	float next;

} AXIS;

typedef struct
{
	char ACK[11];
	char NAK[5];
	char TransBuff[25];
	char ReceiveBuff[27];
	char Command[3];
	
	int receive;
	int need;
} DATA;

#endif
