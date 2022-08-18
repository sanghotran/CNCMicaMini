#ifndef __MYSTRUCT_H
#define __MYSTRUCT_H


#include <stdbool.h>
#include <stdint.h>

#define T_SAMPLE 3

typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	float I_part;
	int e_pre;
	uint8_t time_sample;
	int pos;
	float pwm;
	int setpoint;
	
	bool finish;
	
	bool pid_process;
	
	bool home;
	
	float next;

} AXIS;

typedef struct
{
	char TransBuff[45];
	char ReceiveBuff[27];
	char Command[3];
	
	int receive;
	int need;
} DATA;

#endif
