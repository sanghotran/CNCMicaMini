#ifndef __MYSTRUCT_H
#define __MYSTRUCT_H


#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"

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
	
	TIM_HandleTypeDef* htim_enc;
	
	TIM_HandleTypeDef* htim_motor;
	uint32_t CHANNEL;
	
	GPIO_TypeDef* GPIO_DIR;
	uint16_t PIN_DIR;
	
	GPIO_TypeDef* GPIO_HOME;
	uint16_t PIN_HOME;
	
  float mm_pulse;
	
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
