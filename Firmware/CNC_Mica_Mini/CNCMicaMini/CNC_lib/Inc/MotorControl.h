#ifndef __MOTORCONTROL_H
#define __MOTORCONTROL_H

#include <stdbool.h>
#include "MyMath.h"
#include "MyStruct.h"
#include "stm32f1xx_hal.h"
#include <math.h>

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#define TS 0.03

#define DOWN  false
#define UP 	  true
#define LEFT  false
#define RIGHT true
	
#define LINE_MAX 2 // max line lengh (mm) when draw circle

void PWM(AXIS *axis);

void readEncoder(TIM_HandleTypeDef* htim, int *Pos);

void sample(AXIS *axis);

void PID_control(int sp, AXIS *pid);

void HOME(AXIS *axis);

void move_to(int *x_last, int *y_last, float x, float y, int scale_factor, float x_step_per_mm, float y_step_per_mm);

void draw_line(int x1, int y1, int x2, int y2);

void move_x(bool direction);

void move_y(bool direction);

void draw_arc_cw(float x, float y, float i, float j, int *x_last, int *y_last, int scale_factor, float x_step_per_mm, float y_step_per_mm);


#endif /* __MOTORCONTROL_H */
