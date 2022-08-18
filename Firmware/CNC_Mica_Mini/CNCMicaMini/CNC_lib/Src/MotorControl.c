#include "Inc/MotorControl.h"
#include <math.h>

void PID_control(int sp, AXIS *pid)
{
	int e;
	e = sp - pid->pos;
	pid->I_part += TS*e;
	pid->pwm = pid->Kp*e + pid->Ki*pid->I_part + pid->Kd*(e-pid->e_pre)/TS;
	pid->e_pre = e;
	if( int_abs(pid->e_pre) < 25)
		pid->finish = true;
}

void move_to(int *x_last, int *y_last, float x, float y, int scale_factor, float x_step_per_mm, float y_step_per_mm)
{
	int x1 = *x_last;
	int y1 = *y_last;
	int x2 = round(x * scale_factor * x_step_per_mm);
	int y2 = round(y * scale_factor * y_step_per_mm);
	draw_line(x1, y1, x2, y2);
	*x_last = x2;
	*y_last = y2;
}

void draw_line(int x1, int y1, int x2, int y2)
{
	int dx = x2 - x1;
	int dy = y2 - y1;
	int longest = int_max(int_abs(dx), int_abs(dy));
	int shortest = int_min(int_abs(dx), int_abs(dy));
	int error = - longest;
	int threshold = 0;
	int maximum = (longest << 1);
	int slop = (shortest << 1);
	bool swap_XY = true;
	
	if(int_abs(dx) >= int_abs(dy))
		swap_XY = false;
	for(int i = 0; i < longest; i++)
	{
		if(swap_XY)
		{
			if(dy < 0)
				move_y(DOWN);
			else
				move_y(UP);
		}
		else
		{
			if(dx < 0)
				move_x(LEFT);
			else
				move_x(RIGHT);
		}
		
		error += slop;
		if(error > threshold)
			error = - maximum;
		if(swap_XY)
		{
			if(dx < 0)
				move_x(LEFT);
			else
				move_x(RIGHT);
		}
		else
		{
			if(dy < 0)
				move_y(DOWN);
			else
				move_y(UP);
		}
	}
}

void move_x(bool direction)
{
	
}

void move_y(bool direction)
{
	
}

void draw_arc_cw(float x, float y, float i, float j, int *x_last, int *y_last, int scale_factor, float x_step_per_mm, float y_step_per_mm)
{
	if((i < -100) || (i > 100) || (j < - 100) || (j > 100))
		move_to(x_last, y_last, x, y, scale_factor, x_step_per_mm, y_step_per_mm);
	else
	{
		float dx = x - *x_last;
		float dy = y - *y_last;
		float chord = sqrt(dx * dx + dy * dy);
		float radius = sqrt( i * i + j * j);
		float alpha = 2 * asin(chord / ( 2 * radius));
		float arc = alpha * radius;
		
		int segments = 1;
		float beta;
		if(arc > LINE_MAX)
		{
			segments = (int)(arc / LINE_MAX);
			beta = alpha / segments;
		}
		else
			beta = alpha;
		
		float current_angle = atan2(-j, -i);
		if( current_angle <= 0)
			current_angle += 2 * PI;
		
		float next_angle = current_angle;
		float circle_x = *x_last + i;
		float circle_y = *y_last + j;
		float new_x, new_y;
		for( int segment = 1; segment < segments; segment++)
		{
			next_angle += beta;
			if(next_angle > 2 * PI)
				next_angle -= 2 * PI;
			new_x = circle_x + radius*cos(next_angle);
			new_y = circle_y + radius*sin(next_angle);
			move_to(x_last, y_last, new_x,new_y, scale_factor, x_step_per_mm, y_step_per_mm);
		}
		move_to(x_last, y_last, x, y, scale_factor, x_step_per_mm, y_step_per_mm);
	}
}

