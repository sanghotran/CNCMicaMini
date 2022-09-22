#include "Inc/MotorControl.h"

void PWM(AXIS *axis)
{
	float duty = axis->pwm;
	if (duty > 1) duty = 1;
	else if(duty ==0)  duty = 0;
	else if (duty < -1) duty = -1;
	int16_t pwm = duty*MAX_SPEED;
		
	if (pwm > 0)
		{
			__HAL_TIM_SetCompare(axis->htim_motor, axis->CHANNEL, 100 - pwm);
			HAL_GPIO_WritePin(axis->GPIO_DIR, axis->PIN_DIR, GPIO_PIN_SET);
		}
	else if (pwm == 0)
		{
			__HAL_TIM_SetCompare(axis->htim_motor, axis->CHANNEL, 0);
			HAL_GPIO_WritePin(axis->GPIO_DIR, axis->PIN_DIR, GPIO_PIN_RESET);
		}
	else if (pwm < 0)
		{
			pwm *= -1;
			__HAL_TIM_SetCompare(axis->htim_motor, axis->CHANNEL, pwm);
			HAL_GPIO_WritePin(axis->GPIO_DIR, axis->PIN_DIR, GPIO_PIN_RESET);

		}
}

void readEncoder(TIM_HandleTypeDef* htim, int *Pos)
{
	*Pos += (int16_t)(htim->Instance->CNT);
	htim->Instance->CNT=0;
}

void sample(AXIS * axis)
{
	if(axis->pid_process)
	{
		axis->time_sample++;
		if( axis->time_sample >= T_SAMPLE )
		{
			readEncoder(axis->htim_enc, &axis->pos);
			PID_control(axis->setpoint, axis);
			axis->time_sample = 0;			
		}
	}
}

void PID_control(int sp, AXIS *pid)
{
	int e;
	e = sp - pid->pos;
	if(int_abs(e) < ERROR)
	{
		pid->finish = true;
		return;
	}
	pid->I_part += TS*e;
	pid->pwm = pid->Kp*e + pid->Ki*pid->I_part + pid->Kd*(e-pid->e_pre)/TS;
	pid->e_pre = e;
}

void HOME(AXIS *axis)
{
	if( HAL_GPIO_ReadPin(axis->GPIO_HOME, axis->PIN_HOME) == 1)
		{
			axis->pwm = -0.7;
			PWM(axis);
			axis->home = false;
		}
	else
		{
			axis->pwm = 0;
			PWM(axis);
			axis->home = true;
		}
}
void move(AXIS *axis, float pos)
{
	if( axis->pid_process == false)
	{
		axis->setpoint = (int)(pos*axis->mm_pulse);
	  axis->pid_process = true;
	}
	PWM(axis);	
	if( axis->finish)
	{
		axis->pwm = 0;
		PWM(axis);
		axis->pid_process = false;		
		axis->time_sample = 0;		
	}
}
void moveGcode(AXIS *pAxis)
{
	pAxis->setpoint = (int)(pAxis->last * pAxis->mm_pulse);
	pAxis->pid_process = true;
	while( pAxis->pid_process)
	{
		PWM(pAxis);	
		if( pAxis->finish)
		{
			pAxis->pwm = 0;
			PWM(pAxis);
			pAxis->pid_process = false;		
			pAxis->time_sample = 0;		
			pAxis->finish = false;
		}
	}
}
void drawLine(AXIS *pXAxis, AXIS *pYAxis)
{
	// declare variable
	int dx = round(pXAxis->next - pXAxis->last);
	int dy = round(pYAxis->next - pYAxis->last);
	int longest = int_max(int_abs(dx), int_abs(dy));
	int shortest = int_min(int_abs(dx), int_abs(dy));
	int error = - longest;
	int slope = shortest<<1;
	int max = longest<<1;
	int threshold = 0;
	bool swapXY = true;
	
	// swap x and y if dy > dx
	if(int_abs(dx) > int_abs(dy))
		swapXY = false;
	
	// Bresenham Algorithm
	for(int i = 0; i < longest; i++)
	{
		if(swapXY)
		{
			if( dy > 0)
				pYAxis->last ++;
			else
				pYAxis->last --;
			moveGcode(pYAxis);
		}
		else
		{
			if( dx > 0)
				pXAxis->last ++;
			else
				pXAxis->last --;
			moveGcode(pXAxis);
		}
		error = error + slope;
		if(error >= threshold)
		{
			error = error - max;
			if(!swapXY)
			{
				if( dy > 0)
					pYAxis->last ++;
				else
					pYAxis->last --;
				moveGcode(pYAxis);
			}
			else
			{
				if( dx > 0)
					pXAxis->last ++;
				else
					pXAxis->last --;
				moveGcode(pXAxis);
			}
		}
	}
}
