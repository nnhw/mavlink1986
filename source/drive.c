#include <MDR32Fx.h> 
#include <math.h> 
#include <MDR32F9Qx_timer.h>
#include "drive.h"
#include "information_exchange.h"
#include "hardware.h"

uint16_t angle1 = 1000; //angle - time in us. From 1000 to 2000
uint16_t angle2 = 1000; //angle - time in us. From 1000 to 2000
uint16_t angle4 = 1000; //angle - time in us. From 1000 to 2000

uint16_t P_angle1 = 1500; //angle - time in us. From 1000 to 2000 !!!убрать в функцию после отладки!
uint16_t PID_angle1 = 1500; //angle - time in us. From 1000 to 2000 !!!убрать в функцию после отладки!

float proportional_coefficient = 160;
float integral_coefficient = 0;
float differential_coefficient = 0;

void refresh_pwm(uint32_t _channel,uint16_t _value)
{
	TIMER_SetChnCompare(MDR_TIMER3, _channel, _value);
}

void manual_mode_control(void)
{
	refresh_pwm(TIMER_CHANNEL1,mavlink_angle_1);
	refresh_pwm(TIMER_CHANNEL2,mavlink_angle_2);
	refresh_pwm(TIMER_CHANNEL3,mavlink_angle_3);
	refresh_pwm(TIMER_CHANNEL4,mavlink_angle_4);
}

void Timer1_IRQHandler(void) //step timing
{
	
	TIMER_ClearFlag(MDR_TIMER1,TIMER_STATUS_CNT_ARR);

			
}

int16_t filter_moving_average (int16_t _value)
{
	static int16_t _value_prev;

	_value_prev =  _value*FILTER_COEFF+_value_prev*(1-FILTER_COEFF);

	return _value_prev;

}

int16_t calculate_PID_ch_1 (void)
{
	
	static float error_prev = 0;
	static float I_prev = 0;
	static float angle_prev = 1500;
	
	float angle_value = 0; 
	float error = 0;
	float P, I, D = 0;

	error = mavlink_roll - mavlink_roll_setpoint/DEG_IN_RAD;
		
	if (fabs(error) < TOLERABLE_ERROR)
		{
			angle_value = angle_prev;	
		} 

	else
	{

		P = proportional_coefficient * error;
		I = I_prev + integral_coefficient * error;
		if(I>500) I = 500;
		D = differential_coefficient * (error - error_prev);

		angle_value = P+I+D;
		error_prev = error;
		I_prev = I;
		angle_value += 1500;

		if(angle_value>2000) angle_value = 2000;
		if(angle_value<1000) angle_value = 1000;

		angle_prev = angle_value;

	}

	return (uint16_t)angle_value;

}


int16_t calculate_pwm_P (float _value,float _setpoint)
{

	float error = 0;
	float result = 0;

	error = _value - _setpoint;
	result = 1500 + (error / PI)*500;

	return (uint16_t)result;

}

void PID_control(void)
{
	PID_angle1 = calculate_PID_ch_1 ();
	refresh_pwm(TIMER_CHANNEL1,PID_angle1);

}


void proportional_control(void)
{
	P_angle1 = calculate_pwm_P (mavlink_roll,mavlink_roll_setpoint);
	refresh_pwm(TIMER_CHANNEL1,P_angle1);

}

void stabilization(void)
{
	P_angle1 = calculate_pwm_P (mavlink_roll,0);
	refresh_pwm(TIMER_CHANNEL1,P_angle1);

}


void experimental_stabilization(void)
{

	angle1 = 1500 - 0.5*mavlink_raw_yacc;
	angle4 = angle1;
	angle2 = 1500 - 0.5*mavlink_raw_xacc;

	refresh_pwm(TIMER_CHANNEL1,angle1);
	refresh_pwm(TIMER_CHANNEL2,angle2);
	refresh_pwm(TIMER_CHANNEL4,angle4);

}

void system_default_state(void)
{
  refresh_pwm(TIMER_CHANNEL1,1500);
  refresh_pwm(TIMER_CHANNEL2,1500);
  refresh_pwm(TIMER_CHANNEL3,1500);
  refresh_pwm(TIMER_CHANNEL4,1500);
}

/*
int16_t calculate_PID (int16_t _channel,int16_t _value)
{
	static int16_t angle_prev = 0;

	static int16_t error_prev = 0;

	static int16_t counter = 0;

	static int16_t integral_array[INTERGRAL_ARRAY_LENGHT] = {0};

	int16_t i = 0;

	int16_t angle_value = 0;

	int16_t integral_value = 0;

	int16_t error;


	
	//////////////// вычисление интеграла
	for (i = 0; i <= counter; i++)
	{
		integral_value += integral_array[i];
	}

	integral_array[counter] = integral_value; //лишнего суммирую

	if (counter < 20)
	{
		counter++;
	}

	integral_value = integral_value / counter; // спорно, надо ли?
	
	//////////////// вычисление производной
	differential_value = (error - error_prev)/PID_TIME_INTERVAL;


	error = mavlink_raw_yacc - mavlink_setpoint_yacc;

	switch (_channel)
	{
		case AILERON:
		{
			angle_value = (error*PROPORTIONAL_COEFFICIENT+error*INTERGRAL_COEFFICIENT*integral_value+error*DIFFERENTIAL_COEFFICIENT*differential_value)*CONVERSION_COEFFICIENT;
			//angle_value = (error)*PROPORTIONAL_COEFFICIENT*CONVERSION_COEFFICIENT;
		}
		break;

		case ELEVATOR:
		{

		}

		case RUDDER:
		{

		}

		break;
		default:
		break;
	}


	return angle_value;

}
*/
