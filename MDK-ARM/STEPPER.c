#include "STEPPER.h"
#include <math.h>

uint32_t fullcounter = 4320;
float linear_encoder_in_cm = 120 ;
float radius =3;
uint32_t wheel_size = 55;

extern volatile uint32_t encoder_reading_wheel;
extern volatile uint32_t encoder_reading_left_right;
extern volatile int direction_wheel;
extern volatile int direction_left_right;
extern volatile int forward_speed;


float distance_travelled()
{
	
		return ((wheel_size*encoder_reading_wheel)/fullcounter);
	
}

float left_right_angle()
{
	if(encoder_reading_left_right >1000)
	{
		return(asin(fullcounter-encoder_reading_left_right)/(radius* linear_encoder_in_cm) * 57.29577951);
	}
	else
	{
		return (asin(encoder_reading_left_right/ (radius * linear_encoder_in_cm)) * 57.2957795);
	}
	
}

void move(float distance, float velocity,int dir)
{
	uint32_t travel = 0;
	float dis;
	if(dir == Front)
	{
		while(distance <= travel)
		{
			dis = distance_travelled();
			HAL_GPIO_WritePin(sig_port,sig1,GPIO_PIN_SET);
			HAL_GPIO_WritePin(sig_port,sig2, GPIO_PIN_RESET);
			htim2.Instance->CCR1 = (int)((20 - forward_speed) * (2800 / 17));
			HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
				
		
		}
		
		dis = 0;
		TIM4->CNT = 0;
	
	}
	
	if(dir == Back)
	{
		while(distance <= travel)
		{
			dis = distance_travelled();
			HAL_GPIO_WritePin(sig_port,sig1,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(sig_port,sig2, GPIO_PIN_SET);
			htim2.Instance->CCR1 = (int)((20 - forward_speed) * (2800 / 17));
			HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
				
		}
		
		dis = 0;
		TIM4->CNT = 0;
	
	}
	
			
}