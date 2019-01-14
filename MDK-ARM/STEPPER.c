#include "STEPPER.h"
#include <math.h>


float linear_encoder_in_cm = 120 ;
float radius =3;
uint32_t wheel_size = 55;
float		travel = 0;
float angl=0;


extern volatile uint32_t encoder_reading_wheel;
extern volatile uint32_t encoder_reading_left_right;
extern volatile uint8_t direction_wheel;
extern volatile uint8_t direction_left_right;
extern volatile int forward_speed;
extern volatile float velocity;
extern volatile int throttel_left, throttel_right;


float distance_travelled(uint32_t encoder_reading_wheel)
{
	
		return ((wheel_size*encoder_reading_wheel)/fullcounter);
	
}

float left_right_angle()
{
	if(encoder_reading_left_right >1000)
	{
		return(asin((3000-encoder_reading_left_right)/(radius* linear_encoder_in_cm)) * 57.29577951);
	}
	else
	{
		return -(asin(encoder_reading_left_right/ (radius * linear_encoder_in_cm)) * 57.2957795);
	}
	
}

void move(float distance, float velocity,int dir)
{
	
	float dis;
	
	if(dir == Front)
	{
		while(distance > encoder_reading_wheel )
		{
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
			
			HAL_GPIO_WritePin(sig_port,sig1,GPIO_PIN_SET);
			HAL_GPIO_WritePin(sig_port,sig2, GPIO_PIN_RESET);
			htim2.Instance->CCR1 = (int)((20 - forward_speed) * (2800 / 17));
			HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
		}
		
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);	
	
	}
	
	if(dir == Back)
	{
		while(distance <= travel)
		{
			dis = distance_travelled(encoder_reading_wheel);
			HAL_GPIO_WritePin(sig_port,sig1,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(sig_port,sig2, GPIO_PIN_SET);
			htim2.Instance->CCR1 = (int)((20 - forward_speed) * (2800 / 17));
			HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
				
		}
	
	
	}	
			
}

void set_angle(float ang,uint8_t direction)
{
	if(direction == Right)
	{
		angl = left_right_angle();
		while(ang > angl  )
		{
			angl = left_right_angle();
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
			throttel_left =  -70;
			throttel_right = -70;	
		}
		throttel_left =  0;
		throttel_right = 0;	
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);
	}

	else if(direction == Left)
	{	
		angl = left_right_angle();
		while(ang <  angl)
		{
			angl = left_right_angle();
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
			throttel_left = 70;
			throttel_right = 70;
		}
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);
	}
}