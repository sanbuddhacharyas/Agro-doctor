#include "STEPPER.h"
#include <math.h>

float linear_encoder_in_cm = 120;		//Resolution:120 lines per centimeters
float radius =3;
float wheel_size = 55.6;
float	travel = 0;
float angl=0;
int pid_error;
int PID;
extern float p_scalar , i_scalar, d_scalar;
float integral = 0;
float proportional=0;
float previous_error = 0;
float derivative = 0;
float left_right_error =0;
float test , test1;
 
extern volatile uint32_t encoder_reading_wheel;
extern volatile uint32_t encoder_reading_left_right;
extern volatile uint8_t direction_wheel;
extern volatile uint8_t direction_left_right;
extern volatile int forward_speed;
extern volatile float velocity;
extern volatile int throttel_left, throttel_right;
extern volatile uint16_t encoder_reading_pre;
extern volatile int total_distance ;

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

void move(uint32_t distance, float velocity,int dir)
{
	if(dir == Front)
	{
		while(distance > encoder_reading_wheel )
		{
	
			HAL_GPIO_WritePin(sig_port,sig1,GPIO_PIN_SET);
			HAL_GPIO_WritePin(sig_port,sig2, GPIO_PIN_RESET);
			htim2.Instance->CCR1 = (int)((20 - forward_speed) * (2800 / 17));
			HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
		}	
	
	}
	
	if(dir == Back)
	{
		while(distance > encoder_reading_wheel)
		{
		
			HAL_GPIO_WritePin(sig_port,sig1,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(sig_port,sig2, GPIO_PIN_SET);
			htim2.Instance->CCR1 = (int)((20 - forward_speed) * (2800 / 17));
			HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
				
		}
	
	
	}	
	TIM4->CNT = 0;
	encoder_reading_wheel = 0;
	encoder_reading_pre = 0;
	HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
	HAL_GPIO_WritePin(sig_port,sig1,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(sig_port,sig2, GPIO_PIN_RESET);
			
}

void set_angle(float ang,uint8_t direction)
{
	test1 = ang;
	test = left_right_angle();
	left_right_error = ang - test;
	if (left_right_error < 0.1 && left_right_error > -0.1)
		left_right_error =0;
	
	if(left_right_error > 0 )
	{
		throttel_left =  -70;				//motor will go towards right
		throttel_right = -70;	
		
	}
	else if(left_right_error < 0)
	{
		throttel_left = 70;			//motor will go toward left
		throttel_right = 70;
	}

	else
	{
			throttel_left =  0;
			throttel_right = 0;	
	}
} 

//Sets PID for required distance
/*
mode:0 for DC motor
mode:1 for stepper

Wind_up is the value to limit capacity of motor
*/
int pid(int16_t set_distance,uint16_t wind_up,uint8_t mode)
{
	 pid_error = set_distance - total_distance;
	 if(PID >10 || PID < -10)
		 pid_error += PID * 0.015 ;

	 proportional = pid_error * p_scalar;//Proportional 
	
	 integral += pid_error * i_scalar; // Intregal 
	 if(integral >  wind_up) integral = wind_up; // limit wind-up
	 if(integral < - wind_up) integral =-wind_up;

	 derivative = (pid_error - previous_error) * d_scalar;// Derivative
	 previous_error = pid_error;
	 PID = proportional+derivative+integral; // Required PID
	 if(PID > wind_up) PID = wind_up;
	 if(PID <-wind_up)PID= -wind_up;
	 
	 if(PID < 35 && PID>-35) 
		 PID =0;//Create a dead-band to stop the motors when the robot is balanced
	
		if(mode == 0)
			return PID;
		else 
			return 0;
}

