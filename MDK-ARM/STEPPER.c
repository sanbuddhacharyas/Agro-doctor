#include "STEPPER.h"
#include "math.h"
#include <stdio.h>

#define SCALE 24
float linear_encoder_in_cm = 120;		//Resolution:120 lines per centimeters
float radius =3;
float wheel_size = 55.6;
float	travel = 0;
float angl=0;
int pid_error;
int PID , desired_position;
extern float p_scalar , i_scalar, d_scalar;
float integral = 0;
float proportional=0;
float previous_error = 0;
float derivative = 0;
float left_right_error =0;
float test , test1;
char AVR_Data[50] = {0};

extern volatile uint32_t encoder_reading_wheel;
extern volatile uint32_t encoder_reading_left_right;
extern volatile uint8_t direction_wheel;
extern volatile uint8_t direction_left_right;
extern volatile int forward_speed;
extern volatile float velocity;
extern volatile int throttel_left, throttel_right;
extern volatile uint16_t encoder_reading_pre;
extern volatile int total_distance ;
extern float current_angle;
int input = 0;

	extern STEPPER Rotor;
	extern STEPPER Left_Right;
	extern STEPPER First_Arm;
	extern STEPPER Second_Arm;

float distance_travelled(uint32_t encoder_reading_wheel)
{
	
		return ((wheel_size*encoder_reading_wheel)/fullcounter);
	
}

float left_right_angle(void) 
{
	if(encoder_reading_left_right >1000)
	{
		return(asin((10000-encoder_reading_left_right)/(radius* linear_encoder_in_cm)) * 57.29577951);
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
			htim8.Instance->CCR3 = (int)((20 - forward_speed) * (2800 / 17));
			HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
		}	
	}
	
	if(dir == Back)
	{
		while(distance > encoder_reading_wheel)
		{
	
			HAL_GPIO_WritePin(sig_port,sig1,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(sig_port,sig2, GPIO_PIN_SET);
			htim8.Instance->CCR3 = (int)((20 - forward_speed) * (2800 / 17));
			HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
				
		}
	}	 
	TIM1->CNT = 0;
	encoder_reading_wheel = 0;
	encoder_reading_pre = 0;
	HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_3);
	HAL_GPIO_WritePin(sig_port,sig1,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(sig_port,sig2, GPIO_PIN_RESET);
			
}

void set_angle(float ang,uint8_t direction)
{
	test1 = ang;
	test = left_right_angle();
	left_right_error = ang - test;
	if (left_right_error < 0.5 && left_right_error > -0.5)
		left_right_error =0;
	
	if(left_right_error > 0 )
	{
		Left_Right.throttel =  -20;				//motor will go towards right
	}
	else if(left_right_error < 0)
	{
		Left_Right.throttel = 20;			//motor will go toward left
	}
	else
	{
			Left_Right.throttel = 0;
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

float initial_angle(void)
{
//	float ang;
	//if(direction_left_right == Right)
	//else
		//ang = -((10000 - encoder_reading_wheel)/24);
	
	return ((float)(encoder_reading_wheel/24));
}
	
void set_rotor_angle(int input_angle)
{
	int angle = current_angle;
	input = input_angle;
	if(fabs((float)input_angle - angle) > 0.1)
	{
		if(input_angle > angle)
		Rotor.throttel =10;
	else
		Rotor.throttel = -10;
	}
	else
		Rotor.throttel = 0;
	
}

void Initialize_Steppers(void)
	{
		Rotor.Signal = STEPPER_ROTOR_SIGNAL;
		Rotor.Direction = STEPPER_ROTOR_DIRECTION;
		
		Left_Right.Signal = STEPPER_LEFT_RIGHT_SIGNAL;
		Left_Right.Direction = STEPPER_LEFT_RIGHT_DIRECTION;
		Left_Right.p_scalar = 25;
		
		First_Arm.Signal = STEPPER_FIRST_ARM_SIGNAL;
		First_Arm.Direction = STEPPER_FIRST_ARM_DIRECTION;
		
		Second_Arm.Signal = STEPPER_SECOND_ARM_SIGNAL;
		Second_Arm.Direction = STEPPER_SECOND_ARM_DIRECTION;
		
		Rotor.Port = STEPPER_PORT_ROTOR;
	  Left_Right.Port = STEPPER_PORT_LEFT_RIGHT;
		First_Arm.Port = STEPPER_PORT_FIRST_ARM;
		Second_Arm.Port = STEPPER_PORT_SECOND_ARM;
	}
		
void Calibrate_Base(void)
{
	throttel_left = -10;
}

void Send_Throttels_To_AVR(void)
{
	sprintf(AVR_Data,"%dp%dq%dr",First_Arm.throttel , Second_Arm.throttel , Rotor.throttel);
}
	

/*   After calibration, make the initial value as 5000.
Calculate present position i.e. current angle everytime in f4xx_it.c by taking the difference of latest counts.*/

