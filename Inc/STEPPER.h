#ifndef __STEPPER
#define __STEPPER

#include "stm32f4xx_hal.h"
#include "tim.h"

/****************STEPPERS******************
MOTOR 1	(Rotor):	STEP = PE1	
								  DIR  = PE0
					
MOTOR 2	(Left_Right):	STEP = PC13
											DIR  = PC15
					
MOTOR 3(First_Arm):	STEP = PE4
										DIR  = PE5
					
MOTOR 4(Second_Arm):	STEP = PB4
											DIR  = PB5			


****************STEPPERS******************/

#define STEPPER_PORT_ROTOR GPIOE
#define STEPPER_PORT_LEFT_RIGHT GPIOC
#define STEPPER_PORT_FIRST_ARM GPIOE
#define STEPPER_PORT_SECOND_ARM GPIOB

#define STEPPER_ROTOR_SIGNAL	GPIO_PIN_1
#define STEPPER_LEFT_RIGHT_SIGNAL GPIO_PIN_13
#define STEPPER_FIRST_ARM_SIGNAL GPIO_PIN_4
#define STEPPER_SECOND_ARM_SIGNAL GPIO_PIN_4

#define STEPPER_ROTOR_DIRECTION GPIO_PIN_0
#define STEPPER_LEFT_RIGHT_DIRECTION GPIO_PIN_15
#define STEPPER_FIRST_ARM_DIRECTION GPIO_PIN_5
#define STEPPER_SECOND_ARM_DIRECTION GPIO_PIN_5

#define HIGH GPIO_PIN_SET
#define LOW  GPIO_PIN_RESET
#define Right 1
#define Left 0
#define Front 1
#define Back 0
#define fullcounter 8645
void servo(int angle ,TIM_HandleTypeDef* htim);
void pid_velocity(int distance);
float left_right_angle();
float distance_travelled(uint32_t encoder_reading_wheel);
void move(uint32_t distance, float velocity,int direction);
void set_angle(float angle,uint8_t direction);
int pid(int16_t set_distance,uint16_t wind_up,uint8_t mode);
void set_rotor_angle(int ang);
float initial_angle();
void Calibrate_Base(void);
void Initialize_Steppers(void);

typedef struct {
	GPIO_TypeDef * Port;
	uint16_t Signal;
	uint16_t Direction;
	float pid_error;
	int setpoint;
	float i_scalar;
	float p_scalar;
	float d_scalar;
	float integral;
	float derivative;
	float proportional;
	float previous_error;
	float motor;
	float pid;
	int  throttel;
	int throttel_counter;
	int throttel_previous_memory;
	int throttel_previous_step;
	} STEPPER;

	
	
#endif

