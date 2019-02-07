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
/******************Original

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

*/

#define STEPPER_PORT_ROTOR GPIOE						//Left_right means Second_Arm
#define STEPPER_PORT_LEFT_RIGHT GPIOC
#define STEPPER_PORT_FIRST_ARM GPIOE
#define STEPPER_PORT_SECOND_ARM GPIOC

#define STEPPER_ROTOR_SIGNAL	GPIO_PIN_1
#define STEPPER_LEFT_RIGHT_SIGNAL GPIO_PIN_13
#define STEPPER_FIRST_ARM_SIGNAL GPIO_PIN_4
#define STEPPER_SECOND_ARM_SIGNAL GPIO_PIN_13

#define STEPPER_ROTOR_DIRECTION GPIO_PIN_0
#define STEPPER_LEFT_RIGHT_DIRECTION GPIO_PIN_15
#define STEPPER_FIRST_ARM_DIRECTION GPIO_PIN_5
#define STEPPER_SECOND_ARM_DIRECTION GPIO_PIN_15

/*
#define STEPPER_PORT_ROTOR GPIOC						//lr and rotor means same
#define STEPPER_PORT_LEFT_RIGHT GPIOC
#define STEPPER_PORT_FIRST_ARM GPIOE
#define STEPPER_PORT_SECOND_ARM GPIOB

#define STEPPER_ROTOR_SIGNAL	GPIO_PIN_13
#define STEPPER_LEFT_RIGHT_SIGNAL GPIO_PIN_13
#define STEPPER_FIRST_ARM_SIGNAL GPIO_PIN_4
#define STEPPER_SECOND_ARM_SIGNAL GPIO_PIN_4

#define STEPPER_ROTOR_DIRECTION GPIO_PIN_15
#define STEPPER_LEFT_RIGHT_DIRECTION GPIO_PIN_15
#define STEPPER_FIRST_ARM_DIRECTION GPIO_PIN_5
#define STEPPER_SECOND_ARM_DIRECTION GPIO_PIN_5
*/

#define sig_port GPIOD
#define sig1 GPIO_PIN_5
#define sig2 GPIO_PIN_6

#define Nozzle_In1_Port	GPIOD
#define Nozzle_In2_Port	GPIOC
#define Nozzle_In1_Signal_Pin	GPIO_PIN_0
#define Nozzle_In2_Signal_Pin	GPIO_PIN_12
#define HIGH GPIO_PIN_SET
#define LOW  GPIO_PIN_RESET

#define ROTOR_CLOCKWISE 1
#define ROTOR_ANTICLOCKWISE 2

#define Right 1
#define Left 0
#define Front 1
#define Back 0
#define fullcounter 8645
void servo(int angle ,TIM_HandleTypeDef* htim);
void pid_velocity(int distance);
float left_right_angle(void);
float distance_travelled(uint32_t encoder_reading_wheel);
void move(uint32_t distance, float velocity,int direction);
void set_angle(float angle,uint8_t direction);
int pid(int16_t set_distance,uint16_t wind_up,uint8_t mode);
void set_rotor_angle(int ang);
float initial_angle(void);
void Calibrate_Base(void);
void Initialize_Steppers(void);
void Send_Throttels_To_AVR(void);
void Nozzle_On(void);
void Nozzle_Off(void);
void Set_To_Position(void);
void Stop(void);
void Initialize_Encoder_Counts(void);

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

