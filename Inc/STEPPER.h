#ifndef __STEPPER
#define __STEPPER

#include "stm32f4xx_hal.h"
#include "tim.h"

#define stepper_port GPIOC
#define stepper1_sig GPIO_PIN_2
#define stepper1_dir GPIO_PIN_3
#define stepper2_sig GPIO_PIN_11
#define stepper2_dir GPIO_PIN_2
#define HIGH GPIO_PIN_SET
#define LOW  GPIO_PIN_RESET
#define sig_port GPIOC
#define sig1 GPIO_PIN_0
#define sig2 GPIO_PIN_1
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

typedef struct {
	uint8_t Signal;
	uint8_t Direction;
	float pid_error;
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

