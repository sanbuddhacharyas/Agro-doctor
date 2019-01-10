#ifndef __STEPPER
#define __STEPPER

#include "stm32f4xx_hal.h"

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




void servo(int angle ,TIM_HandleTypeDef* htim);


#endif
