#ifndef PID_H_
#define PID_H_


#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "STEPPER.h"
#include "mpu6050.h"


void PID_calculate(STEPPER* INFO ,int pid_setpoint ,  int updated_angle);
void Pulse_Width_Calculator(STEPPER* INFO);


#endif