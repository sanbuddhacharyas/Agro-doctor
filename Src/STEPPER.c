#include "STEPPER.h"


void servo(int angle,TIM_HandleTypeDef* htim)
{
	//0.5 millisecond 0 degree ,1.5 millisecond 90 degree and 2.5 millisecond 180 degree
	//TIM_OC_InitTypeDef sConfigOC;
	float pulse_width;
	pulse_width=(float)(12.)*angle + 500;
	htim->Instance->CCR1 = (float)(0.072)*pulse_width;
}
