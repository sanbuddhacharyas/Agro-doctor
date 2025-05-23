/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define Stepper3_Signal_Pin GPIO_PIN_4
#define Stepper3_Signal_GPIO_Port GPIOE
#define Stepper3_Direction_Pin GPIO_PIN_5
#define Stepper3_Direction_GPIO_Port GPIOE
#define Stepper2_Signal_Pin GPIO_PIN_13
#define Stepper2_Signal_GPIO_Port GPIOC
#define Stepper2_Direction_Pin GPIO_PIN_15
#define Stepper2_Direction_GPIO_Port GPIOC
#define Rotor_enco_1_Pin GPIO_PIN_6
#define Rotor_enco_1_GPIO_Port GPIOA
#define Rotor_enco_2_Pin GPIO_PIN_7
#define Rotor_enco_2_GPIO_Port GPIOA
#define Wheel_enco_1_Pin GPIO_PIN_9
#define Wheel_enco_1_GPIO_Port GPIOE
#define Wheel_enco_2_Pin GPIO_PIN_11
#define Wheel_enco_2_GPIO_Port GPIOE
#define Stopper_Interrupt_Pin GPIO_PIN_10
#define Stopper_Interrupt_GPIO_Port GPIOD
#define Stopper_Interrupt_EXTI_IRQn EXTI15_10_IRQn
#define LED_Pin GPIO_PIN_14
#define LED_GPIO_Port GPIOD
#define Nozzle1_In1_Pin GPIO_PIN_12
#define Nozzle1_In1_GPIO_Port GPIOC
#define Nozzle1_In2_Pin GPIO_PIN_0
#define Nozzle1_In2_GPIO_Port GPIOD
#define Nozzle2_In2_Pin GPIO_PIN_3
#define Nozzle2_In2_GPIO_Port GPIOD
#define Nozzle2_In1_Pin GPIO_PIN_4
#define Nozzle2_In1_GPIO_Port GPIOD
#define Stepper4_Signal_Pin GPIO_PIN_4
#define Stepper4_Signal_GPIO_Port GPIOB
#define Stepper4_Direction_Pin GPIO_PIN_5
#define Stepper4_Direction_GPIO_Port GPIOB
#define Stepper1_Direction_Pin GPIO_PIN_0
#define Stepper1_Direction_GPIO_Port GPIOE
#define Stepper1_Signal_Pin GPIO_PIN_1
#define Stepper1_Signal_GPIO_Port GPIOE

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
