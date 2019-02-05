/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "Headers.h"

/* USER CODE BEGIN Includes */

#define RAD_TO_DEG 57.295779513082320876798154814105
#define PI 3.1415926535897932384626433832795

#define mpu1_address 0xD2
#define mpu2_address 0xD0
#define mpu3_address 0xD0

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Private variables ---------------------------------------------------------*/
/*************Initializtions****************/

	MPU6050 MPU1;
	MPU6050 MPU2;
	MPU6050 MPU3;
	
	STEPPER Rotor;
	STEPPER Left_Right;
	STEPPER First_Arm;
	STEPPER Second_Arm;
	
	
struct __FILE{
	
int handle;
};

FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f)
{
	ITM_SendChar(ch);
	return(ch);
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_TIM8_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
	HAL_TIM_Encoder_Start_IT(&htim1,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start_IT(&htim2,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start_IT(&htim3,TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_Base_Start_IT(&htim9);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
	HAL_ADC_Start_IT(&hadc1);
	//HAL_TIM_Base_Start_IT(&htim6);
	HAL_UART_Receive_IT(&huart2, (uint8_t *)&uart_rx2 ,1 );
	HAL_UART_Receive_IT(&huart3, (uint8_t *)&uart_rx3 ,1 );
	TIM1->CNT = 11;
	encoder_reading_wheel = 11;
	encoder_reading_pre =11;
	direction_left_right =0 ;
	TIM3->CNT = 5000;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	Initialize_Steppers();
	Initialize_MPUs();
		//HAL_TIM_Base_Start_IT(&htim1);
		//TIM1->CNT = 0;
	//MPU_GYRO_CAL_Y(&MPU1);
	//Calibrate_Base();
	//Rotor.throttel = -10;			//For base calibration
  while (1)
  {
		//MPU_SHOW_DATA(&MPU1);
		//MPU_SHOW_DATA(&MPU2);
	//	MPU_SHOW_DATA(&MPU3);
	/*	Left_Right.throttel = 30;
		HAL_Delay(7000);
		Left_Right.throttel = -30;
		HAL_Delay(7000);*/
//		First_Arm.throttel = 20;
//		Second_Arm.throttel = 20;
		//set_rotor_angle(20);
		//MPU_GET_VALUE(&MPU1);
//		checker = MPU1.Angle;
//	PID_calculate(&MPU1,&MOTOR_1,setting);
		sprintf(tx_data,"Angle1: %d , Angle2: %d\r\n",MPU1.Angle , MPU2.Angle);
		HAL_UART_Transmit(&huart2,(uint8_t*)&tx_data,sizeof(tx_data),0xFFFF);
	//	HAL_UART_Receive_IT(&huart2, (uint8_t *)&uart_rx ,1 );
//		sprintf(tx_data,"Angle: %f , Setpoint: %d , error: %f , throttel: %d\r\n",MPU1.Angle , MOTOR_1.setpoint,MOTOR_1.pid_error , MOTOR_1.throttel);
//		HAL_UART_Transmit(&huart2,(uint8_t*)&tx_data,sizeof(tx_data),0xFFFF);
//		HAL_UART_Receive_IT(&huart2, (uint8_t *)&uart_rx ,1 );
		//my_angle = left_right_angle();
//		HAL_GPIO_WritePin(stepper_port,stepper1_dir,HIGH);
		//displacement = distance_travelled(encoder_reading_wheel);
	//set_angle(30,Right);
		/*
		if(HAL_GetTick() - x >= dt)
		{
			x = HAL_GetTick();
			omega = (6.2831 * TIM4->CNT*50)/4320;
			TIM4->CNT =0;
			printf("%f\n",omega);
		}
		*/
	//angle = left_right_angle();
	//  set_angle(10, Right);
	/*	if(rec == 0)
		{
			throttel_left = 0;
			throttel_right = 0;
			HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
			HAL_GPIO_WritePin(sig_port,sig1,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(sig_port,sig2, GPIO_PIN_RESET);
		}
		
		 else if(rec == 1)
		{
				ds  = 40;
				distance =  ds *fullcounter * 0.018181818182 ;
				move(distance, 20 ,Front);
				//HAL_Delay(500);
				//set_angle(-15,Left);
				//HAL_Delay(1000);
				//TIM4->CNT = 0;
				//encoder_reading_wheel = 0;
				//encoder_reading_pre =0;
				//move(3000, 20 ,Front);
		}
		
			
		else if(rec == 2)
		{
			HAL_GPIO_WritePin(sig_port,sig2,GPIO_PIN_SET);
			HAL_GPIO_WritePin(sig_port,sig1,GPIO_PIN_RESET);
			htim2.Instance->CCR1 = (int)((20 - forward_speed) * (2800 / 17));
			HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
		}	
		
		//turning left
		else if(rec == 3)
		{
			throttel_left =  30;
			throttel_right = 30;
			HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
		}
		else if(rec == 4)
		{
			throttel_left = -30;
			throttel_right = -30;
			HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
		}
		
		
		//Moving right with curvy step
		else if(rec == 6)
		{
			throttel_left = 60 ;
			throttel_right = 60; 
			HAL_GPIO_WritePin(sig_port,sig1,GPIO_PIN_SET);
			HAL_GPIO_WritePin(sig_port,sig2,GPIO_PIN_RESET);
			htim2.Instance->CCR1 = (int)((20 - forward_speed) * (2800 / 17));
			HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
		}
		//Moving left with curvy step
		else if(rec == 5)
		{
			throttel_left = -70 ;
			throttel_right = -70 ;
			HAL_GPIO_WritePin(sig_port,sig1,GPIO_PIN_SET);
			HAL_GPIO_WritePin(sig_port,sig2,GPIO_PIN_RESET);
			htim2.Instance->CCR1 = (int)((20 - forward_speed) * (2800 / 17));
			HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
			
			}
	*/
}
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM4)			//Stepper_Refresher(20us)
	{
		
		/******************ROTOR CALCULATIONS*****************/
		if(calibrated == CALIBRATING)
		{
			set_rotor_angle(setting);
			if(current_angle >=31)
			{
				current_angle = 0;
				setting = 0;
				current_encoder_reading = 0;
				calibrated = TRUE;
			}
		}
		else if (calibrated == TRUE)
			set_rotor_angle(setting);
		
		/******************ROTOR CALCULATIONS*****************/
		
	  Pulse_Width_Calculator(&Rotor);
		Pulse_Width_Calculator(&Left_Right);
		Pulse_Width_Calculator(&First_Arm);
		Pulse_Width_Calculator(&Second_Arm);
	}
	
	//PID For motor
	if(htim->Instance == TIM5)				//PID Refresher(1ms)
	{
		PID_calculate(&First_Arm,set_arm_first,my_angle1);
		PID_calculate(&Second_Arm,set_arm_second,my_angle2);
		
		/*******************WHEEL CALCULATIONS**************************/
		_pid = pid(ds, 1000, 0 );
	
		if (_pid > 0 )
		{
			HAL_GPIO_WritePin(sig_port,sig1,GPIO_PIN_SET);
			HAL_GPIO_WritePin(sig_port,sig2, GPIO_PIN_RESET);
			htim8.Instance->CCR3 = 450;
			HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
		}
		else if (_pid < 0 )
		{
			HAL_GPIO_WritePin(sig_port,sig1,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(sig_port,sig2, GPIO_PIN_SET);
			htim8.Instance->CCR3 = 450;
			HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
		}
		else
		{
			HAL_GPIO_WritePin(sig_port,sig1,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(sig_port,sig2, GPIO_PIN_RESET);
			HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_3);
		}		
		
		/*******************WHEEL CALCULATIONS**************************/
		
			/*******************LEFT_RIGHT CALCULATIONS**************************/
		//	set_angle(my_angle,NULL);
			/*******************LEFT_RIGHT CALCULATIONS**************************/
	}
	
	if(htim->Instance == TIM9)			//Angle Calculator(4 ms)
	{
	  MPU_SHOW_DATA(&MPU1);			//This will give raw data(CAution!!!!!)
		 MPU_SHOW_DATA(&MPU2);			//This will give raw data(CAution!!!!!)
     MPU1.Angle += MPU1.Gyroscope_Y*0.004;		//As we require angle in milisecond basis
		MPU2.Angle += MPU2.Gyroscope_Y/1000;		//As we require angle in milisecond basis
		 cal = sqrt(MPU1.Accelerometer_X*MPU1.Accelerometer_X+MPU1.Accelerometer_Y*MPU1.Accelerometer_Y+MPU1.Accelerometer_Z*MPU1.Accelerometer_Z);
		cal1 = sqrt(MPU2.Accelerometer_X*MPU2.Accelerometer_X+MPU2.Accelerometer_Y*MPU2.Accelerometer_Y+MPU2.Accelerometer_Z*MPU2.Accelerometer_Z);
     MPU1.Accel_Angle = asin((float)MPU1.Accelerometer_X/cal)*RAD_TO_DEG;
		 MPU2.Accel_Angle = asin((float)MPU2.Accelerometer_X/cal1)*RAD_TO_DEG;
 
		if(set_gyro_angle)
		{
	  MPU1.Angle = MPU1.Angle*0.96 +MPU1.Accel_Angle*0.04;
		MPU2.Angle = MPU2.Angle*0.96 +MPU2.Accel_Angle*0.04;
		}
		else
		{
			MPU1.Angle = MPU1.Accel_Angle;
	   	MPU2.Angle = MPU2.Accel_Angle;
			set_gyro_angle = 1;
		}
//		sprintf(string,"Angle ->%f\r\n",MPU1.Angle);
//	  HAL_UART_Transmit(&huart2,(uint8_t *)&string,sizeof(string),0xFFFF);
	}	
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	  while((HAL_ADC_PollForConversion(&hadc1, 10)) != HAL_OK);
		adc_value = 4095-HAL_ADC_GetValue(&hadc1);
		humid_percentage = adc_value/40.95;
	  HAL_ADC_Start_IT(&hadc1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
			receive2 = uart_rx2 - 48;
		  if(uart_rx2 == 'p' )				//angle1
			{
				if(receive_buffer2[0] == '-' - 48)
					checker = 1;
				else
					checker = 0;
					
				for(int i=checker;i<range;i++)
				{
					buff_sum = buff_sum*10 + receive_buffer2[i];
				}
				set_arm_first = buff_sum;
				if(receive_buffer2[0] == '-' - 48)
					set_arm_first = (-1)*set_arm_first;

				buff_sum =0;
				range =0;
			}
			
			 else if(uart_rx2 == 'q' )				//angle2
			{
				if(receive_buffer2[0] == '-' - 48)
					checker = 1;
				else
					checker = 0;
					
				for(int i=checker;i<range;i++)
				{
					buff_sum = buff_sum*10 + receive_buffer2[i];
				}
					set_arm_second = buff_sum;
				if(receive_buffer2[0] == '-' - 48)
					set_arm_second = (-1)*set_arm_second;

				buff_sum =0;
				range =0;
			}
			
			else  if(uart_rx2 == 'r' )				//rotor
			{
				if(receive_buffer2[0] == '-' - 48)
					checker = 1;
				else
					checker = 0;
					
				for(int i=checker;i<range;i++)
				{
					buff_sum = buff_sum*10 + receive_buffer2[i];
				}
				setting = buff_sum;
				if(receive_buffer2[0] == '-' - 48)
					setting = (-1)*setting;
				
				buff_sum =0;
				range =0;
			}
			
			else if(uart_rx2 == 'd' )				//distance
			{
				if(receive_buffer2[0] == '-' - 48)
					checker = 1;
				else
					checker = 0;
					
				for(int i=checker;i<range;i++)
				{
					buff_sum = buff_sum*10 + receive_buffer2[i];
				}
				ds = buff_sum;
				if(receive_buffer2[0] == '-' - 48)
					ds = (-1)*ds;

				buff_sum =0;
				range =0;
			}
			
			else if(uart_rx2 == 'l' )				//left_right
			{
				if(receive_buffer2[0] == '-' - 48)
					checker = 1;
				else
					checker = 0;
					
				for(int i=checker;i<range;i++)
				{
					buff_sum = buff_sum*10 + receive_buffer2[i];
				}
				my_angle = buff_sum;
				if(receive_buffer2[0] == '-' - 48)
					my_angle = (-1)*my_angle;

				buff_sum =0;
				range =0;
			}
			else
			{
			  receive_buffer2[range++] = receive2;
			}
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&uart_rx2 ,1 );
	}
	
	if(huart->Instance == USART3)
	{
			receive3 = uart_rx3 - 48;
		  if(uart_rx3 == 'm' )
			{
				if(receive_buffer3[0] == '-' - 48)
					checker = 1;
				else
					checker = 0;
					
				for(int i=checker;i<range;i++)
				{
					buff_sum = buff_sum*10 + receive_buffer3[i];
				}
				my_angle1 = buff_sum;
				if(receive_buffer3[0] == '-' - 48)
					my_angle1 = (-1)*my_angle1;

				buff_sum =0;
				range =0;
			}
			
			if(uart_rx3 == 'n' )
			{
				if(receive_buffer3[0] == '-' - 48)
					checker = 1;
				else
					checker = 0;
					
				for(int i=checker;i<range;i++)
				{
					buff_sum = buff_sum*10 + receive_buffer3[i];
				}
				my_angle2 = buff_sum;
				if(receive_buffer3[0] == '-' - 48)
					my_angle2 = (-1)*my_angle2;
				
				buff_sum =0;
				range =0;
			}
			else
			{
				//if((receive3!= '.' - 48)&&(receive3!= '0' - 48)&&(receive3!= '-' - 48))
			  receive_buffer3[range++] = receive3;
			}

				HAL_UART_Receive_IT(&huart3, (uint8_t *)&uart_rx3 ,1 );
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
