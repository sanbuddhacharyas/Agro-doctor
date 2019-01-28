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
#include "main.h"
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "STEPPER.h"
#include "string.h"
#include "mpu6050.h"
#include "PID.h"

#define RAD_TO_DEG 57.295779513082320876798154814105
#define PI 3.1415926535897932384626433832795

#define mpu1_address 0xD2
#define mpu2_address 0xD0
#define mpu3_address 0xD0

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU6050 MPU1 = {mpu2_address , &hi2c2};
//MPU6050 MPU2 = {mpu2_address , &hi2c2};
STEPPER MOTOR_1 = {stepper1_sig,stepper1_dir};
STEPPER MOTOR_2 = {stepper2_sig,stepper2_dir};
/* Private variables ---------------------------------------------------------*/

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
	int throttel_left_counter=0 , throttle_counter_right_motor , throttel_previous_memory , throttel_left,throttel_right,left_motor,right_motor,throttle_right_motor_memory;
  volatile int forward_speed = 16;
	int _pid , turning_pid;
	int buffer[30];
	uint8_t uart_rx;
	long int uart_buffer;
	int speed_counter=0;
	volatile uint16_t encoder_reading_pre =0;
	volatile int total_distance ;
	 float p_scalar =15, i_scalar = 0, d_scalar=40;

	int receive_buffer[15] ={0};
	int receive =0 ,rec=0;
	int range =0;
	int buff_sum=0;
	uint8_t data;
	uint32_t x =0;
	uint8_t dt =5;
	extern double cal,cal1;
	extern int set_gyro_angle;
	
	char d[30] = "hello \r\n";
	
	volatile int32_t encoder_reading_wheel =0;
	volatile uint32_t encoder_reading_left_right =0;
	volatile float velocity= 0;
	volatile uint8_t direction_wheel;
	volatile uint8_t direction_left_right;
	char encoder_buffer[20];
	uint32_t distance = 0;
	uint32_t encoder_wheel_state=0;
	uint32_t reading_pre=0;
	float angle;
	int ds = 0 ,my_angle = 0 ;
	char str[30];
	char tx_data[100];
	float checker;
	int setting = 0;
	extern float current_angle;
	extern int calibrated;
	
	#define TRUE 1
	
	

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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  MX_I2C2_Init();

  /* USER CODE BEGIN 2 */
	HAL_TIM_Encoder_Start_IT(&htim4,TIM_CHANNEL_ALL);
	//HAL_TIM_Encoder_Start_IT(&htim5,TIM_CHANNEL_ALL);
	//HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim6);
//	HAL_UART_Receive_IT(&huart2, (uint8_t *)&uart_rx ,1 );
//	TIM4->CNT = 11;
//	encoder_reading_wheel = 11;
//	encoder_reading_pre =11;
//	direction_left_right =0 ;
	TIM4->CNT = 5000;
	
	MOTOR_1.p_scalar = 30;
	MOTOR_1.i_scalar = 0;
	MOTOR_1.d_scalar = 10;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_Delay(1000);
	//MPU6050_Initialize(&MPU1);
		//HAL_TIM_Base_Start_IT(&htim1);
		//TIM1->CNT = 0;
	//MPU_GYRO_CAL_Y(&MPU1);
	//Calibrate_Base();
	throttel_left = -10;
  while (1)
  {
		
			if(calibrated == TRUE)
		{
			set_rotor_angle(setting);
		}
		//set_rotor_angle(20);
		//MPU_GET_VALUE(&MPU1);
//		checker = MPU1.Angle;
//	PID_calculate(&MPU1,&MOTOR_1,setting);
		//sprintf(tx_data,"Angle: %f ,, throttel_left: %d\r\n",current_angle , throttel_left);
		//HAL_UART_Transmit(&huart2,(uint8_t*)&tx_data,sizeof(tx_data),0xFFFF);
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
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
}
  /* USER CODE END 3 */

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
	if(htim->Instance == TIM3)
	{
		//HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14);
			
		
		throttel_left_counter ++;                                        //Increase the throttel_left_counter variable by 1 every time this routine is executed
		if(throttel_left_counter > throttel_previous_memory )
		{             //If the number of loops is larger then the throttel_previous_memory variable
				
			throttel_left_counter = 0; 
			throttel_previous_memory = throttel_left;			//Reset the throttel_left_counter variable                     //Load the next throttle_left_motor variable
				if(throttel_previous_memory < 0){                                     //If the throttel_previous_memory is negative  
					HAL_GPIO_WritePin(stepper_port,  stepper1_dir , LOW);
					throttel_previous_memory *= -1;//Invert the throttel_previous_memory variable
				} 
				else HAL_GPIO_WritePin(stepper_port,  stepper1_dir , HIGH);                                    //Set output 3 high for a forward direction of the stepper motor
		}
		else if(throttel_left_counter == 1)HAL_GPIO_WritePin(stepper_port,  stepper1_sig , HIGH);             //Set output 2 high to create a pulse for the stepper controller
		else if(throttel_left_counter == 2)HAL_GPIO_WritePin(stepper_port,  stepper1_sig, LOW);
		
		throttle_counter_right_motor ++;                                          //Increase the throttle_counter_right_motor variable by 1 every time the routine is executed
		if(throttle_counter_right_motor > throttle_right_motor_memory)
		{           	//If the number of loops is larger then the throttle_right_motor_memory variable
				throttle_counter_right_motor = 0;  
				throttle_right_motor_memory	 =throttel_right;		//Reset the throttle_counter_right_motor variable
				if(throttle_right_motor_memory < 0){   
																												//If the throttle_right_motor_memory is negative
					HAL_GPIO_WritePin(stepper_port,  stepper2_dir , LOW);	//Set output 5 low to reverse the direction of the stepper controller
					throttle_right_motor_memory *= -1;																									//Invert the throttle_right_motor_memory variable
				}
				else HAL_GPIO_WritePin(stepper_port,  stepper2_dir , HIGH);                                                          //Set output 5 high for a forward direction of the stepper motor
		}
		else if(throttle_counter_right_motor == 1)HAL_GPIO_WritePin(stepper_port,  stepper2_sig , HIGH);             //Set output 4 high to create a pulse for the stepper controller
		else if(throttle_counter_right_motor == 2)HAL_GPIO_WritePin(stepper_port,  stepper2_sig , LOW);           //Set output 4 low because the pulse only has to last for 20us
		
		//Pulse_Width_Calculator(&MOTOR_1);
	}

	//PID For motor
	if(htim->Instance == TIM6)
	{
		//	set_rotor_angle(5);
		/*_pid = pid(ds, 1000, 0 );
	
		if (_pid > 0 )
		{
			HAL_GPIO_WritePin(sig_port,sig1,GPIO_PIN_SET);
			HAL_GPIO_WritePin(sig_port,sig2, GPIO_PIN_RESET);
			htim2.Instance->CCR1 = 600;
			HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
		}
		else if (_pid < 0 )
		{
			HAL_GPIO_WritePin(sig_port,sig1,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(sig_port,sig2, GPIO_PIN_SET);
			htim2.Instance->CCR1 = 600;
			HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
		}
		else
		{
			HAL_GPIO_WritePin(sig_port,sig1,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(sig_port,sig2, GPIO_PIN_RESET);
			HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
			
		}		*/
		//set_angle(my_angle,NULL);
		
	}
	
	if(htim->Instance == TIM1)
	{
	  MPU_SHOW_DATA(&MPU1);			//This will give raw data(CAution!!!!!)
		 //MPU_SHOW_DATA(&MPU2);			//This will give raw data(CAution!!!!!)
     MPU1.Angle += MPU1.Gyroscope_Y*0.004;		//As we require angle in milisecond basis
		//MPU2.Angle += MPU2.Gyroscope_Y/1000;		//As we require angle in milisecond basis
		 cal = sqrt(MPU1.Accelerometer_X*MPU1.Accelerometer_X+MPU1.Accelerometer_Y*MPU1.Accelerometer_Y+MPU1.Accelerometer_Z*MPU1.Accelerometer_Z);
		//cal1 = sqrt(MPU2.Accelerometer_X*MPU2.Accelerometer_X+MPU2.Accelerometer_Y*MPU2.Accelerometer_Y+MPU2.Accelerometer_Z*MPU2.Accelerometer_Z);
     MPU1.Accel_Angle = asin((float)MPU1.Accelerometer_X/cal)*RAD_TO_DEG;
		 //MPU2.Accel_Angle = asin((float)MPU2.Accelerometer_X/cal1)*RAD_TO_DEG;

		if(set_gyro_angle)
		{
	  MPU1.Angle = MPU1.Angle*0.96 +MPU1.Accel_Angle*0.04;
		//MPU2.Angle = MPU2.Angle*0.96 +MPU2.Accel_Angle*0.04;
		}
		else
		{
			MPU1.Angle = MPU1.Accel_Angle;
		//	MPU2.Angle = MPU2.Accel_Angle;
			set_gyro_angle = 1;
		}
//		sprintf(string,"Angle ->%f\r\n",MPU1.Angle);
//	  HAL_UART_Transmit(&huart2,(uint8_t *)&string,sizeof(string),0xFFFF);
	}		
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	
	if(huart->Instance == USART2)
	{
		
			receive = uart_rx - 48;
			if(receive == 1)
			{
				p_scalar += 0.1;
				
			}
			else if(receive == 2)
			{
				p_scalar = p_scalar - 0.1;
			}
			else if(receive == 5)
			{
				d_scalar += 0.1;
			}
			
			else if(receive == 6)
				d_scalar -= 0.1;
		
		  if(uart_rx == 's' )
			{
				for(int i=0;i<range;i++)
				{
					buff_sum = buff_sum*10 + receive_buffer[i];
					
				}
				forward_speed = buff_sum;
				//speed_memory= 100;

				buff_sum =0;
				range =0;
			}
			
			 else if(uart_rx == 'f' )
			{
				for(int i=0;i<range;i++)
				{
					buff_sum = buff_sum*10 + receive_buffer[i];
				}
				ds = buff_sum;

				buff_sum =0;
				range =0;
			}
			
			 else if(uart_rx == 'x' )
			{
				for(int i=0;i<range;i++)
				{
					buff_sum = buff_sum*10 + receive_buffer[i];
					
				}
				setting = buff_sum;

				buff_sum =0;
				range =0;
			}
			
			 else if(uart_rx == 'y' )
			{
				for(int i=0;i<range;i++)
				{
					buff_sum = buff_sum*10 + receive_buffer[i];
					
				}
				my_angle = (-1) * buff_sum;

				buff_sum =0;
				range =0;
			}
			
		  else if(uart_rx == '.')
			{
				for(int i=0;i<range;i++)
				{
					buff_sum = buff_sum*10 + receive_buffer[i];
					
				}
				rec = buff_sum;
				range =0;
				buff_sum =0;
			}
			else if(uart_rx == 'u')
			{
				rec = uart_rx;
			}
			else if(uart_rx == 'd')
			{
				rec = uart_rx;
			}
			
			else if(uart_rx == 'e')
			{
					rec = uart_rx;
			
			}
			
			
			else
			{
			  receive_buffer[range++] = receive;
			}

				HAL_UART_Receive_IT(&huart2, (uint8_t *)&uart_rx ,1 );
	
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
