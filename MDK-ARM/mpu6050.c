#include "mpu6050.h"

#define mpu1_address 0xD0  
#define mpu2_address 0xD2    
#define mpu3_address 0xD0  

float gyro_x,gyro_y,gyro_z,accel_x=0,accel_y=0,accel_z=0,temp,mag_x, mag_y ,mag_z;
long int gyro_cal_y;
float asax , asay ,asaz; //Initialing sensitivity adjustment values
float Xa,Ya,Za,ts;
float Xg=0,Yg=0,Zg=0;
double cal,cal1;
float angle_gyro=0,del_x;
int set_gyro_angle = 0;
char string[100];

extern MPU6050 MPU1;
extern MPU6050 MPU2;
extern MPU6050 MPU3;
	
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
	

#define RAD_TO_DEG 57.295779513082320876798154814105
#define PI 3.1415926535897932384626433832795

//void MPU6050_Initialize(I2C_HandleTypeDef* I2Cx,MPU6050* Datastruct)
void MPU6050_Initialize(MPU6050* Datastruct)
{
	uint8_t temp;
	char string[50] ;
	uint8_t buffer[2]={SMPLRT_DIV,0x07};//Gyroscope output value is 8KHZ so sample rate is 1KHz
	
  while(HAL_I2C_Master_Transmit(&hi2c1,Datastruct->Address,(uint8_t *)&buffer,2,0xFFFF) != HAL_OK)
	{
		sprintf(string,"Initializing Sample_Rate Division\r\n");
		HAL_UART_Transmit(&huart2,(uint8_t *)&string,sizeof(string),0xFFFF);
	}
	
	
	buffer[0] = PWR_MGMT_1;//Power Managment system
	buffer[1] = 0x00;//Internal clock of 8MHz oscillator
 while(HAL_I2C_Master_Transmit(&hi2c1,Datastruct->Address,(uint8_t *)&buffer,2,0xFFFF) != HAL_OK)
	{
		sprintf(string,"Initializing Power Mgmt\r\n");
		HAL_UART_Transmit(&huart2,(uint8_t *)&string,sizeof(string),0xFFFF);
	}
	
	//Configure
	buffer[0] = CONFIG;
	buffer[1] = 0x00;//Digital low pass filter Gyro = 8kHz
 while(HAL_I2C_Master_Transmit(&hi2c1,Datastruct->Address,(uint8_t *)&buffer,2,0xFFFF) != HAL_OK)
	{
		sprintf(string,"Initializing Config\r\n");
		HAL_UART_Transmit(&huart2,(uint8_t *)&string,sizeof(string),0xFFFF);
	}

	//Configure Gyro
	buffer[0] = GYRO_CONFIG;
	while(HAL_I2C_Master_Transmit(&hi2c1,Datastruct->Address,(uint8_t *)&buffer,1,0xFFFF) != HAL_OK);
	HAL_I2C_Master_Receive(&hi2c1,Datastruct->Address,(uint8_t *)&temp,1,0xFFFF);
	temp=(temp&0xE7)|0x00;
	buffer[0] = 0x1B;
	buffer[1] = temp; // 
while(HAL_I2C_Master_Transmit(&hi2c1,Datastruct->Address,(uint8_t *)&buffer,2,0xFFFF) != HAL_OK)
	{
		sprintf(string,"Initializing Gyroscope\r\n");
		HAL_UART_Transmit(&huart2,(uint8_t *)&string,sizeof(string),0xFFFF);
	}
	
	//Configure Accelerometer 
	buffer[0] = ACCEL_CONFIG;
	while(HAL_I2C_Master_Transmit(&hi2c1,Datastruct->Address,(uint8_t *)&buffer,1,0xFFFF) != HAL_OK);
	HAL_I2C_Master_Receive(&hi2c1,Datastruct->Address,(uint8_t *)&temp,1,0xFFFF);
	temp=(temp&0xE7)|0x00;
	buffer[0] = 0x1C;
	buffer[1] = temp;	
	while(HAL_I2C_Master_Transmit(&hi2c1,Datastruct->Address,(uint8_t *)&buffer,2,0xFFFF) != HAL_OK)
	{
		sprintf(string,"Initializing Accelerometer\r\n");
		HAL_UART_Transmit(&huart2,(uint8_t *)&string,sizeof(string),0xFFFF);
	}
	//Interrupt Enable
	buffer[0] = INT_ENABLE;
	buffer[1] = 0x01;
while(HAL_I2C_Master_Transmit(&hi2c1,Datastruct->Address,(uint8_t *)&buffer,2,0xFFFF) != HAL_OK)
	{
		sprintf(string,"Initializing INT Enable\r\n");
		HAL_UART_Transmit(&huart2,(uint8_t *)&string,sizeof(string),0xFFFF);
	}
}

void MPU_GET_VALUE(MPU6050* Datastruct)
{
	uint8_t data[14];
	uint8_t buffer=ACCEL_XOUT_H;
	HAL_I2C_Master_Transmit(&hi2c1,Datastruct->Address,(uint8_t *)&buffer,1,0xFFFF);
	//HAL_Delay(20);
	HAL_I2C_Master_Receive(&hi2c1,Datastruct->Address | 0x01 ,(uint8_t *)&data,14,0xFFFF);
	Datastruct->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);//2`s complements value to signed value value
	Datastruct->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
	Datastruct->Accelerometer_Z =(int16_t)(data[4] << 8 | data[5]);
	Datastruct->Temperature = (int16_t)((data[6]<<8)|(data[7]));
	Datastruct->Gyroscope_X = (int16_t)(data[8] << 8 | data[9]);
	Datastruct->Gyroscope_Y = (int16_t)(data[10] << 8 | data[11]);
	//gyro_y = gyro_y - gyro_cal_y;
	Datastruct->Gyroscope_Z  = ((int16_t)(data[12] << 8 | data[13]));
	
	Datastruct->Accelerometer_X = Datastruct->Accelerometer_X/16384;								/* Divide raw value by sensitivity scale factor to get real values */
	Datastruct->Accelerometer_Y = Datastruct->Accelerometer_Y/16384;
	Datastruct->Accelerometer_Z = Datastruct->Accelerometer_Z/16384;
	 
	Datastruct->Gyroscope_X =  Datastruct->Gyroscope_X/131;
	Datastruct->Gyroscope_Y = ( Datastruct->Gyroscope_Y ) /131;
	Datastruct->Gyroscope_Z =  Datastruct->Gyroscope_Z/131;
}

void MPU_GYRO_CAL_Y(MPU6050* Datastruct)
{
	for(int i=0; i<0xFFFF;i++)
	{
		MPU_GET_VALUE(Datastruct);
		Datastruct->Gyro_Cal_Y += Datastruct->Gyroscope_Y;
	}
	Datastruct->Gyro_Cal_Y/=0xFFFF;
}


void MPU_SHOW_DATA(MPU6050* Datastruct)
{
     MPU_GET_VALUE(Datastruct);
	
//		sprintf(string,"Accelerometer_X = %f , Accelerometer_Y = %f , Accelerometer_Z = %f\r\n",
//		Datastruct->Accelerometer_X,Datastruct->Accelerometer_Y,Datastruct->Accelerometer_Z);
//	  HAL_UART_Transmit(&huart2,(uint8_t *)&string,sizeof(string),0xFFFF);
	
	//		sprintf(string,"Gyroscope_X = %f , Gyroscope_Y = %f , Gyroscope_Z = %f\r\n",
//		Datastruct->Gyroscope_X,Datastruct->Gyroscope_Y,Datastruct->Gyroscope_Z);
//	  HAL_UART_Transmit(&huart2,(uint8_t *)&string,sizeof(string),0xFFFF);
}

void Initialize_MPUs(void)
{
	MPU1.Address = mpu1_address;
	MPU2.Address = mpu2_address;
	MPU3.Address = mpu3_address;
	
	MPU1.I2C_Port = &hi2c1;
	MPU1.I2C_Port = &hi2c1;
	MPU3.I2C_Port = &hi2c1;
	
	MPU6050_Initialize(&MPU1);
	//MPU6050_Initialize(&MPU2);
	//MPU6050_Initialize(&MPU3);
}

