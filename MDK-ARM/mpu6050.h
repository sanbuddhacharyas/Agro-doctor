#ifndef MPU6050_H_
#define MPU6050_H_

#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <inttypes.h>
#include "MPU_RES_defines.h"

/**
 * @brief  Parameters for accelerometer range
 */
typedef enum {
	TM_MPU6050_Accelerometer_2G = 0x00, /*!< Range is +- 2G */
	TM_MPU6050_Accelerometer_4G = 0x01, /*!< Range is +- 4G */
	TM_MPU6050_Accelerometer_8G = 0x02, /*!< Range is +- 8G */
	TM_MPU6050_Accelerometer_16G = 0x03 /*!< Range is +- 16G */
} TM_MPU6050_Accelerometer_t;

/**
 * @brief  Parameters for gyroscope range
 */
typedef enum {
	TM_MPU6050_Gyroscope_250s = 0x00,  /*!< Range is +- 250 degrees/s */
	TM_MPU6050_Gyroscope_500s = 0x01,  /*!< Range is +- 500 degrees/s */
	TM_MPU6050_Gyroscope_1000s = 0x02, /*!< Range is +- 1000 degrees/s */
	TM_MPU6050_Gyroscope_2000s = 0x03  /*!< Range is +- 2000 degrees/s */
} TM_MPU6050_Gyroscope_t;

/**
 * @brief  Main MPU6050 structure
 */

typedef struct {
	/* Private */
	uint8_t Address;         /*!< I2C address of device. Only for private use */
	I2C_HandleTypeDef* I2C_Port;
	/* Public */
	float Accelerometer_X; /*!< Accelerometer value X axis */
	float Accelerometer_Y; /*!< Accelerometer value Y axis */
	float Accelerometer_Z; /*!< Accelerometer value Z axis */
	float Gyroscope_X;     /*!< Gyroscope value X axis */
	float Gyroscope_Y;     /*!< Gyroscope value Y axis */
	float Gyro_Cal_Y;
	float Gyroscope_Z;     /*!< Gyroscope value Z axis */
	float Temperature;       /*!< Temperature in degrees */
	float Accel_Angle;
	int Angle;
} MPU6050;

void MPU6050_Initialize(MPU6050* Datastruct);
void MPU_GET_VALUE(MPU6050* Datastruct);
void MPU_SHOW_DATA(MPU6050* Datastruct);
void MPU_GYRO_CAL_Y(MPU6050* Datastruct);
void Initialize_MPUs(void);

//MPU6050 MPU1 = {mpu2_address , &hi2c2};
//MPU6050 MPU2 = {mpu2_address , &hi2c2};

#endif

