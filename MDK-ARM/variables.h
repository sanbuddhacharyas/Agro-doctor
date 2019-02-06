	#ifndef VARIABLES_H_
	#define VARIABLES_H_
	
	#include <inttypes.h>
	
	int throttel_left_counter=0 , throttle_counter_right_motor , throttel_previous_memory , throttel_left,throttel_right,left_motor,right_motor,throttle_right_motor_memory;
  volatile int forward_speed = 16;
	int _pid , turning_pid;
	int buffer[30];
	uint8_t uart_rx2 , uart_rx3;
	long int uart_buffer2 , uart_buffer3;
	int speed_counter=0;
	volatile uint16_t encoder_reading_pre =0;
	volatile int total_distance ;
	 float p_scalar =15, i_scalar = 0, d_scalar=40;

	int receive_buffer2[15] ={0};
	int receive_buffer3[15] ={0};
	int receive2 =0 , receive3 = 0 ,rec=0;
	int range =0;
	int buff_sum=0;
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
	float humid_percentage = 0;
	int ds = 0 ,my_angle1 = 0 , my_angle2 = 0 ,my_angle = 0 ,temp_angle = 0 , set_arm_first = 55 , set_arm_second = 0 , adc_value = 0;
	char str[30];
	char tx_data[100];
	int checker = 0;
	int setting =0;
	extern float current_angle;
	extern int calibrated;
	extern int current_encoder_reading;
	#define TRUE 1
	
	#define CALIBRATING 3
	
#endif
