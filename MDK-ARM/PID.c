
#include "PID.h"

/*void PID_calculate(float angle_gyr,int pid_setpoint=0)
	{
		pid_error = angle_gyr-pid_setpoint;
		float proportional = pid_error * p_scalar;
		static float integral = 0;
		integral += pid_error * i_scalar;
		if(integral >  400) integral = 400; // limit wind-up
		if(integral < -400) integral = -400;
		static float previous_error = 0;
		float derivative = (pid_error - previous_error) * d_scalar;
		previous_error = pid_error;
		pid = proportional+derivative+integral;
		if(pid > 400) pid = 400;
		if(pid < -400)pid = -400;
	    if(pid <5 && pid>-5) pid =0;                                   //Create a dead-band to stop the motors when the robot is balanced
		if(pid > 0)pid = 405 - (1/(pid + 9)) * 5500;
		else if(pid<0) pid= -405 - (1/(pid- 9)) * 5500;
		if(pid > 0)motor = 400 - pid;                                  //Calculate the needed pulse time for `stepper motor controllers
		else if(pid < 0)motor = -400 - pid;
		else motor = 0;
		throttel = motor;
	}
	
	
	{
		throttel_counter ++;                                           //Increase the throttel_left_counter variable by 1 every time this routine is executed
		if(throttel_counter > throttel_previous_memory)
		{                                                              //If the number of loops is larger then the throttel_previous_memory variable
			throttel_counter = 0;                                      //Reset the throttel_left_counter variable
			throttel_previous_memory = throttel;                       //Load the next throttle_left_motor variable
			if(throttel_previous_memory < 0){                          //If the throttel_previous_memory is negative
				PORTC &= (~(1<<port_dir));                             //Set output 3 low to reverse the direction of the stepper controller
				throttel_previous_memory *= -1;                        //Invert the throttel_previous_memory variable
			}
			else PORTC |= (1<<port_dir);                               //Set output 3 high for a forward direction of the stepper motor
		}
		
		else if(throttel_counter == 1)PORTC |= (1<<port);             //Set output 2 high to create a pulse for the stepper controller
		
		else if(throttel_counter == 2)PORTC &= (~(1<<port));
		
	}*/