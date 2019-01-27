
#include "PID.h"


void PID_calculate(MPU6050* Datastruct,STEPPER* INFO ,int pid_setpoint)
	{
		INFO->setpoint = pid_setpoint;
		INFO->pid_error = (Datastruct->Angle)-pid_setpoint;
	  INFO->proportional = INFO->pid_error * INFO->p_scalar;
		//INFO->integral = 0;
		INFO->integral += INFO->pid_error * INFO->i_scalar;
		if(INFO->integral >  400) INFO->integral = 400; // limit wind-up
		if(INFO->integral < -400) INFO->integral = -400;
		INFO->derivative = (INFO->pid_error - INFO->previous_error) * INFO->d_scalar;
		INFO->previous_error = INFO->pid_error;
		INFO->pid = INFO->proportional+INFO->derivative+INFO->integral;
		if(INFO->pid > 400) INFO->pid = 400;
		if(INFO->pid < -400)INFO->pid = -400;
	    if(INFO->pid <10 && INFO->pid>-10) INFO->pid =0;                                   //Create a dead-band to stop the motors when the robot is balanced
		if(INFO->pid > 0)INFO->pid = 405 - (1/(INFO->pid + 9)) * 5500;
		else if(INFO->pid<0) INFO->pid= -405 - (1/(INFO->pid- 9)) * 5500;
		if(INFO->pid > 0)INFO->motor = 400 - INFO->pid;                                  //Calculate the needed pulse time for `stepper motor controllers
		else if(INFO->pid < 0)INFO->motor = -400 - INFO->pid;
		else INFO->motor = 0;
		INFO->throttel = INFO->motor;
	}
	
void Pulse_Width_Calculator(STEPPER* INFO)
	{
		INFO->throttel_counter ++;                                           //Increase the throttel_left_counter variable by 1 every time this routine is executed
		if(INFO->throttel_counter > INFO->throttel_previous_memory)
		{                                                              //If the number of loops is larger then the throttel_previous_memory variable
			INFO->throttel_counter = 0;                                      //Reset the throttel_left_counter variable
			INFO->throttel_previous_memory = INFO->throttel;                       //Load the next throttle_left_motor variable
			if(INFO->throttel_previous_memory < 0){                          //If the throttel_previous_memory is negative
				HAL_GPIO_WritePin(stepper_port, INFO->Direction , LOW);                            //Set output 3 low to reverse the direction of the stepper controller
				INFO->throttel_previous_memory *= -1;                        //Invert the throttel_previous_memory variable
			}
			else HAL_GPIO_WritePin(stepper_port, INFO->Direction, HIGH);                              //Set output 3 high for a forward direction of the stepper motor
		}
		
		else if(INFO->throttel_counter == 1)HAL_GPIO_WritePin(stepper_port, INFO->Signal, HIGH);            //Set output 2 high to create a pulse for the stepper controller
		
		else if(INFO->throttel_counter == 2)HAL_GPIO_WritePin(stepper_port, INFO->Signal , LOW); 
		
	}
		