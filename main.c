#define F_CPU 16000000UL
#define STAGEAMOUNT 32 //amount of stages. For now this number is arbitrary
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "usart.h"
#include "motorboard.h"
#include "potentiometer.h"

#define BAUDRATE 57600 //baudrate number is also arbitrary for now
#define BAUD_PRESCALER ((F_CPU/(BAUDRATE*16UL))-1)

typedef struct
{
	uint8_t R1; //right arm bottom actuator
	uint8_t R2; //right arm middle actuator
	uint8_t R3; //right arm top actuator
	uint8_t L1; //left arm ...
	uint8_t L2;
	uint8_t L3;
} pwm;

//function prototypes
void motor_set (unsigned char); //sets motor pwm.
uint8_t optocoupler_check (void); //checks if optocoupler signal is 1 or 0. (might need to be an interrupt instead of function).
char value_check (unsigned char); //PID controller. checks angle, velocity, etc. If needed values are achieved, the device starts grabbing bar.

pwm motor; //this variable is used for setting pwm values for each of the 6 motors on the machine
uint8_t angle; //this variable is used by accelerometer to check whether the needed angle is reached
volatile int ms=0;

int main(void)
{
	uart_init();
	io_redirect();
	//PWM pins set as outputs
	DDRB |= (1 << DDB1) | (1 << DDB2) | (1 << DDB3);
	DDRD |= (1 << DDD3) | (1 << DDD5) | (1 << DDD6);

	// Latch, Clock and Data pins for 74HC595 set as outputs
	DDRD |= (1 << DDD2) | (1 << DDD4) | (1 << DDD7);
	
	char finish=1; //when finish =0, program stops.
	char stage_number=1; //which stage device is currently on. Used to determine what pwm speed to set.
	
	// COUNTER (1 ms)
	TCCR0A|=(1<<WGM01);	 // set timer to ctc
	OCR0A=0xF9;			 // set value
	TIMSK0|=(1<<OCIE0A); // enable interrupt on compare a for timer 0
	sei();
	
/*		loop for bars
		1. Set pwm signal and the signals duration
		2. When expected values (angle, velocity, acceleration) are correct, stage_number++
		3. Loop starts again but with stage_number=+1, therefore next stage begins.
		4. Second stage, gripper motors activate. Machine grips bar.
		6. When Opto-coupler and switch =1, gripper caught bar. Stage complete
		7. Begin next stage
*/		
	while(finish){
		do 
		{
				motor_set(stage_number);
				while(value_check(stage_number)==0){ //either this or time-based command.
					//wait to achieve the needed values
					
				stage_number++;
			}
		} while (stage_number<STAGEAMOUNT); // robot reached the last ladder and finished all stages
			finish=0;
		}
		
	return (0);
}


	
void motor_set (unsigned char input_number){

	switch (input_number)
	{
		case 1:{
			motor.L1=1; //requested velocity for each motor in set stage is input into these values
			motor.L2=2;
			motor.L3=2;
			motor.R1=2;
			motor.R2=2;
			motor.R3=2;
			
			angle=43; //requested angle for machine in set stage is set
			
		}
		case 2:{}
	// and so on for all needed stages.
	
	//Using a PWM command, input the chosen motorL1,L2,... values
	
	}
		

}

char value_check (unsigned char input_number){
	char done=0;
	
	 return done;
}

ISR(TIMER0_COMPA_vect){
	ms++;
}
