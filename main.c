#define F_CPU 16000000UL
#define STAGEAMOUNT 32 //amount of stages. For now this number is arbitrary
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include "usart.h"
#include "potentiometer.h"
#define BAUDRATE 57600 //baudrate number is also arbitrary for now
#define BAUD_PRESCALER ((F_CPU/(BAUDRATE*16UL))-1)

volatile unsigned int ms_counter; //value for interrupts
float elapsedtime=0; //value for main function. It equals time that has passed.

//function prototypes
void pwm_set (uint8_t); //sets motor pwm.
uint8_t optocoupler_check (void); //checks if optocoupler signal is 1 or 0. (might need to be an interrupt instead of function).
char check_values (char); //checks angle, velocity, etc. If needed values are achieved, the device starts grabbing bar.
void init_interrupt(); //initiates interrupts


typedef struct 
{
	uint8_t motorR1;
	uint8_t motorR2;
	uint8_t motorR3;
	uint8_t motorL1;
	uint8_t motorL2;
	uint8_t motorL3;
} pwm;

pwm motor; //this variable is used for setting pwm values for each of the 6 motors on the machine


int main(void)
{
	char finish=1; //when finish =0, program stops.
	uint8_t stage_number=1; //which stage device is currently on. Used to determine what pwm speed to set.
	
	init_interrupt();
/*		loop for bars
		1. Set pwm signal for first stage, motors start moving
		2. When expected values (angle, velocity, acceleration) are correct, stage_number++
		3. Loop starts again but with stage_number=+1, therefore next stage begins 
		4. When robot reaches last stage, therefore reaches the last ladder bar, the program ends */
	while(finish){
		do 
		{
				pwm_set(stage_number);
				while(check_values(stage_number)==0){ //either this or time-based command.
					//wait to achieve the needed values
					
				stage_number++;
			}
		} while (stage_number<STAGEAMOUNT); // robot reached the last ladder and finished all stages
			finish=0;
		}
		
	return (0);
}

void init_interrupt(void){
	//set the timer mode to CTC <- count to desired value and then restart and count again
	TCCR0A |= (1 << WGM01);
	//set the value that you want to count to <- 250 timer ticks (0-249)
	OCR0A = 0xF9;
	
	//enable the interrupt for on compare a for timer 0
	TIMSK0 |= (1 << OCIE0A); //can be interrupted by Compare A matrch
	//enable all interrupts
	sei();
	
	//start the timer
	TCCR0B |= (1 << CS01) | (1 << CS00); //set prescaler to 64 and start
}

ISR (TIMER0_COMPA_vect) {
	ms_counter++;
	if (ms_counter == 250){
		ms_counter=0;
		elapsedtime=+0.25;
	}
	
}

void pwm_set (uint8_t input_number){

	switch (input_number)
	{
		case 1:{
			motor.motorL1=1; //i wrote random values here but once we figure out the needed velocity, we'll insert a pwm value needed to get that.
			motor.motorL2=2;
			motor.motorL3=2;
			motor.motorR1=2;
			motor.motorR2=2;
			motor.motorR3=2;
		}
		case 2:{}
	// and so on for all needed stages.
	
	//Using a PWM command, input the chosen motorL1,L2,... values
	
	}
		

}

char check_values (char input_number){
	uint8_t done=0;
	 //code that checks the input values. Returns 1 when done.
	 return done;
}