/*
 * Main_code.c
 *
 * Created: 31/05/2021 23.24.01
 * Author : Grp7_EMB_TEAM
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <util/delay.h>
#include <stdbool.h>
#include "sensorscontrol.h"

#define CW 1
#define CCW 2
#define BRAKE 3
#define R1 1
#define R2 2
#define R3 3
#define L1 4
#define L2 5
#define L3 6
#define P1 0
#define P2 1
#define P3 2
#define P4 3
#define OR 1
#define OL 2
#define MSR 1
#define MSL 2

void set_pins(void);
void init_interrupt(); //function initiates 
volatile int ms=0;
float second_counter=0;

int main(void)
{
	set_pins();
	uart_init();   // open the communication to the micro controller
	i2c_init();    // initialize the i2c communication.
	io_redirect(); // redirect the input/output to the computer.
	// COUNTER
	init_interrupt();
	
    while (1) 
    {
		//start-up -> claws are open
		motor_direction(R3, BRAKE);
		motor_direction(L3, BRAKE);
		
		/*M5 M6 -> 255
		Clockwise
		function if microswitch = 0 then BRAKE and set pwm signal to zero
		
		M4 needs CCW 20 degrees from 180 degrees 2670 mV to get the right degree
		M3 needs CW 20 degrees from 180 degrees (5V = 300 degrees) -> 3333 mV 
		
		M2 needs CW 20 degrees 
		M1 needs CCW 20 degrees 
		//create a loop where motor 5 and 6 is going clockwise fullspeed until the microswitch does not close down
		
		// stage 1
		// open the claws until the microswitch does not open - M6
		// start motor 2 4 to grab the bar #3 and motor 1 3 to get enough velocity */
    }
}

void set_pins(){
		//Setting up PINS to operate
		DDRC &= (0 << DDC0) & (0 << DDC1) & (0 << DDC2) & (0 << DDC3); // A0...A3 set pin as inputs
		DDRB &= (0 << DDB0) & (0 << DDB4) & (0 << DDB5); DDRD &= (0 << DDD0); // Set D8 D12 D13 D0 as inputs
		PORTB |= (1 << PORTB0) | (1 << PORTB5) | (1 << PORTB5); PORTD |= (1 << PORTD0); // Enable pull-ups for D8 D12 D13 D0 pins
		DDRD |= (1 << DDD2) | (1 << DDD3) | (1 << DDD4) | (1 << DDD5) | (1 << DDD6) | (1 << DDD7); DDRB |= (1 << DDB1) | (1 << DDB2) | (1 << DDB3); // setting D2 D3 D4 D5 D6 D7 D9 D10 D11 as outputs
}

void init_interrupt(void){
	//set the timer mode to CTC <- count to desired value and then restart and count again
	TCCR0A |= (1 << WGM01);
	//set the value that you want to count to <- 250 timer ticks (0-249)
	OCR0A = 0xF9;
	
	//enable the interrupt for on compare a for timer 0
	TIMSK0 |= (1 << OCIE0A); //can be interrupted by Compare A match
	//enable all interrupts
	sei();
	
	//start the timer
	TCCR0B |= (1 << CS01) | (1 << CS00); //set prescaler to 64 and start
}

//INTERRUPT ROUTINE for the COUNTER
ISR (TIMER0_COMPA_vect) {
	ms++;
	if (ms == 50) second_counter=second_counter+0.05;
	
}
