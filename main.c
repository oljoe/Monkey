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
#include "motorcontrol.h"

#define STAGE_AMOUNT 6
#define CW 1
#define CCW 2
#define BRAKE 3
#define R1 1 //right base motor
#define R2 2 //right elbow motor
#define R3 3 //right gripper
#define L1 4 //left base motor
#define L2 5 //left elbow motor
#define L3 6 //left gripper
#define P1 0 //potentiometer 1
#define P2 1 //potentiometer 2
#define P3 2 //potentiometer 3
#define P4 3 //potentiometer 4
#define OR 1 //right optocoupler
#define OL 2 //left optocoupler
#define MSR 1 // right microswitch
#define MSL 2 // left microswitch

void open_gripper(unsigned char motor);
void close_gripper(unsigned char motor);
void set_pins(void);
void init_interrupt(); //function initiates 
volatile int miliseconds=0;
unsigned char stage=0;
unsigned int passed_time=0;


int main(void)
{
	set_pins();
	uart_init();   // open the communication to the micro controller
	i2c_init();    // initialize the i2c communication.
	io_redirect(); // redirect the input/output to the computer.
	// COUNTER
	init_interrupt();
	
	//start-up -> assume starting position
	close_gripper(R3); //close right
	close_gripper(L3); //close left gripper
	//make sure all motors are in BRAKE mode
	motor_direction(R1, BRAKE);
	motor_direction(L1, BRAKE);
	motor_direction(R2, BRAKE);
	motor_direction(L2, BRAKE);
	motor_direction(R3, BRAKE);
	motor_direction(L3, BRAKE);
	//L2 needs CCW 20 degrees from 180 degrees 2670 mV to get the right degree
	//R2 needs CW 20 degrees from 180 degrees (5V = 300 degrees) -> 3333 mV
	
	//------------------------BEGIN MOTION-------------------------------//
	
	while(1){
		
		for(unsigned char stage=0;stage<STAGE_AMOUNT;stage++){
			if(micro_switch(MSL)==0 || (passed_time-miliseconds<100)) close_gripper(L3);
			if(micro_switch(MSR)==0 || (passed_time-miliseconds<100)) close_gripper(R3);
			move_elbow(stage,R2, 20);
			move_elbow(stage,L2, 20);
			move_base(stage,R1, 20);
			move_base(stage,L1, 20);
		}
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

void open_gripper(unsigned char motor){

	if (motor == R3) 
	{
		motor_contoller(R3, 255, CCW);
		if(micro_switch(MSR)==1) passed_time=miliseconds;
		if(micro_switch(MSR)==1 && passed_time>0)
		{
			if(miliseconds-passed_time>100){
				 motor_contoller (R3, 0, BRAKE); //wait for 0.1sec more after microswitch signal
				 passed_time=0;
			}
		}
	}
	if (motor == L3)
	{
		if(micro_switch(MSL)==0 || (passed_time-miliseconds<100)){
			motor_contoller(L3, 255, CCW);
			if(micro_switch(MSL)==1) passed_time=miliseconds;
			if(micro_switch(MSL)==1 && passed_time>0){
				if(miliseconds-passed_time>100){
					 motor_contoller (L3, 0, BRAKE); //wait for 0.1sec more after microswitch signal
					 passed_time=0;
				}	
			}
		}
	}
}

void close_gripper(unsigned char motor){
	if (motor == R3 )
	{
		while (opto_coupler(OR)==1) {};
		{
			motor_contoller( R3, 255, CW);
		}
		passed_time=miliseconds;
		
	}if (micro_switch(MSR)==0) motor_contoller (R3, 0, BRAKE);

	if (motor == L3 )
	while (opto_coupler(OL)==1) {};
	{
		motor_contoller( L3, 255, CW);
	} // Wait 0.5ms
	if (micro_switch(MSL)==0) motor_contoller (L3, 0, BRAKE);
}

//INTERRUPT ROUTINE for the COUNTER
ISR (TIMER0_COMPA_vect) {
	miliseconds++;
	
}
