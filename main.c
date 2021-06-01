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
void move_elbow(unsigned char stage, unsigned char motor, unsigned int angle);
void move_base(unsigned char stage, unsigned char motor, unsigned int angle);
volatile int miliseconds=0;
unsigned char stage=0;
unsigned int passed_timeL=0;
unsigned int passed_timeR=0;
int previous_angle_L1 = 0, previous_angle_L2 = 0, previous_angle_R1 = 0, previous_angle_R2 = 0;
unsigned char motor_speed_R2=0, motor_speed_L2=0, motor_speed_R1=0, motor_speed_L1=0;


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
			if((micro_switch(MSL)==0) || ((passed_timeL-miliseconds<100) && (stage==0))) close_gripper(L3);
			if((micro_switch(MSR)==0) || ((passed_timeL-miliseconds<100) && (stage==0))) close_gripper(R3);
			move_elbow(stage,R2, 20);
			move_elbow(stage,L2, 20);
			move_base(stage,R1, 20);
			move_base(stage,L1, 20);
			if(previous_angle_L1==20 && previous_angle_L2==20 && previous_angle_R1==20 && previous_angle_R2==20) stage++;
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
		if(micro_switch(MSR)==1) passed_timeR=miliseconds;
		if(micro_switch(MSR)==1 && passed_timeR>0)
		{
			if(miliseconds-passed_timeR>100){
				 motor_contoller (R3, 0, BRAKE); //wait for 0.1sec more after microswitch signal
				 passed_timeR=0;
			}
		}
	}
	if (motor == L3)
	{
		if(micro_switch(MSL)==0 || (passed_timeL-miliseconds<100)){
			motor_contoller(L3, 255, CCW);
			if(micro_switch(MSL)==1) passed_timeL=miliseconds;
			if(micro_switch(MSL)==1 && passed_timeL>0){
				if(miliseconds-passed_timeL>100){
					 motor_contoller (L3, 0, BRAKE); //wait for 0.1sec more after microswitch signal
					 passed_timeL=0;
				}	
			}
		}
	}
}

void close_gripper(unsigned char motor){
	//if right gripper requested
	if (motor == R3)
	if((opto_coupler(OR)==1) && (micro_switch(MSR)==1))
	{
		motor_contoller(R3, 255, CW);
	} 
	if ((opto_coupler(OR)==0) && (micro_switch(MSR)==0) && (passed_timeR==0)) passed_timeR=miliseconds; 
	if ((micro_switch(MSR)==0) && (miliseconds-passed_timeR==100)) //after microswitch changes signal, wait 0.1 before braking.
	{
		motor_contoller (L3, 0, BRAKE); 
		passed_timeR=0;
	}
	//if left gripper requested
	if (motor == L3)
	if((opto_coupler(OL)==1) && (micro_switch(MSL)==1))
	{
		motor_contoller(L3, 255, CW);
	} 
	if((opto_coupler(OL)==0) && (micro_switch(MSL)==0) && (passed_timeL==0)) passed_timeL=miliseconds; 
	if ((micro_switch(MSL)==0) && (miliseconds-passed_timeL==100)) //after microswitch changes signal, wait 0.1 before braking.
	{
		motor_contoller (L3, 0, BRAKE); 
		passed_timeL=0;
	}
}

void move_elbow(unsigned char stage, unsigned char motor, unsigned int angle){
	if (stage ==0){
		if (motor == R2){
			motor_speed_R2 = 255;
			previous_angle_R2=potentiometer_angle(P3);
			if (previous_angle_R2<angle) motor_contoller(R2,motor_speed_R2,CW);
			if (previous_angle_R2>angle) motor_contoller(R2,motor_speed_R2,CCW);
		if (previous_angle_R2==angle) motor_contoller(R2,0,BRAKE);}
		if (motor == L2){
			motor_speed_L2 = 255;
			previous_angle_L2=potentiometer_angle(P4);
			if (previous_angle_L2<angle) motor_contoller(L2,motor_speed_L2,CW);
			if (previous_angle_L2>angle) motor_contoller(L2,motor_speed_L2,CCW);
		if (previous_angle_L2==angle) motor_contoller(L2,0,BRAKE);}
	}
}

void move_base(unsigned char stage, unsigned char motor, unsigned int angle){
	if (stage ==0){
		if (motor == R1){
			motor_speed_R1 = 255;
			previous_angle_R1=potentiometer_angle(P1);
			if (previous_angle_R1<angle) motor_contoller(R1,motor_speed_R2,CW);
			if (previous_angle_R1>angle) motor_contoller(R1,motor_speed_R2,CCW);
		if (previous_angle_R1==angle) motor_contoller(R1,0,BRAKE);}
		if (motor == L1){
			motor_speed_L1 = 255;
			previous_angle_L1=potentiometer_angle(P2);
			if (previous_angle_L1<angle) motor_contoller(L1,motor_speed_L1,CW);
			if (previous_angle_L1>angle) motor_contoller(L1,motor_speed_L1,CCW);
		if (previous_angle_L1==angle) motor_contoller(L1,0,BRAKE);}
	}
}



//INTERRUPT ROUTINE for the COUNTER
ISR (TIMER0_COMPA_vect) {
	miliseconds++;
	
}
