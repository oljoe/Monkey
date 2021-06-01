#include <avr/io.h>
#include <stdio.h>
#include <avr/io.h>
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

int previous_angle_L1 = 0, previous_angle_L2 = 0, previous_angle_R1 = 0, previous_angle_R2 = 0;
unsigned char motor_speed_R2=0, motor_speed_L2=0, motor_speed_R1=0, motor_speed_L1=0;

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
	
