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

void open_gripper(unsigned char motor){
	//when microswitch==1, do 0,5 second more of opening
}

void close_gripper(unsigned char motor){
	//open at max speed until microswitch signal == 1 on & optocoupler == 0 then rotate 0.1 ms

}

void move_elbow(unsigned char motor, unsigned int angle){
	// if CW angle <0, then |angle|, but CCW
	// if desired angle > angle, then CCW
}

void move_base(unsigned char motor, unsigned int angle){
	
	
}