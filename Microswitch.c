

 
#define F_CPU 16000000UL

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "usart.h"

int main(void) {    

    u
	art_init(); // open the communication to the microcontroller
	io_redirect(); // redirect input and output to the communication

    DDRD = 0x00; //Set all PD pins as inputs for the microswitch (used PD7)
    PORTD = 0xFF; // Enable pull at PD inputs
	
	
	printf ("Working bitch");
	
    char Check_Switch(void)   //returns 1 if switch attached to PD7 is closed, 0 if switch is open
    {
	    if (PIND==0b11111111) //check if pin PD7 is low, meaning switch is on
	    {
		    _delay_ms(50);  //wait 50 msec to allow any switch bounce to die out

		    if(PIND==0b01111111) return(1);   //the switch is really closed
		    printf ("The switch is pressed.");
	    }
	    return(0);  //the switch is open
	 }  /* end of Check_P0 function */

}

