 #define F_CPU 16000000UL

 #include <stdio.h>
 #include <avr/io.h>
 #include <util/delay.h>
 #include "usart.h"

unsigned char on=0;

 unsigned char opto(void) {

	 uart_init(); // open the communication to the microcontroller
	 io_redirect(); // redirect input and output to the uart

		 DDRD = 0x00; //Set all PD pins as inputs for the optocoupler (PD7)
		 PORTD = 0xFF; // Enable pull at PD inputs
		 if (PIND==0b01111111) // change if PD7 pin is not used
		 {
			 on=1;
		 }
		 else
		 {
			 on=0;
		 }
	return on;
	 }

