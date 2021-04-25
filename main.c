#define F_CPU 16000000UL
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#define BAUDRATE 57600
#define BAUD_PRESCALER ((F_CPU/(BAUDRATE*16UL))-1)

//function prototypes
void pwm_set (char); //sets motor pwm.
char optocoupler_check (void); //checks if optocoupler signal is 1 or 0. (might need to be an interrupt instead of function).
char check_values (char); //checks angle, velocity, etc. If needed values are achieved, the device starts grabbing bar.

typedef struct 
{
	int motorR1;
	int motorR2;
	int motorR3;
	int motorL1;
	int motorL2;
	int motorL3;
} pwm;

pwm motor;


int main(void)
{
	char stage_number=1; //which stage device is currently on. Used to determine what pwm speed to set.
	
	//initialize all ports, enable pullups, in/out etc.
	
	//configure interrupts 
	UCSR0B|=(1<<RXCIE0); //enable interrupts for RXIE
	sei(); //enable global interrupts
	
	//program loops
	
		// loop for bars 1-5. ->Normal brachiation
		// 1. set pwm for first and second motor, to leave start position.
		// 2. After set speed and angle has been achieved, initiate movement for first motor, to grab next bar.
		// 3. Confirm if grabbed -> check velocity, angle, optocoupler signal
		// 4. If state confirmed, change variables to achieve next needed speed and angle
		// 5. Repeat loop with the changed values and using the second motor
		do 
		{
				pwm_set(stage_number);
				while(check_values(stage_number)==0){ //either this or time-based command.
					//wait to achieve the needed values
				stage_number++;
			}
		} while (stage_number<32); //stage number smaller than number of stages at normal brachiation
				
		// loop for bars 6-8. ->Sprinting motion
		
		// 1. set pwm for first motor, to leave start position.
		// 2. After set speed and angle has been achieved, initiate movement for first motor, to grab next bar.
		// 3. When Optocoupler sends positive signal, release second motor from previous bar, grab next bar.
		// 5. repeat, with different values for speed and angle and using previously achieved momentum. */
	
	return (0);
}

ISR (USART_RX_vect){ //'RX' is for 'recieve'
	volatile unsigned char recieved_data=UDR0;
	if(recieved_data==1) PORTD^=0b00010000; // ^= is XOR
	if(recieved_data==2) PORTD^=0b00100000; // these commands turn LEDs on
	if(recieved_data==3) PORTD^=0b01000000;
	if(recieved_data==4) PORTD^=0b10000000;
}

void pwm_set (char input_number){

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
	char agree=0;
	 
	 return agree;
}