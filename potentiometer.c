#include <avr/io.h>
#include <stdio.h>
#include <stdint.h>
#include "potentiometer.h"

//function prototypes
uint16_t adc_read(uint8_t adc_channel); 

void potentiometer_init(void){
	uint16_t adc_result; 
	// Select Vref = AVcc
	ADMUX = (1<<REFS0); //set prescaler to 128 and turn on the ADC module
	ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN); 
	while(1){
	adc_result = adc_read(ADC_PIN);
	//do something with this result…
	}
}
uint16_t adc_read(uint8_t adc_channel){
ADMUX &= 0xf0; // clear any previously used channel, but keep internal reference
ADMUX |= adc_channel; // set the desired channel 
//start a conversion
ADCSRA |= (1<<ADSC);
// now wait for the conversion to complete
while ( (ADCSRA & (1<<ADSC)) );
// now we have the result, so we return it to the calling function as a 16 bit unsigned int
return ADC;
}