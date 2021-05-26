/*
 * Potentiometer.c
 *
 * Created: 06/05/2021 13.00.10
 * Author : Daniel Szanka
 */ 

#define F_CPU16000000UL
#include <avr/io.h>
#include <stdio.h>
#include <stdint.h>
#include "usart.h"
#include <util/delay.h>
#define ADC_PIN 1
uint16_t adc_read(uint8_t adc_channel);
uint16_t adc_result;
double voltage [10], sum_voltage = 0, avg_voltage;
int i=0, n=0;

// This is another solution if we use ceramitic cap 100 pF.
int main (void)
{
	uart_init();
	io_redirect();
	DDRC=0xF0;//PC0..PC3 are input of the ADC
	PORTC=0x00;//make sure no pull-ups *make my own connections
	// Select Vref = AVcc
	ADMUX=(1<<REFS0);
	//set pre-scaler to 128 and turn on the ADC module
	ADCSRA=(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);
    while (1) 
    {
		adc_result=adc_read(ADC_PIN);
		voltage[i]=(5000.0/1024)*adc_result;
		if (n<3)
		{
			sum_voltage += voltage[i];
			avg_voltage = sum_voltage/(i+1);
		} 
		else
		{
			if (i == 3) sum_voltage -= voltage[0];
			if (i != 3) sum_voltage -= voltage[i+1];
			sum_voltage += voltage[i];
			avg_voltage = sum_voltage/3;
		}
		
		printf("%f, %f, %f, %d\n", voltage[i], sum_voltage, avg_voltage, i);
		n++;
		i++;
		if (i == 3) i = 0;
	}
	return (0);
}

uint16_t adc_read(uint8_t adc_channel){
	ADMUX &= 0xF0; // clear any previously used channel, but keep internal reference
	ADMUX |= adc_channel; // set the desired channel
	//start a conversion
	ADCSRA |= (1<<ADSC);
	// now wait for the conversion to complete
	while ( (ADCSRA & (1<<ADSC)) );
	// now we have the result, so we return it to the calling function as a 16 bit unsigned int
	return ADC;
}

