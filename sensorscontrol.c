#include <avr/io.h>
#include <stdio.h>
#include <avr/io.h>
#include <stdbool.h>

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

bool R1_IN0 = 1, R1_IN1 = 0, L1_IN0 = 0, L1_IN1 = 0, R2_IN0 = 0, R2_IN1 = 0, L2_IN0 = 0, L2_IN1=0, R3_IN0 = 0, R3_IN1 = 0, L3_IN0 = 0, L3_IN1 = 0;
bool motor_array[12];

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

bool opto_coupler(unsigned char opto_coupler_nr){
	bool on;
	if (opto_coupler_nr == OR)
	{
		if  (1 << PINB5) on=1;
		if  (0 << PINB5) on=0;
	}
	
	if (opto_coupler_nr == OL)
	{
		if  (1 << PIND0) on=1;
		if  (0 << PIND0) on=0;
	}
	return on;
}

bool micro_switch(unsigned char micro_switch_nr){
	bool on;
	
	if (micro_switch_nr == MSR)
	{
		if  (1 << PINB0) on=1;
		if  (0 << PINB0) on=0;
	}
	if (micro_switch_nr == MSL)
	{
		if  (1 << PINB4) on=1;
		if  (0 << PINB4) on=0;
	}
	return on;
}

unsigned int potentiometer_angle(unsigned char potentiometer_nr){
	uint16_t adc_result;
	int angle;
	int i = 0, n = 0;
	double voltage [10], sum_voltage = 0, avg_voltage;
	ADMUX=(1<<REFS0); // Set voltage reference to 5V (Vcc)
	ADCSRA=(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN); //set pre-scaler to 128 and turn on the ADC module
	
	adc_result=adc_read(potentiometer_nr);
	voltage[i]=(5000.0/1024)*adc_result;
	if (n<10)
	{
		sum_voltage += voltage[i];
		avg_voltage = sum_voltage/(i+1);
	}
	else
	{
		if (i == 9) sum_voltage -= voltage[0];
		if (i != 9) sum_voltage -= voltage[i+1];
		sum_voltage += voltage[i];
		avg_voltage = sum_voltage/10;
	}
	angle = (avg_voltage*300)/5000;
	n++;
	i++;
	if (i == 10) i = 0;
	return angle;
}

void motor_direction(unsigned int motor_nr, unsigned int motor_state){
	if (motor_nr == R1){
		if (motor_state ==CW) {R1_IN0 = 1; R1_IN1 = 0;}
		if (motor_state ==CCW) {R1_IN0 = 0; R1_IN1 = 1;}
		if (motor_state ==BRAKE) {R1_IN0 = 0; R1_IN1 = 0;}
	}
	if (motor_nr == L1){
		if (motor_state ==CW) {L1_IN0 = 1; L1_IN1 = 0;}
		if (motor_state ==CCW) {L1_IN0 = 0; L1_IN1 = 1;}
		if (motor_state ==BRAKE) {L1_IN0 = 0; L1_IN1 = 0;}
	}
	if (motor_nr == R2){
		if (motor_state ==CW) {R2_IN0 = 1; R2_IN1 = 0;}
		if (motor_state ==CCW) {R2_IN0 = 0; R2_IN1 = 1;}
		if (motor_state ==BRAKE) {R2_IN0 = 0; R2_IN1 = 0;}
	}
	if (motor_nr == L2){
		if (motor_state ==CW) {L2_IN0 = 1; L2_IN1 = 0;}
		if (motor_state ==CCW) {L2_IN0 = 0; L2_IN1 = 1;}
		if (motor_state ==BRAKE) {L2_IN0 = 0; L2_IN1 = 0;}
	}
	if (motor_nr == R3){
		if (motor_state ==CW) {R3_IN0 = 1; R3_IN1 = 0;}
		if (motor_state ==CCW) {R3_IN0 = 0; R3_IN1 = 1;}
		if (motor_state ==BRAKE) {R3_IN0 = 0; R3_IN1 = 0;}
	}
	if (motor_nr == L3){
		if (motor_state ==CW) {L3_IN0 = 1; L3_IN1 = 0;}
		if (motor_state ==CCW) {L3_IN0 = 0; L3_IN1 = 1;}
		if (motor_state ==BRAKE) {L3_IN0 = 0; L3_IN1 = 0;}
	}
}

void motor_contoller (unsigned char motor_nr, unsigned char motor_speed, unsigned char motor_state){
	void pwm_init (void);
	void send_data_to_chip (void);
	   
	pwm_init();
	switch (motor_nr)
	if(motor_nr == R1){
		 OCR2B = motor_speed; //D3 pin -> MotorBoard-1 enA, 0-255 values
		 motor_direction(motor_nr, motor_state); // motor R1, rotate CW, CCW, BREAK
	}
	if(motor_nr == L1){
		 OCR0B = motor_speed; //D5 pin -> MotorBoard-1 enB, 0-255 values
		 motor_direction(motor_nr, motor_state);
	}
	if(motor_nr == R2){
		 OCR0A = motor_speed; //D6 pin -> MotorBoard-2 enA, 0-255 values
		 motor_direction(motor_nr, motor_state);
	}
	if(motor_nr == L2){
		OCR1A = motor_speed; //D9 pin -> MotorBoard-2 enB, 0-255 values
		motor_direction(motor_nr, motor_state);
	}
	if(motor_nr == R3){
		OCR1B = motor_speed; //D10 pin -> MotorBoard-3 enA, 0-255 values
		motor_direction(motor_nr, motor_state);
	}
	if(motor_nr == L3){
		OCR2A = motor_speed; //D11 pin -> MotorBoard-3 enB, 0-255 values
		motor_direction(motor_nr, motor_state);
	}
	
	//motor states

	   motor_array[0]=L3_IN1;
	   motor_array[1]=L3_IN0;
	   motor_array[2]=R3_IN1;
	   motor_array[3]=R3_IN0;
	   motor_array[4]=L2_IN1;
	   motor_array[5]=L2_IN0;
	   motor_array[6]=R2_IN1;
	   motor_array[7]=R2_IN0;
	   motor_array[8]=L1_IN1;
	   motor_array[9]=L1_IN0;
	   motor_array[10]=R1_IN1;
	   motor_array[11]=R1_IN0;
	   send_data_to_chip();
   }
  
void send_data_to_chip (void){
	PORTD &= (0 << DDD7); // Latch is on low
	
	for (int i=11; i<=0; i--)
	{
		PORTD &= (0 << DDD4); //Clock is on low
		if (motor_array[i] == 0) PORTD &= (0 << DDD2); //Send 0 to data pin
		if (motor_array[i] == 1) PORTD |= (1 << DDD2); //Send 1 to data pin
		PORTD |= (1 << DDD4); //Clock is on high
	}
	PORTD |= (1 << DDD7); // Latch is on high
}

void pwm_init(void) {
	TCCR0A |= (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00); // enable fast PWM mode and PD3
	TCCR0B |= (1 << CS00) | (1 << CS01); // pre-scaler 64 (arduino_clk/(2*pre-scaler*2^8) = Hz
	
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10); // enable fast PWM mode and PD3
	TCCR1B |= (1 << WGM12) | (1 << CS10) | (1 << CS11); // pre-scaler 64 (arduino_clk/(2*pre-scaler*2^8) = Hz
	
	TCCR2A |= (1<< COM2A1) | (1 << COM2B1) | (1 << WGM21) | (1 << WGM20); // enable fast PWM mode and PD3
	TCCR2B |= (1 << CS22); // pre-scaler 64 (arduino_clk/(2*pre-scaler*2^8) = Hz
}

