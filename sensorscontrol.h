#ifndef SENSORSCONTROL_H_INCLUDED
#define SENSORSCONTROL_H_INCLUDED

bool opto_coupler(unsigned char);
void motor_contoller(unsigned char, unsigned char, unsigned char); // specify which motor, what speed and which direction (clockwise/counter-clockwise)
unsigned int potentiometer_angle (unsigned char);
uint16_t adc_read (uint8_t);
bool micro_switch (unsigned char);
void motor_direction(unsigned char, unsigned char); //which motor, which direction.

#endif