#ifndef MOTORCONTROL_H_INCLUDED
#define MOTORCONTROL_H_INCLUDED

void move_elbow(unsigned char stage, unsigned char motor, unsigned int angle);
void move_base(unsigned char stage, unsigned char motor, unsigned int angle);

#endif