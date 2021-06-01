#ifndef SENSORSCONTROL_H_INCLUDED
#define SENSORSCONTROL_H_INCLUDED

void open_gripper(unsigned char motor);
void close_gripper(unsigned char motor);

void move_elbow(unsigned char motor, unsigned int angle);
void move_base(unsigned char motor, unsigned int angle);

#endif