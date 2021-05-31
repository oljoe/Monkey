#ifndef MOTORBOARD_H_INCLUDED
#define MOTORBOARD_H_INCLUDED

void pwm_init (void);
void motor_state (void);
void set_pwm_signal (void);
void send_data_to_chip (void);

#endif