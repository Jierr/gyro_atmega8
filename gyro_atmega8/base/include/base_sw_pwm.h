/*
 * base_sw_pwm.h
 *
 * Created: 11.10.2016 00:02:22
 *  Author: Jierr
 */ 


#ifndef BASE_SW_PWM_H_
#define BASE_SW_PWM_H_

#include "config.h"
#include "base_timer.h"



void base_sw_pwm_init(port_type_t port, pin_type_t pin);
void base_sw_pwm_timer0_callback();



#endif /* BASE_SW_PWM_H_ */