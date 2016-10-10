/*
 * base_sw_pwm.c
 *
 * Created: 11.10.2016 00:02:35
 *  Author: Jierr
 */ 

#include "base_sw_pwm.h"



void base_sw_pwm_init(port_type_t port, pin_type_t pin)
{
	switch(port)
	{
		case BASE_PORTB:
			DDRB |= pin;
			PORTB |= pin;
		break;
		case BASE_PORTC:
			DDRC |= pin;
			PORTC |= pin;
		break;
		case BASE_PORTD:
			DDRD |= pin;
			PORTD |= pin;
		break;
		default:
		break;
	}
}

void base_sw_pwm_timer0_callback()
{
	
}