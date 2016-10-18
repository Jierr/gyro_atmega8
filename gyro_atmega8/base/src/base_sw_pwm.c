/*
 * base_sw_pwm.c
 *
 * Created: 11.10.2016 00:02:35
 *  Author: Jierr
 */ 

#include "base_sw_pwm.h"

volatile base_sw_pwm_ctx_t base_sw_pwm_ctx;


void base_sw_pwm_ctx_init()
{
	uint8_t p = 0;
	base_sw_pwm_ctx.config_count = 0;
	for(p = 0; p < BASE_MAX_PINS; ++p)
	{
		base_sw_pwm_ctx.pin[p].pwm_duty = 50;
		base_sw_pwm_ctx.pin[p].pin = BASE_PIN_NONE;
		base_sw_pwm_ctx.pin[p].port = BASE_PORT_NONE;
	}
}

uint8_t base_sw_pwm_init(port_type_t port, pin_type_t pin)
{
	if(port == BASE_PORT_NONE || pin == BASE_PIN_NONE)
		return BASE_MAX_PINS;
		
	++base_sw_pwm_ctx.config_count;
	base_sw_pwm_ctx.pin[base_sw_pwm_ctx.config_count].pin = pin;
	base_sw_pwm_ctx.pin[base_sw_pwm_ctx.config_count].port = port;
	
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
	
	return base_sw_pwm_ctx.config_count-1;
}

void base_sw_pwm_timer0_callback()
{
	uint8_t p = 0;
	
	for(p = 0; p < base_sw_pwm_ctx.config_count; ++p)
	{
		
	}
	
}