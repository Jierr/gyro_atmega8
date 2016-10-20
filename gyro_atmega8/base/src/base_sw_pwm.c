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
	base_sw_pwm_ctx.cycle_tick_count = 100;
	base_sw_pwm_ctx.cycle_tick = 0;
	for(p = 0; p < BASE_MAX_PINS; ++p)
	{
		base_sw_pwm_ctx.pin[p].pwm_duty = 50;
		base_sw_pwm_ctx.pin[p].pwm_duty_ticks = 50;
		base_sw_pwm_ctx.pin[p].pin = BASE_PIN_NONE;
		base_sw_pwm_ctx.pin[p].port = BASE_PORT_NONE;
	}
}

uint16_t base_sw_pwm_set_global_cycle(uint16_t hz)
{
	uint16_t cycle_count;
	
	base_sw_pwm_ctx.cycle_tick = 0;
	if(hz == 0)
	{		
		base_sw_pwm_ctx.cycle_tick_count = 100;
		return base_timer0_s_ticks(1)/100;		
	}
	
	cycle_count = base_timer0_s_ticks(1) / hz;
	
	
	if(cycle_count < 100)
	{
		base_sw_pwm_ctx.cycle_tick_count = 100;
		return base_timer0_s_ticks(1)/100;
	}
	else
	{
		base_sw_pwm_ctx.cycle_tick_count = cycle_count;		
		return hz;
	}
	
}

uint8_t base_sw_pwm_init(port_type_t port, pin_type_t pin)
{
	if(port == BASE_PORT_NONE || pin == BASE_PIN_NONE)
		return BASE_MAX_PINS;
		
	++base_sw_pwm_ctx.config_count;
	base_sw_pwm_ctx.pin[base_sw_pwm_ctx.config_count].pin = pin;
	base_sw_pwm_ctx.pin[base_sw_pwm_ctx.config_count].port = port;
	
	base_sw_pwm_ctx.pin[base_sw_pwm_ctx.config_count].pwm_duty = 50;
	base_sw_pwm_ctx.pin[base_sw_pwm_ctx.config_count].pwm_duty_ticks = 
		(uint16_t)(((uint32_t)base_sw_pwm_ctx.cycle_tick_count  * (uint32_t)base_sw_pwm_ctx.pin[base_sw_pwm_ctx.config_count].pwm_duty) / (uint32_t)100);
	
	switch(port)
	{
		case BASE_PORTB:
			DDRB |= pin;
			PORTB &= ~pin;
		break;
		case BASE_PORTC:
			DDRC |= pin;
			PORTC &= ~pin;
		break;
		case BASE_PORTD:
			DDRD |= pin;
			PORTD &= ~pin;
		break;
		default:
		break;
	}
	
	return base_sw_pwm_ctx.config_count-1;
}

void base_sw_pwm_set_duty(uint8_t pin_nr, int8_t percent)
{
	if(percent > 100)
	{
		percent=100;
	}
	else if(percent < 0)
	{
		percent = 0;
	}
	
	base_sw_pwm_ctx.pin[pin_nr].pwm_duty = percent;
	base_sw_pwm_ctx.pin[pin_nr].pwm_duty_ticks = (uint16_t)(((uint32_t)base_sw_pwm_ctx.cycle_tick_count  * (uint32_t)percent) / (uint32_t)100);
}

void base_sw_pwm_duty(uint8_t pin_nr, int8_t percent)
{
	percent += base_sw_pwm_ctx.pin[pin_nr].pwm_duty;
	if(percent > 100)
	{
		percent=100;
	}
	else if(percent < 0)
	{
		percent = 0;
	}
	
	base_sw_pwm_ctx.pin[pin_nr].pwm_duty = percent;
	base_sw_pwm_ctx.pin[pin_nr].pwm_duty_ticks = (uint16_t)(((uint32_t)base_sw_pwm_ctx.cycle_tick_count  * (uint32_t)percent) / (uint32_t)100);
}



void base_sw_pwm_timer0_callback()
{
	uint8_t p = 0;
	
	
	for(p = 0; p < base_sw_pwm_ctx.config_count; ++p)
	{
		if(base_sw_pwm_ctx.pin[p].pwm_duty_ticks == 0)
		{
			switch(base_sw_pwm_ctx.pin[p].port)
			{
				case BASE_PORTB:
				PORTB &= ~base_sw_pwm_ctx.pin[p].pin;
				break;
				case BASE_PORTC:
				PORTC &= ~base_sw_pwm_ctx.pin[p].pin;
				break;
				case BASE_PORTD:
				PORTD &= ~base_sw_pwm_ctx.pin[p].pin;
				break;
				default:
				break;
			}
		}
		else if (base_sw_pwm_ctx.pin[p].pwm_duty == 100)
		{
			switch(base_sw_pwm_ctx.pin[p].port)
			{
				case BASE_PORTB:
				PORTB |= base_sw_pwm_ctx.pin[p].pin;
				break;
				case BASE_PORTC:
				PORTC |= base_sw_pwm_ctx.pin[p].pin;
				break;
				case BASE_PORTD:
				PORTD |= base_sw_pwm_ctx.pin[p].pin;
				break;
				default:
				break;
			}
		}
		else
		{
			if(base_sw_pwm_ctx.cycle_tick == 0)
			{
				switch(base_sw_pwm_ctx.pin[p].port)
				{
					case BASE_PORTB:
					PORTB |= base_sw_pwm_ctx.pin[p].pin;
					break;
					case BASE_PORTC:
					PORTC |= base_sw_pwm_ctx.pin[p].pin;
					break;
					case BASE_PORTD:
					PORTD |= base_sw_pwm_ctx.pin[p].pin;
					break;
					default:
					break;
				}
			}
			else if (base_sw_pwm_ctx.cycle_tick == base_sw_pwm_ctx.pin[p].pwm_duty_ticks)
			{
				switch(base_sw_pwm_ctx.pin[p].port)
				{
					case BASE_PORTB:
					PORTB &= ~base_sw_pwm_ctx.pin[p].pin;
					break;
					case BASE_PORTC:
					PORTC &= ~base_sw_pwm_ctx.pin[p].pin;
					break;
					case BASE_PORTD:
					PORTD &= ~base_sw_pwm_ctx.pin[p].pin;
					break;
					default:
					break;
				}
			}
		}
		

		
	}
	
	++base_sw_pwm_ctx.cycle_tick;	
	if(base_sw_pwm_ctx.cycle_tick >= base_sw_pwm_ctx.cycle_tick_count)
	{
		/*
		base_usart_send_decimal(base_sw_pwm_ctx.cycle_tick_count);
		base_usart_send_string("\r\n");
		base_usart_send_decimal(base_sw_pwm_ctx.cycle_tick);
		base_usart_send_string("\r\n");
		*/
		base_sw_pwm_ctx.cycle_tick = 0;
	}
	
	
	
}