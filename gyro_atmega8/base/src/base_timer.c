/*
 * base_timer.c
 *
 * Created: 06.08.2016 19:52:37
 *  Author: Jierr
 */ 

/*
TCCR1A/B Timer/Counter Control Registers
COM1x1:0 bits control the OC1x pin output source
COM1x1:0 bits for defining the Output Compare (OC1x) state at the next Compare Match
*/

#include "config.h"
#include "base_timer.h"
#include "base_usart.h"


volatile base_timer1_context_t base_timer1_context = {
	.top = 0xFF,
	.pwm_duty = 50,
	.invertiert = 0
};

volatile base_timer0_context_t base_timer0_context = {
	.tick = 0,
	.prescaler = 1,
	.ms_tick_count = 0,
};

static uint32_t _base_timer0_ms_ticks(uint32_t ms);

void base_timer0_init()
{
	uint8_t reg8 = 0;
	base_timer0_context.tick = 0;
	base_timer0_context.prescaler = BASE_TIMER0_PRESCALER;
	base_timer0_context.ms_tick_count = _base_timer0_ms_ticks(1);
	TCNT0 = 0x00;
	//reset Interrupt
	TIFR |= 0x01;
	//Enable Overflow Interrupt
	TIMSK |= 0x01;
	//select prescaler /64 -> 250000 ticks / second -> Ein Tick alle 4 us
	
	switch(base_timer0_context.prescaler)
	{
		case 8:
		TCCR0 = 0x02;
		break;
		case 64:
		TCCR0 = 0x03;
		break;
		case 256:
		TCCR0 = 0x04;
		break;
		case 1024:
		TCCR0 = 0x05;
		break;
		case 1:
		TCCR0 = 0x01;
		break;
		default:
		TCCR0 = 0x00;
		break;
	}
	TCCR0 = 0x03;	
}

uint32_t _base_timer0_ms_ticks(uint32_t ms)
{
	uint32_t sec_ticks = 0;
	uint32_t ms_ticks = 0;
	
	sec_ticks = (uint32_t)F_CPU / base_timer0_context.prescaler / 256;
	ms_ticks = ((uint32_t)ms*sec_ticks) / 1000 ;
	return ms_ticks;
}

uint32_t base_timer0_ms_ticks(uint32_t ms)
{
	return ms * base_timer0_context.ms_tick_count;
}



void base_timer1_init()
{
	uint8_t reg8 = 0;
	//Configure OC1x buffer before DDR, set Pin via FOC1x in normal mode
	//TOP is 0x03FF
	
	reg8 = TCCR1A;
	//set output to high level
	reg8 |= (1<<COM1A1);
	reg8 &= ~(1<<COM1A0);
	base_timer1_context.invertiert = 0;
	//force Output to 0
	reg8 |= (1<<FOC1A);
	reg8 |= (1<<WGM11);
	reg8 |= (1<<WGM10);
	TCCR1A = reg8;
	
	reg8 = TCCR1B;
	reg8 &= ~(1<<WGM13);
	reg8 |= (1<<WGM12);
	TCCR1B |= reg8;
	base_timer1_context.top = 0x3FF;
	
	OCR1AH = (uint8_t)0x1;
	OCR1AL = (uint8_t)0xFF;
	base_timer1_context.pwm_duty = 50;
	
	//TIMSK |= OCIE1A;
	
	//choose internal clocksource
	reg8 = TCCR1B;
	reg8 &= ~(1<<CS10);
	reg8 &= ~(1<<CS11);
	reg8 |= (1<<CS12);
	TCCR1B = reg8;
	
	//configure PB1 as output for OC1A
	PORTB &= ~(1<<PB1);
	DDRB |= (1<<DDB1);	
}


void base_timer1_set_pwm_frequency()
{
	
}


void base_timer1_set_pwm_change_duty_by(int8_t percentage)
{
	volatile uint16_t duty_cycle = 0;
			
	if(base_timer1_context.invertiert)
		percentage = -percentage;
				
	if(	((base_timer1_context.pwm_duty == 0) && (percentage < 0)) ||
		((base_timer1_context.pwm_duty == 100) && (percentage > 0)))
		return;
		

	base_timer1_context.pwm_duty = base_timer1_context.pwm_duty + percentage;		
	if(base_timer1_context.pwm_duty < 100)
	{
		duty_cycle = (uint16_t)(((uint32_t)base_timer1_context.top * (uint32_t)base_timer1_context.pwm_duty) / (uint32_t)100);
	}
	else
	{
		duty_cycle = base_timer1_context.top;
		base_timer1_context.pwm_duty = 100;
	}
	
	base_usart_send_string(":change_duty_by:> ");
	base_usart_send_decimal(base_timer1_context.pwm_duty);
	base_usart_send_string("\r\n");
	//(*(volatile uint8_t*)&OCR1AH) = (uint8_t)((duty_cycle>>8)&0xFF);
	//(*(volatile uint8_t*)&OCR1AL) = (uint8_t)((duty_cycle)&0xFF);
	OCR1AH = (uint8_t)((duty_cycle>>8) & 0xFF);
	OCR1AL = (uint8_t)((duty_cycle) & 0xFF);
	
}

void base_timer1_set_pwm_duty(uint8_t percentage)
{	
	uint16_t duty_cycle = 0;
	
	if(base_timer1_context.invertiert)
		percentage = 100 - percentage;
		
	if(percentage < 100)
	{
		duty_cycle = (uint16_t)(((uint32_t)base_timer1_context.top * (uint32_t)base_timer1_context.pwm_duty) / (uint32_t)100);
		base_timer1_context.pwm_duty = percentage;
	}
	else
	{
		duty_cycle = base_timer1_context.top - 1;
		base_timer1_context.pwm_duty = 100;
	}
	(*(volatile uint8_t*)&OCR1AH) = (uint8_t)((duty_cycle>>8) & 0xFF);
	(*(volatile uint8_t*)&OCR1AL) = (uint8_t)((duty_cycle) & 0xFF);
}

void base_timer1_set_pwm_level(uint8_t high)
{
	uint8_t reg8 = 0;
	reg8 = TCCR1A;
	if(high != 0)
	{		
		//non inverted, low level after compare match
		reg8 |= (1<<COM1A1);
		reg8 &= ~(1<<COM1A0);
		base_timer1_context.invertiert = 0;
	}
	else
	{
		//inverted, high level ater compare match
		reg8 |= (1<<COM1A1);
		reg8 |= (1<<COM1A0);
		base_timer1_context.invertiert = 1;
		
	}
	TCCR1A = reg8;
}