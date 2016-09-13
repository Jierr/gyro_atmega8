/*
 * _7segment.c
 *
 * Created: 06.08.2016 19:05:12
 *  Author: Jierr
 */ 

#include "abstract_7segment.h"


void abstract_7segment_init()
{
	DDRD |= (1<<DDD6);
	DDRD |= (1<<DDD7);
	DDRB |= (1<<DDB1) | (1<<DDB2) | (1<<DDB3) | (1<<DDB4);
	DDRC |= (1<<DDC1) | (1<<DDC0) | (1<<DDC3) | (1<<DDC2);
	PORTD |= (1<<PD6);
	PORTD |= (1<<PD7);
}

void abstract_7segment_display(uint8_t s1, uint8_t s2)
{
	static volatile uint8_t curr = 1;
	
	if(curr == 1)
	{
		PORTD |= (1<<PD6);
		PORTD &= ~(1<<PD7);
		switch(s1)
		{
			case 1:
			PORTB &= B1_ON;
			PORTB |= B1_OFF;
			PORTC &= C1_ON;
			PORTC |= C1_OFF;
			break;
			case 2:
			PORTB &= B2_ON;
			PORTB |= B2_OFF;
			PORTC &= C2_ON;
			PORTC |= C2_OFF;
			break;
			case 3:
			PORTB &= B3_ON;
			PORTB |= B3_OFF;
			PORTC &= C3_ON;
			PORTC |= C3_OFF;
			break;
			case 4:
			PORTB &= B4_ON;
			PORTB |= B4_OFF;
			PORTC &= C4_ON;
			PORTC |= C4_OFF;
			break;
			case 5:
			PORTB &= B5_ON;
			PORTB |= B5_OFF;
			PORTC &= C5_ON;
			PORTC |= C5_OFF;
			break;
			case 6:
			PORTB &= B6_ON;
			PORTB |= B6_OFF;
			PORTC &= C6_ON;
			PORTC |= C6_OFF;
			break;
			case 7:
			PORTB &= B7_ON;
			PORTB |= B7_OFF;
			PORTC &= C7_ON;
			PORTC |= C7_OFF;
			break;
			case 8:
			PORTB &= B8_ON;
			PORTB |= B8_OFF;
			PORTC &= C8_ON;
			PORTC |= C8_OFF;
			break;
			case 9:
			PORTB &= B9_ON;
			PORTB |= B9_OFF;
			PORTC &= C9_ON;
			PORTC |= C9_OFF;
			break;
			default:
			PORTB &= B0_ON;
			PORTB |= B0_OFF;
			PORTC &= C0_ON;
			PORTC |= C0_OFF;
			break;
		}
		
		
		curr = 2;
	}
	else
	{
		PORTD |= (1<<PD7);
		PORTD &= ~(1<<PD6);
		switch(s2)
		{
			case 1:
			PORTB &= B1_ON;
			PORTB |= B1_OFF;
			PORTC &= C1_ON;
			PORTC |= C1_OFF;
			break;
			case 2:
			PORTB &= B2_ON;
			PORTB |= B2_OFF;
			PORTC &= C2_ON;
			PORTC |= C2_OFF;
			break;
			case 3:
			PORTB &= B3_ON;
			PORTB |= B3_OFF;
			PORTC &= C3_ON;
			PORTC |= C3_OFF;
			break;
			case 4:
			PORTB &= B4_ON;
			PORTB |= B4_OFF;
			PORTC &= C4_ON;
			PORTC |= C4_OFF;
			break;
			case 5:
			PORTB &= B5_ON;
			PORTB |= B5_OFF;
			PORTC &= C5_ON;
			PORTC |= C5_OFF;
			break;
			case 6:
			PORTB &= B6_ON;
			PORTB |= B6_OFF;
			PORTC &= C6_ON;
			PORTC |= C6_OFF;
			break;
			case 7:
			PORTB &= B7_ON;
			PORTB |= B7_OFF;
			PORTC &= C7_ON;
			PORTC |= C7_OFF;
			break;
			case 8:
			PORTB &= B8_ON;
			PORTB |= B8_OFF;
			PORTC &= C8_ON;
			PORTC |= C8_OFF;
			break;
			case 9:
			PORTB &= B9_ON;
			PORTB |= B9_OFF;
			PORTC &= C9_ON;
			PORTC |= C9_OFF;
			break;
			default:
			PORTB &= B0_ON;
			PORTB |= B0_OFF;
			PORTC &= C0_ON;
			PORTC |= C0_OFF;
			break;
		}
		
		curr = 1;
	}
}
