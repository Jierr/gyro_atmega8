/*
 * base_usart.c
 *
 * Created: 06.08.2016 19:13:23
 *  Author: Jierr
 */ 

#include "base_usart.h"


void base_usart_init(uint16_t ubrr)
{
	uint8_t sreg = SREG;
	//cli();
	UBRRH = (uint8_t)(ubrr>>8);
	UBRRL = (uint8_t)(ubrr);
	
	UCSRB = (1 << TXEN)  | (1 << RXEN) | (1 << RXCIE);
	UCSRC = (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);
	SREG = sreg;
}

void base_usart_send_byte(uint8_t data)
{
	while(!(UCSRA & (1<<UDRE)))
	;
	UDR = data;
}

void base_usart_send_string(char* str)
{
	uint8_t c = 0;
	c = 0;
	while (str[c])
	{
		base_usart_send_byte((uint8_t)str[c]);
		++c;
	}
}

void base_usart_send_byte_hex(uint8_t data)
{
	uint8_t low = 0;
	uint8_t high = 0;
	
	high = (data>>4)&0xF;
	low = (data)&0xF;
	
	switch(high)
	{
		case 10:
		base_usart_send_byte('A');
		break;
		case 11:
		base_usart_send_byte('B');
		break;
		case 12:
		base_usart_send_byte('C');
		break;
		case 13:
		base_usart_send_byte('D');
		break;
		case 14:
		base_usart_send_byte('E');
		break;
		case 15:
		base_usart_send_byte('F');
		break;
		default:
		base_usart_send_byte(high + '0');
		break;
	}
	
	switch(low)
	{
		case 10:
		base_usart_send_byte('A');
		break;
		case 11:
		base_usart_send_byte('B');
		break;
		case 12:
		base_usart_send_byte('C');
		break;
		case 13:
		base_usart_send_byte('D');
		break;
		case 14:
		base_usart_send_byte('E');
		break;
		case 15:
		base_usart_send_byte('F');
		break;
		default:
		base_usart_send_byte(low + '0');
		break;
	}
}



void base_usart_send_decimal(int32_t data)
{
	uint32_t val = 0;
	//4294967296
	//10 numbers, 1 sign, zero terminated
	char result[12] = {0,};
	char nr[10] = {0,};
	uint8_t c = 0;
	uint8_t n = 0;
	
	if(data >= 0)
	{
		val = data;
		c= 0;
	}
	else
	{
		val = -data;
		result[c] = '-';
		c = 1;
	}
	
	
	while(val > 0)
	{
		nr[n] = val%10 + '0';
		val /= 10;
		++n;		
	}
	
	if(n == 0)
	{
		nr[0] = val%10 + '0';
		++n;
	}
	
	while(n > 0)
	{
		--n;
		result[c] = nr[n];
		++c;
	}
	result[c] = 0;	
	
	base_usart_send_string(result);
}

void base_usart_send_byte_hex_string(uint8_t* data, uint8_t len)
{
	uint8_t c = 0;
	
	base_usart_send_string("0x");
	for(c = 0; c < len; ++len)
	{		
		base_usart_send_byte_hex(data[c]);
		base_usart_send_byte(' ');
	}
}



