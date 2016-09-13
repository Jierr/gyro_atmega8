/*
 * base_i2c.c
 *
 * Created: 06.08.2016 18:38:45
 *  Author: Jierr
 */ 

#include <avr/interrupt.h>
#include "base_i2c.h"

volatile i2c_ctx_t i2c_ctx;

void base_i2c_init()
{
	//Control Register TWINT TWEA TWSTA TWSTO TWWC TWEN – TWIE
	TWCR = 0b01000101;
	
	//define Pin 4,5 as Input (active pull - up registers)
	PORTC |= (1 << PC4);
	PORTC |= (1 << PC5);
	
	//set bitrate to 100 kb/s -> 16Mhz/160
	TWBR = 0x12;
	TWSR |= (1 << TWPS0);
	TWSR &= ~(1 << TWPS1);
	
	i2c_ctx.twi_state = STATE_TWI_IDLE;
	i2c_ctx.slave = 0;
	i2c_ctx.slave_local_addr = 0;
	i2c_ctx.recv = 0;
	i2c_ctx.trans = 0;
	
}


void base_i2c_set_slave(volatile i2c_ctx_t* ctx, uint8_t addr)
{
	uint8_t state = STATE_TWI_IDLE;
	cli();
	state = ctx->twi_state;
	sei();
	if(state != STATE_TWI_IDLE)
	return;
	
	ctx->slave = addr;
}


void base_i2c_start_read(volatile i2c_ctx_t* ctx, uint8_t slave, uint8_t local, uint8_t* buf, int count)
{
	uint8_t state = STATE_TWI_IDLE;
	uint8_t twcr = TWCR;
	cli();
	state = ctx->twi_state;
	sei();
	if(state != STATE_TWI_IDLE)
	return;
	
	ctx->twi_state = STATE_TWI_READ_START;
	ctx->slave = slave;
	ctx->slave_local_addr = local;
	ctx->recv_buf = buf;
	ctx->bytes = count;
	ctx->curr = 0;
	twcr |= (1 << TWEA);
	twcr &= ~(1 << TWSTO);
	twcr |= (1 << TWSTA);
	twcr |= (1 << TWINT);
	TWCR = twcr;
}


void base_i2c_start_write(volatile i2c_ctx_t* ctx, uint8_t slave, uint8_t local, uint8_t* buf, int count)
{
	uint8_t state = STATE_TWI_IDLE;
	uint8_t twcr = TWCR;
	cli();
	state = ctx->twi_state;
	sei();
	if(state != STATE_TWI_IDLE)
	return;
	
	ctx->twi_state = STATE_TWI_WRITE_START;
	ctx->slave = slave;
	ctx->slave_local_addr = local;
	ctx->trans_buf = buf;
	ctx->bytes = count;
	ctx->curr = 0;
	twcr |= (1 << TWEA);
	twcr &= ~(1 << TWSTO);
	twcr |= (1 << TWSTA);
	twcr |= (1 << TWINT);
	TWCR = twcr;
}



ISR(TWI_vect)
{
	uint8_t state = i2c_ctx.twi_state;
	uint8_t status = 0;
	uint8_t data = 0;
	uint8_t twcr = 0;
	
	
	//uint8_t c = 0;
	//char str[10] = "";
	/*
	if(1)
	{
		snprintf(str, 10, "%d - ", state);
		c = 0;
		while(str[c])
		{
			base_usart_send_byte((uint8_t)str[c]);
			++c;
		}
		status = TWSR & 0xF8;
		snprintf(str, 10, "%X - ", status);
		c = 0;
		while(str[c])
		{
			base_usart_send_byte((uint8_t)str[c]);
			++c;
	}
		snprintf(str, 10, "%X\r\n", TWCR);
		c = 0;
		while(str[c])
		{
			base_usart_send_byte((uint8_t)str[c]);
			++c;
		}
	}
	*/
	switch(state)
	{
		case STATE_TWI_READ_START:
			status = TWSR & 0xF8;
			twcr = TWCR;
			if(status == 0x08)
			{
				//SLA+W
				data = ((i2c_ctx.slave << 1) & 0xFE) + TW_WRITE;
				TWDR = data;
				//TWDR |= 1;
				//TWDR &= ~0x01;
				state = STATE_TWI_READ_ADDR;
			}
			else
				state = STATE_TWI_ERROR;
				
			twcr &= ~(1 << TWSTA);
			twcr |= (1 << TWINT);
			TWCR = twcr; 
			break;
		case STATE_TWI_READ_ADDR:
			status = TWSR & 0xF8;
			twcr = TWCR;
			if(status == 0x18)
			{
				TWDR = i2c_ctx.slave_local_addr;
				state = STATE_TWI_READ_REMOTE_ADDR;
			}
			else
			{
				state = STATE_TWI_ERROR;				
			}
			
			TWCR |= (1 << TWINT);		
			break;
		case STATE_TWI_READ_REMOTE_ADDR:
			status = TWSR & 0xF8;
			twcr = TWCR;
			if(status == 0x28)
			{
				state = STATE_TWI_READ_RSTART;
				twcr |= (1 << TWSTA);
			}
			else
			{
				state = STATE_TWI_ERROR;				
			}
			
			twcr |= (1 << TWINT);	
			TWCR = twcr;	
			break;
		case STATE_TWI_READ_RSTART:
			status = TWSR & 0xF8;
			twcr = TWCR;
			twcr &= ~(1 << TWSTA);
			if(status == 0x10)
			{
				//SLA+R
				TWDR = (i2c_ctx.slave << 1) + TW_READ;
				state = STATE_TWI_READ_RADDR;
			}
			else
				state = STATE_TWI_ERROR;
				
			twcr |= (1 << TWINT);
			TWCR = twcr;
			break;

		case STATE_TWI_READ_RADDR:
			status = TWSR & 0xF8;
			twcr = TWCR;
			if(status == 0x40)
			{
				i2c_ctx.curr = 0;
				if(i2c_ctx.curr >= (i2c_ctx.bytes - 1))
					twcr &= ~(1 << TWEA);
				else
					twcr |= (1 << TWEA);
				state = STATE_TWI_READ_DATA;
			}
			else
				state = STATE_TWI_ERROR;
			
			twcr |= (1 << TWINT);
			TWCR = twcr;
			break;
		case STATE_TWI_READ_DATA:
			status = TWSR & 0xF8;
			twcr = TWCR;
			if(status == 0x58)
			{
				twcr |= (1 << TWEA);
				twcr |= (1 << TWSTO);
				i2c_ctx.recv = TWDR;
				i2c_ctx.recv_buf[i2c_ctx.curr] = i2c_ctx.recv;
				++i2c_ctx.curr;
				
				/*
				snprintf(str, 10, "\t%X\r\n", i2c_ctx.recv);
				c = 0;
				while(str[c])
				{
					base_usart_send_byte((uint8_t)str[c]);
					++c;
				}
				*/
				state = STATE_TWI_IDLE;
			}
			else if (status == 0x50)
			{				
				i2c_ctx.recv = TWDR;
				i2c_ctx.recv_buf[i2c_ctx.curr] = i2c_ctx.recv;
				++i2c_ctx.curr;
				
				if(i2c_ctx.curr >= (i2c_ctx.bytes - 1))
					twcr &= ~(1 << TWEA);
				else
					twcr |= (1 << TWEA);
				
				/*
				snprintf(str, 10, "\t%X\r\n", i2c_ctx.recv);
				c = 0;
				while(str[c])
				{
					base_usart_send_byte((uint8_t)str[c]);
					++c;
				}
				*/
				
				state = STATE_TWI_READ_DATA;
			}
			else
				state = STATE_TWI_ERROR;

			twcr |= (1 << TWINT);
			TWCR = twcr;
			break;
			
		
		case STATE_TWI_WRITE_START:
			status = TWSR & 0xF8;
			twcr = TWCR;
			if(status == 0x08)
			{
				//SLA+W
				data = ((i2c_ctx.slave << 1) & 0xFE) + TW_WRITE;
				TWDR = data;
				state = STATE_TWI_WRITE_ADDR;
			}
			else
				state = STATE_TWI_ERROR;
			
			twcr &= ~(1 << TWSTA);
			twcr |= (1 << TWINT);
			TWCR = twcr;
			break;
		case STATE_TWI_WRITE_ADDR:
			status = TWSR & 0xF8;
			twcr = TWCR;
			if(status == 0x18)
			{
				TWDR = i2c_ctx.slave_local_addr;
				state = STATE_TWI_WRITE_REMOTE_ADDR;
			}
			else
			{
				state = STATE_TWI_ERROR;
			}
			
			TWCR |= (1 << TWINT);
			break;
		case STATE_TWI_WRITE_REMOTE_ADDR:
			status = TWSR & 0xF8;
			twcr = TWCR;
			if(status == 0x28)
			{
				twcr |= (1 << TWEA);
				i2c_ctx.curr = 0;
				TWDR = i2c_ctx.trans_buf[i2c_ctx.curr];
				++i2c_ctx.curr;
				state = STATE_TWI_WRITE_DATA;				
			}
			else
			{
				state = STATE_TWI_ERROR;
			}
			
			twcr |= (1 << TWINT);
			TWCR = twcr;
			break;
		case STATE_TWI_WRITE_DATA:
			status = TWSR & 0xF8;
			twcr = TWCR;
			if(status == 0x28)
			{
				if(i2c_ctx.curr < i2c_ctx.bytes)
				{
					twcr |= (1 << TWEA);
					TWDR = i2c_ctx.trans_buf[i2c_ctx.curr];
					++i2c_ctx.curr;
					state = STATE_TWI_WRITE_DATA;					
				}
				else
				{
					twcr |= (1 << TWEA);
					twcr |= (1 << TWSTO);
					state = STATE_TWI_IDLE;					
				}
			}
			
			twcr |= (1 << TWINT);
			TWCR = twcr;
			break;	
			
			
		case STATE_TWI_ERROR:
			twcr = TWCR;
			twcr |= (1 << TWSTO);
			twcr |= (1 << TWINT);
			state = STATE_TWI_IDLE;
			TWCR = twcr;
			break;
		case STATE_TWI_IDLE:
		default:
			twcr = TWCR;
			twcr &= ~(1 << TWSTO);
			TWCR &= ~(1 << TWINT);
			break;
		
	}
	i2c_ctx.twi_state = state;
}


void base_i2c_wait()
{
	while(i2c_ctx.twi_state != STATE_TWI_IDLE)
	;
}

uint8_t base_i2c_is_ready()
{
	return (i2c_ctx.twi_state == STATE_TWI_IDLE);	
}