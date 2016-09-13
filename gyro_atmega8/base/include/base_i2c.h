/*
 * base_i2c.h
 *
 * Created: 06.08.2016 18:38:23
 *  Author: Jierr
 */ 


#ifndef BASE_I2C_H_
#define BASE_I2C_H_

#include "config.h"
#include <util/twi.h>

#define STATE_TWI_IDLE 0
#define STATE_TWI_ERROR 1
#define STATE_TWI_WRITE_START 10
#define STATE_TWI_WRITE_ADDR 11
#define STATE_TWI_WRITE_REMOTE_ADDR 12
#define STATE_TWI_WRITE_DATA 13
#define STATE_TWI_READ_START 20
#define STATE_TWI_READ_ADDR 21
#define STATE_TWI_READ_REMOTE_ADDR 22
#define STATE_TWI_READ_RSTART 23
#define STATE_TWI_READ_RADDR 24
#define STATE_TWI_READ_DATA 25


#define TWI_SLA_MPU6050 0x68


typedef struct
{
	uint8_t twi_state;
	uint8_t slave;
	uint8_t slave_local_addr;
	uint8_t recv;
	uint8_t trans;
	uint8_t bytes;
	uint8_t* recv_buf;
	uint8_t* trans_buf;
	uint8_t curr;
} i2c_ctx_t;

extern volatile i2c_ctx_t i2c_ctx;

void base_i2c_init();
void base_i2c_set_slave(volatile i2c_ctx_t* ctx, uint8_t addr);
void base_i2c_start_read(volatile i2c_ctx_t* ctx, uint8_t slave, uint8_t local, uint8_t* buf, int count);
void base_i2c_start_write(volatile i2c_ctx_t* ctx, uint8_t slave, uint8_t local, uint8_t* buf, int count);
void base_i2c_wait();
uint8_t base_i2c_is_ready();
uint8_t base_i2c_read(uint8_t slave, uint8_t adr);

ISR(TWI_vect);



#endif /* BASE_I2C_H_ */