/*
 * base_usart.h
 *
 * Created: 06.08.2016 19:13:09
 *  Author: Jierr
 */


#ifndef BASE_USART_H_
#define BASE_USART_H_

#include "config.h"


void base_usart_send_byte(uint8_t data);
void base_usart_send_string(char* str);
void base_usart_send_byte_hex(uint8_t data);
void base_usart_send_byte_hex_string(uint8_t* data, uint8_t len);
void base_usart_send_decimal(int32_t data);
void base_usart_init(uint16_t ubrr);


#endif /* BASE_USART_H_ */