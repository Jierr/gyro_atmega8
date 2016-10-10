/*
 * config.h
 *
 * Created: 06.08.2016 18:36:30
 *  Author: Jierr
 */ 


#ifndef CONFIG_H_
#define CONFIG_H_


/*clock: Timer, Recv: interrupt, transmission: synchron*/
#define F_CPU 16000000UL
#define BAUD 500000UL
#define UF_CPU 16000000UL
#define MUBRR (UF_CPU/((BAUD*16)) - 1)

#define CONFIG_DEBUG_GEN 1
#define CONFIG_DEBUG_MP6050 0


#include <avr/io.h>

typedef enum {
	BASE_PORTB = 0,
	BASE_PORTC = 0,
	BASE_PORTD = 0,
} port_type_t;

typedef enum {
	BASE_PIN1 = 0x01,
	BASE_PIN2 = 0x02,
	BASE_PIN3 = 0x04,
	BASE_PIN4 = 0x08,
	BASE_PIN5 = 0x10,
	BASE_PIN6 = 0x20,
	BASE_PIN7 = 0x40,
	BASE_PIN8 = 0x80,
	
} pin_type_t;


#endif /* CONFIG_H_ */