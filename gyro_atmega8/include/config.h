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


#endif /* CONFIG_H_ */