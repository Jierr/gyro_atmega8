/*
 * _7segment.h
 *
 * Created: 06.08.2016 19:00:08
 *  Author: Jierr
 */ 


#ifndef _7SEGMENT_H_
#define _7SEGMENT_H_

#include "config.h"




#define B0_ON ~((1<<PB3) | (1<<PB2) | (1<<PB1))
#define B0_OFF (1 << PB4)
#define B1_ON ~((1<<PB1))
#define B1_OFF ((1 << PB4) | (1<<PB3) | (1<<PB2))
#define B2_ON ~((1<<PB2) | (1<<PB1) | (1 << PB4))
#define B2_OFF (1 << PB3)
#define B3_ON ~((1<<PB2) | (1<<PB1) | (1 << PB4))
#define B3_OFF ((1<<PB3))
#define B4_ON ~((1<<PB3) | (1<<PB4) | (1<<PB1))
#define B4_OFF ((1 << PB2))
#define B5_ON ~((1<<PB4) | (1<<PB3) | (1<<PB2))
#define B5_OFF (1 << PB1)
#define B6_ON ~((1<<PB4) | (1<<PB3) | (1<<PB2))
#define B6_OFF (1 << PB1)
#define B7_ON ~((1<<PB2) | (1<<PB1))
#define B7_OFF ((1<<PB3) | (1<<PB4))
#define B8_ON ~((1<<PB3) | (1<<PB2) | (1<<PB1) | (1 << PB4))
#define B8_OFF 0
#define B9_ON ~((1<<PB3) | (1<<PB2) | (1<<PB1) | (1 << PB4))
#define B9_OFF 0


#define C0_ON ~((1<<PC3) | (1<<PC2) | (1<<PC1))
#define C0_OFF (1 << PC0)
#define C1_ON ~((1<<PC2))
#define C1_OFF ((1 << PC0) | (1<<PC3) | (1<<PC1))
#define C2_ON ~((1<<PC3) | (1<<PC1))
#define C2_OFF (1 << PC2 | (1 << PC0))
#define C3_ON ~((1<<PC2) | (1<<PC1))
#define C3_OFF ((1<<PC3) | (1 << PC0))
#define C4_ON ~((1<<PC2))
#define C4_OFF ((1 << PC1) | (1 << PC0) | (1 << PC3))
#define C5_ON ~((1<<PC2) | (1<<PC1))
#define C5_OFF (1 << PC3 | (1<<PC0))
#define C6_ON ~((1<<PC3) | (1<<PC2) | (1<<PC1))
#define C6_OFF (1 << PC0)
#define C7_ON ~((1<<PC2))
#define C7_OFF ((1<<PC3) | (1<<PC0) | (1<<PC1))
#define C8_ON ~((1<<PC3) | (1<<PC2) | (1<<PC1))
#define C8_OFF (1 << PC0)
#define C9_ON ~((1<<PC2) | (1<<PC1))
#define C9_OFF ((1 << PC0) | (1<<PC3))


void abstract_7segment_display(uint8_t s1, uint8_t s2);
void abstract_7segment_init();




#endif /* _7SEGMENT_H_ */