/*
 * base_timer.h
 *
 * Created: 06.08.2016 19:52:23
 *  Author: Jierr
 */ 


#ifndef BASE_TIMER_H_
#define BASE_TIMER_H_

#define BASE_TIMER0_PRESCALER (256)

typedef struct base_timer1_context
{
	uint16_t top;	
	uint8_t pwm_duty;
	uint8_t invertiert;
}base_timer1_context_t;

typedef struct base_timer0_context
{
	uint32_t tick;
	uint16_t prescaler;
	uint32_t ms_tick_count;
	uint32_t s_tick_count;
} base_timer0_context_t;

extern volatile base_timer1_context_t base_timer1_context;
extern volatile base_timer0_context_t base_timer0_context;


void base_timer0_init();
uint32_t base_timer0_ms_ticks(uint32_t ms);

void base_timer1_init();
void base_timer1_set_pwm_frequency();
void base_timer1_set_pwm_duty(uint8_t percentage);
void base_timer1_set_pwm_change_duty_by(int8_t percentage);
void base_timer1_set_pwm_level(uint8_t high);



#endif /* BASE_TIMER_H_ */