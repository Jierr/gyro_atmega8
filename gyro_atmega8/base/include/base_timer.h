/*
 * base_timer.h
 *
 * Created: 06.08.2016 19:52:23
 *  Author: Jierr
 */ 


#ifndef BASE_TIMER_H_
#define BASE_TIMER_H_

typedef struct base_timer_context
{
	uint16_t top;	
	uint8_t pwm_duty;
	uint8_t invertiert;
}base_timer_context_t;

extern volatile base_timer_context_t base_timer_context;

void base_timer_init();
void base_timer_set_pwm_frequency();
void base_timer_set_pwm_duty(uint8_t percentage);
void base_timer_set_pwm_change_duty_by(int8_t percentage);
void base_timer_set_pwm_level(uint8_t high);



#endif /* BASE_TIMER_H_ */