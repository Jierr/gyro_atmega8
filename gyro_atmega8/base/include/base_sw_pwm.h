/*
 * base_sw_pwm.h
 *
 * Created: 11.10.2016 00:02:22
 *  Author: Jierr
 */


#ifndef BASE_SW_PWM_H_
#define BASE_SW_PWM_H_

#include "config.h"
#include "base_timer.h"


typedef struct base_sw_pwm_config
{
  port_type_t port;
  pin_type_t pin;
  uint8_t pwm_duty;
  uint16_t pwm_duty_ticks;
} base_sw_pwm_config_t;

typedef struct base_sw_pwm_ctx
{
  base_sw_pwm_config_t pin[BASE_MAX_PINS];
  uint8_t config_count;
  uint16_t cycle_tick_count;
  uint16_t cycle_tick;

} base_sw_pwm_ctx_t;

extern volatile base_sw_pwm_ctx_t base_sw_pwm_ctx;

void base_sw_pwm_ctx_init();
uint16_t base_sw_pwm_set_global_cycle(uint16_t hz);
uint8_t base_sw_pwm_init(port_type_t port, pin_type_t pin);
void base_sw_pwm_set_duty(uint8_t pin_nr, int8_t percent);
void base_sw_pwm_duty(uint8_t pin_nr, int8_t percent);
void base_sw_pwm_timer0_callback();



#endif /* BASE_SW_PWM_H_ */