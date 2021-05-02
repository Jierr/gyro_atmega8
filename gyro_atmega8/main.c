#include "config.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include <util/delay.h>
#include <stdio.h>
#include <math.h>


#include "base_i2c.h"
#include "base_usart.h"
#include "base_timer.h"
#include "base_sw_pwm.h"
#include "abstract_7segment.h"
#include <avr/interrupt.h>

#define UART_MAX_LENGTH 80
#define I2C_RECEIVE_BUFFER_LENGTH 8
#define I2C_TRANSMIT_BUFFER_LENGTH 8
#define MP6050_GYRO_ERROR_ACCUMULATION 100
#define MP6050_GYRO_ERROR_DISCARD 200
#define MP6050_ACCEL_ERROR_ACCUMULATION 100
#define MP6050_ACCEL_ERROR_DISCARD 20
#define MP6050_READ_INTERVAL_MS 10

/*
0 250°/s  131 LSB/°/s
1 500°/s  65.5 LSB/°/s
2 1000°/s 32.8 LSB/°/s
3 2000°/s 16.4 LSB/°/s
*/
#define MP6050_GYRO_VALUE_SCALING 32.8f

/*
0 2g  16384 LSB/g
1 4g  8192 LSB/g
2 8g  4096 LSB/g
3 16g 2048 LSB/g
*/
#define MP6050_ACCEL_VALUE_SCALING 16384.0f

#define VIEW_GYRO_HIGHPASS  0.98f
#define VIEW_ACCEL_LOWPASS  (1.0f - VIEW_GYRO_HIGHPASS)

uint8_t i2c_transmit_buffer[I2C_TRANSMIT_BUFFER_LENGTH] = {0,};
uint8_t i2c_receive_buffer[I2C_RECEIVE_BUFFER_LENGTH] = {0,};
char usart_buffer[UART_MAX_LENGTH] = {0,};
volatile uint8_t usart_data_in = 0;
volatile uint16_t timer2_milliseconds = 0;
volatile uint8_t timer2_seconds = 0;
volatile uint8_t timer2_minutes = 0;
volatile uint8_t timer2_hours = 0;

/* signals, ISR */
volatile uint8_t signal_100ms_event = 0;
volatile uint8_t signal_10ms_event = 0;

typedef struct
{
  short x;
  short y;
  short z;
  int32_t xerror;
  int32_t yerror;
  int32_t zerror;
  unsigned short error_init_cycles;
} mp6050_gyro_t;

typedef struct
{
  float xroll;
  float ypitch;
  float zyaw;
} angle_t;

typedef struct
{
  float x;
  float y;
  float z;
} angular_velocity_t;

typedef struct
{
  short x;
  short y;
  short z;
  int32_t xerror;
  int32_t yerror;
  int32_t zerror;
  unsigned short error_init_cycles;
} mp6050_accel_t;

typedef struct
{
  float x;
  float y;
  float z;
} force_t;

typedef struct
{
  uint8_t descriptor; //< pin descriptor for PWM driver
  uint8_t duty; //< current motor duty 0..100%
} motor_t;

typedef struct
{
  motor_t ltop;
  motor_t rtop;
  motor_t lbottom;
  motor_t rbottom;
} motors_t;

typedef enum
{
  STATE_STOP = 0,
  STATE_HOLD_HOVER,
  STATE_LIFT_OFF,
  STATE_FLIGHT,
  STATE_LAND,
} control_state_t;

typedef struct
{
  motors_t motors;
  uint8_t config_hover_duty;
  uint8_t config_max_acceleration_duty;
  uint8_t config_min_deceleration_duty;
  int8_t config_acceleration_angle;
  int8_t config_deceleration_angle;
  control_state_t state;
} control_t;

void motors_init(motors_t* motors)
{
  base_sw_pwm_set_global_cycle(0);
  motors->rtop.descriptor = base_sw_pwm_init(BASE_PORTB, BASE_PIN1);
  motors->rtop.duty = 0;
  motors->ltop.descriptor = base_sw_pwm_init(BASE_PORTB, BASE_PIN3);
  motors->ltop.duty = 0;
  motors->rbottom.descriptor = base_sw_pwm_init(BASE_PORTD, BASE_PIN8);
  motors->rbottom.duty = 0;
  motors->lbottom.descriptor = base_sw_pwm_init(BASE_PORTB, BASE_PIN2);
  motors->lbottom.duty = 0;

  base_sw_pwm_set_duty(motors->ltop.descriptor, 0);
  base_sw_pwm_set_duty(motors->rtop.descriptor, 0);
  base_sw_pwm_set_duty(motors->lbottom.descriptor, 0);
  base_sw_pwm_set_duty(motors->rbottom.descriptor, 0);
}

void motors_set_all(motors_t* motors, uint8_t duty)
{
  uint8_t balance_duty = 0;
  if (duty > 30)
    balance_duty  = duty - 30;
  base_sw_pwm_set_duty(motors->ltop.descriptor, duty);
  base_sw_pwm_set_duty(motors->rtop.descriptor, duty);
  base_sw_pwm_set_duty(motors->lbottom.descriptor, duty);
  base_sw_pwm_set_duty(motors->rbottom.descriptor, duty);
}


void control_init(control_t* control, uint8_t hover_duty)
{
  motors_init(&control->motors);
  control->config_hover_duty = hover_duty;
  control->config_acceleration_angle = 60;
  control->config_deceleration_angle = 60;
  control->config_max_acceleration_duty = 100;
  control->config_min_deceleration_duty = hover_duty >> 1;
  control->state = STATE_HOLD_HOVER;
}

// This will put the quadrocoptor in hover 1.5m above ground
void control_init_sequence(control_t* control, const angle_t* angle,
                           const mp6050_accel_t* accel, int16_t* hold)
{
  if(control->state >= STATE_FLIGHT)
    return;

  short vertical_force = accel->z - accel->zerror / MP6050_ACCEL_ERROR_ACCUMULATION;
  if (control->state == STATE_HOLD_HOVER)
  {
    --(*hold);
    // if lift is not at least g force increase
    if (vertical_force > 0)
    {
      if(control->config_hover_duty < control->config_max_acceleration_duty)
      {
        ++control->config_hover_duty;
      }
      motors_set_all(&control->motors, control->config_hover_duty);
    }

    if (*hold <= 0)
    {
      control->state = STATE_LIFT_OFF;
      control->config_min_deceleration_duty = control->config_hover_duty >> 1;
    }

    // if lift is too strong, decrease
    if(vertical_force < 0)
    {
      if(control->config_hover_duty > 0)
      {
        --control->config_hover_duty;
      }
      motors_set_all(&control->motors, control->config_hover_duty);
    }

  }
  else if(control->state == STATE_LIFT_OFF)
  {
    uint8_t lift_duty = 100; //control->config_hover_duty + 40;
    if (lift_duty > control->config_max_acceleration_duty)
    {
      lift_duty = control->config_max_acceleration_duty;
    }

    ++(*hold);
    if(*hold > 300)
      control->state = STATE_STOP;

    motors_set_all(&control->motors, lift_duty);
  }
  else if(control->state == STATE_STOP)
  {
    control->config_hover_duty = 5;
    motors_set_all(&control->motors, control->config_hover_duty);
  }
}

void control(control_t* control, const angle_t* angle, const mp6050_accel_t* accel)
{
  if(control->state < STATE_FLIGHT)
    return;
  control_hover(angle, accel);
}

void control_hover(const angle_t* angle, const mp6050_accel_t* accel)
{
}


int mp6050_get_gyro(mp6050_gyro_t* raw);
int mp6050_get_gyro_error(mp6050_gyro_t* gyro);
int mp6050_get_accel(mp6050_accel_t* raw);
int mp6050_get_accel_error(mp6050_accel_t* accel);
void view_calc_current_angles(angle_t* angle, angular_velocity_t* angular_velocity,
                              const mp6050_gyro_t* gyro, const mp6050_accel_t* accel, short delta_time_milliseconds);
void view_calc_current_forces(force_t* force, const mp6050_accel_t* accel,
                              short delta_time_milliseconds);

void mp6050_print_info();


/*fips = interrupts per second*/
inline void timer2_ctc_interrupt_init(uint32_t fips, uint16_t prescaler, uint8_t async)
{
  uint8_t sreg = SREG;
  cli();

  if (async)
    ASSR |= (1 << AS2);
  else
    ASSR &= ~(1 << AS2);

  TCCR2 = (1 << WGM21);
  /* OCR = 249: fips = 1000, prescaler = 64
  ->interrupt every 1ms */
  OCR2 = (uint8_t)((F_CPU / (prescaler * fips)) - 1);
  TCNT2 = 0x00; /* Timer disabled */

  /* CS22:0 */
  switch (prescaler)
  {
  case 1:
    TCCR2 |= 0x01;
    break;
  case 8:
    TCCR2 |= 0x02;
    break;
  case 32:
    TCCR2 |= 0x03;
    break;
  case 64:
    TCCR2 |= 0x04;
    break;
  case 128:
    TCCR2 |= 0x05;
    break;
  case 256:
    TCCR2 |= 0x06;
    break;
  case 1024:
    TCCR2 |= 0x07;
    break;
  default:
    TCCR2 |= 0x00;
    break;
  }

  /* wait for everything to finish*/
  if (async)
  {
    while((ASSR & (1 << TCN2UB)) || (ASSR & (1 << OCR2UB)) || (ASSR & (1 << TCR2UB)))
      ;
  }

  /* clear interrupt flags */
  TIFR &= ~((1 << OCF2) | (1 << TOV2));
  /* enable compare match interrupt */
  TIMSK |= (1 << OCIE2);
  SREG = sreg;
}



ISR(USART_RXC_vect)
{

  uint8_t sreg = SREG;
  cli();

#if 0
  DDRB |= (1 << DDB0);
  if(PORTB & (1 << PB0))
    PORTB &= ~(1 << PB0);
  else
    PORTB |= (1 << PB0);
#endif


  usart_data_in = UDR;
  switch (usart_data_in)
  {
  case 'y':
    //base_timer1_set_pwm_change_duty_by(-1);

    base_sw_pwm_duty(0, -1);
    base_sw_pwm_duty(1, -1);
    base_sw_pwm_duty(2, -1);
    base_sw_pwm_duty(3, -1);


#if CONFIG_DEBUG_GEN
    base_usart_send_string(":ISR(USART_RXC_vect):-1> ");
    base_usart_send_decimal(base_sw_pwm_ctx.pin[0].pwm_duty);
    base_usart_send_string(" pwm_duty\r\n");
#endif
    break;
  case 'x':
    //base_timer1_set_pwm_change_duty_by(1);

    base_sw_pwm_duty(0, 1);
    base_sw_pwm_duty(1, 1);
    base_sw_pwm_duty(2, 1);
    base_sw_pwm_duty(3, 1);

#if CONFIG_DEBUG_GEN
    base_usart_send_string(":ISR(USART_RXC_vect):+1> ");
    base_usart_send_decimal(base_sw_pwm_ctx.pin[0].pwm_duty);
    base_usart_send_string(" pwm_duty\r\n");
#endif
    break;

  case '0':
    base_sw_pwm_set_duty(0, 0);
    base_sw_pwm_set_duty(1, 0);
    base_sw_pwm_set_duty(2, 0);
    base_sw_pwm_set_duty(3, 0);
    break;
  case '1':
    base_sw_pwm_set_duty(0, 100);
    base_sw_pwm_set_duty(1, 100);
    base_sw_pwm_set_duty(2, 100);
    base_sw_pwm_set_duty(3, 100);
    break;
  default:
    break;
  }
  SREG = sreg;
}


ISR(TIMER0_OVF_vect)
{
  uint8_t sreg = SREG;
  cli();
  uint32_t mod;
  uint32_t a, b;

  base_timer0_context.tick += 1;
  base_sw_pwm_timer0_callback();

#if 0
  a = base_timer0_context.tick;
  b = base_timer0_s_ticks(1);
  mod = a % b;
  if(mod == 0)
  {
    DDRB |= (1 << DDB0);
    if(PORTB & (1 << PB0))
      PORTB &= ~(1 << PB0);
    else
      PORTB |= (1 << PB0);
  }
#endif

  SREG = sreg;
}

ISR(TIMER2_COMP_vect)
{
#if CONFIG_TIME == 1
  static uint8_t msTimer = 0;
  msTimer += 1;

  if(msTimer % 10 == 0)
  {
    signal_10ms_event = 1;
  }

  if (msTimer == 100)
  {
    timer2_milliseconds = (timer2_milliseconds + 100) % 1000;
    if (!timer2_milliseconds)
    {
      timer2_seconds = (timer2_seconds + 1) % 60;
      if (!timer2_seconds)
      {
        timer2_minutes = (timer2_minutes + 1) % 60;
        if (!timer2_minutes)
          timer2_hours = (timer2_hours + 1) % 24;
      }
    }

    signal_100ms_event = 1;
    msTimer = 0;
  }
#endif

  //abstract_7segment_display(sec/10, sec%10);
}


int main()
{
  control_t control;
  int hold = 200;
  cli();
  DDRB = 0;
  DDRC = 0;
  DDRD = 0;
  PORTB = 0;
  PORTC = 0;
  PORTD = 0;
  //led
  DDRB |= (1 << DDB0);
  //base_timer1_init();
  /* OCR = 249, 1 Interrupt per milisecond --> 1000 ips ISR = TIMER2_COMP_vect */
  timer2_ctc_interrupt_init(1000UL, 64, 0);
  base_usart_init(MUBRR);
  //7segment
  //abstract_7segment_init();
  base_sw_pwm_ctx_init();
  base_timer0_init();
  //bottom right pins
  control_init(&control, 30);
  sei();

  base_usart_send_string("Initializing.\r\n");


#if CONFIG_DEBUG_GEN
  base_usart_send_string("TIMER0 Ticks pro Sekunde: '");
  base_usart_send_decimal(base_timer0_context.s_tick_count);
  base_usart_send_string("'\r\n");
  base_usart_send_string("TIMER0 Ticks pro Millisekunde: '");
  base_usart_send_decimal(base_timer0_context.ms_tick_count);
  base_usart_send_string("'\r\n");
  base_usart_send_string("PWM  Max Cycle Ticks: '");
  base_usart_send_decimal(base_sw_pwm_ctx.cycle_tick_count);
  base_usart_send_string("'\r\n");
  base_usart_send_string("PWM Current Cycle Tick: '");
  base_usart_send_decimal(base_sw_pwm_ctx.cycle_tick);
  base_usart_send_string("'\r\n");
  base_usart_send_string("PWM Motoren Duty Ticks: '");
  base_usart_send_string("'\r\n0: '");
  base_usart_send_decimal(base_sw_pwm_ctx.pin[motor[0]].pwm_duty_ticks);
  base_usart_send_string("'\r\n1: '");
  base_usart_send_decimal(base_sw_pwm_ctx.pin[motor[1]].pwm_duty_ticks);
  base_usart_send_string("'\r\n2: '");
  base_usart_send_decimal(base_sw_pwm_ctx.pin[motor[2]].pwm_duty_ticks);
  base_usart_send_string("'\r\n3: '");
  base_usart_send_decimal(base_sw_pwm_ctx.pin[motor[3]].pwm_duty_ticks);
  base_usart_send_string("'\r\n");
  snprintf(usart_buffer, UART_MAX_LENGTH, "Initialize I2C\r\n");
  base_usart_send_string(usart_buffer);
#endif
  base_i2c_init();
  base_i2c_set_slave(&i2c_ctx, TWI_SLA_MPU6050);
  base_i2c_wait();
  base_i2c_start_read(&i2c_ctx, TWI_SLA_MPU6050, 0x75, i2c_receive_buffer, 1);
  base_i2c_wait();

#if CONFIG_DEBUG_GEN
  snprintf(usart_buffer, UART_MAX_LENGTH, "I2C Identity %X\r\n", i2c_receive_buffer[0]);
  base_usart_send_string(usart_buffer);
#endif
  // Configure Power Up
  i2c_transmit_buffer[0] = 1;
  base_i2c_start_write(&i2c_ctx, TWI_SLA_MPU6050, 0x6B, i2c_transmit_buffer, 1);
  base_i2c_wait();
  // Configure Gyro resolution: [deg/s] 0x00 -> 250, 0x08 -> 500, 0x10 -> 1000, 0x18 -> 2000
  i2c_transmit_buffer[0] = 0x10;
  base_i2c_start_write(&i2c_ctx, TWI_SLA_MPU6050, 0x1B, i2c_transmit_buffer, 1);
  base_i2c_wait();

  /*while loop only does tasks every 100ms*/
  //base_usart_send_string("Initialization complete.\r\n");


  uint8_t ms100 = 0;
  mp6050_gyro_t raw_gyro = {0, 0, 0, 0, 0, 0, 0};
  mp6050_accel_t raw_accel = {0, 0, 0, 0, 0, 0, 0};
  angle_t angle = {0.0f, 0.0f, 0.0f};
  angular_velocity_t angular_velocity = {0.0f, 0.0f, 0.0f};

  base_usart_send_string("Init complete. Entering Mainloop.\r\n");
  while (1)
  {
    uint16_t sms = 0;
    uint8_t ssec = 0;
    uint8_t smin = 0;
    uint8_t shour = 0;
    if (signal_100ms_event)
    {
      /*when preparing components for string no interrupts allowed*/
      ms100 = 1;
      cli();
      signal_100ms_event = 0;

      sms = timer2_milliseconds;
      ssec = timer2_seconds;
      smin = timer2_minutes;
      shour = timer2_hours;

      sei();
    }

    if(signal_10ms_event)
    {
      signal_10ms_event = 0;
      int error = 0;
      if((mp6050_get_gyro(&raw_gyro) == 0) && (mp6050_get_accel(&raw_accel) == 0))
      {
        error = mp6050_get_accel_error(&raw_accel);
        error += mp6050_get_gyro_error(&raw_gyro);
      }

      if(error == 0)
      {
        view_calc_current_angles(&angle, &angular_velocity, &raw_gyro, &raw_accel,
                                 MP6050_READ_INTERVAL_MS);

        if(ms100 == 1)
        {
          control_init_sequence(&control, &angle, &raw_accel, &hold);
          ms100 = 0;
          base_usart_send_string("Current Duty: ");
          base_usart_send_decimal(control.config_hover_duty);
          base_usart_send_string("\r\nState: ");
          base_usart_send_decimal(control.state);
          base_usart_send_string("\r\n");
        }

      }
    }


#if CONFIG_DEBUG_MP6050 == 1
    if(ms100 == 5)
    {
      ms100 = 0;
      //snprintf(usart_buffer, UART_MAX_LENGTH, "\033[0Jx = %f\r\ny = %f\r\nz = %f\r\n\r\033[3A",
      //         angle.xroll, angle.ypitch, angle.zyaw);
      snprintf(usart_buffer, UART_MAX_LENGTH, "\033[0Jx = %d\r\ny = %d\r\nz = %d\r\n\r\033[3A",
               (int)(angle.xroll * 100.0f), (int)(angle.ypitch * 100.0f), (int)(angle.zyaw * 100.0f));
      base_usart_send_string(usart_buffer);

#if 0
      // Clear everything from cursor downwards
      base_usart_send_string("\033[0J");
      snprintf(usart_buffer, UART_MAX_LENGTH, "%02d:%02d:%02d:%03d\r\n", shour, smin, ssec,
               sms);
      base_usart_send_string(usart_buffer);
      mp6050_print_info();
      // Place cursor to the beginning
      base_usart_send_string("\r\033[4A");
#endif
    }
#endif
  }
  return 0;
}


int mp6050_get_gyro(mp6050_gyro_t* raw)
{
  base_i2c_start_read(&i2c_ctx, TWI_SLA_MPU6050, 0x43, i2c_receive_buffer, 6);
  base_i2c_wait();

  raw->x = (i2c_receive_buffer[0] << 8) | i2c_receive_buffer[1];
  raw->y = (i2c_receive_buffer[2] << 8) | i2c_receive_buffer[3];
  raw->z = (i2c_receive_buffer[4] << 8) | i2c_receive_buffer[5];

  return 0;
}

int mp6050_get_accel(mp6050_accel_t* raw)
{
  if(base_i2c_is_ready())
  {
    base_i2c_start_read(&i2c_ctx, TWI_SLA_MPU6050, 0x3B, i2c_receive_buffer, 6);
    base_i2c_wait();

    raw->x = (i2c_receive_buffer[0] << 8) | i2c_receive_buffer[1];
    raw->y = (i2c_receive_buffer[2] << 8) | i2c_receive_buffer[3];
    raw->z = (i2c_receive_buffer[4] << 8) | i2c_receive_buffer[5];
  }
  else
  {
    return -1;
  }
  return 0;
}

int mp6050_get_accel_error(mp6050_accel_t* accel)
{
  if(accel->error_init_cycles < MP6050_ACCEL_ERROR_DISCARD)
  {
    ++accel->error_init_cycles;
    return -1;
  }
  if(accel->error_init_cycles < MP6050_ACCEL_ERROR_DISCARD +
      MP6050_ACCEL_ERROR_ACCUMULATION)
  {
    ++accel->error_init_cycles;
    accel->xerror += accel->x;
    accel->yerror += accel->y;
    accel->zerror += accel->z - (int32_t)MP6050_ACCEL_VALUE_SCALING;

    return -1;
  }
  return 0;
}

int mp6050_get_gyro_error(mp6050_gyro_t* gyro)
{
  if(gyro->error_init_cycles < MP6050_GYRO_ERROR_DISCARD)
  {
    ++gyro->error_init_cycles;
    return -1;
  }
  if(gyro->error_init_cycles < MP6050_GYRO_ERROR_DISCARD + MP6050_GYRO_ERROR_ACCUMULATION)
  {
    ++gyro->error_init_cycles;
    gyro->xerror += gyro->x;
    gyro->yerror += gyro->y;
    gyro->zerror += gyro->z;

    return -1;
  }
  return 0;
}

void view_calc_current_angles(angle_t* angle, angular_velocity_t* angular_velocity,
                              const mp6050_gyro_t* gyro, const mp6050_accel_t* accel, short delta_time_milliseconds)
{
  float delta = (float)delta_time_milliseconds * 0.001f;
  float xerror = (float)gyro->xerror;
  float yerror = (float)gyro->yerror;
  float zerror = (float)gyro->zerror;

  float xAccel = ((float)accel->x - (float)accel->xerror /
                  MP6050_ACCEL_ERROR_ACCUMULATION) / MP6050_ACCEL_VALUE_SCALING;
  float yAccel = ((float)accel->y - (float)accel->yerror /
                  MP6050_ACCEL_ERROR_ACCUMULATION) / MP6050_ACCEL_VALUE_SCALING;
  float zAccel = ((float)accel->z - (float)accel->zerror /
                  MP6050_ACCEL_ERROR_ACCUMULATION) / MP6050_ACCEL_VALUE_SCALING;

  float accelRoll = 180.0f * atan2 (yAccel, zAccel) / M_PI;
  float accelPitch = 180.0f * atan2 (-xAccel, sqrt(yAccel * yAccel + zAccel *
                                     zAccel)) / M_PI;
  /*float accelYaw = 180.0f * atan (zAccel / sqrt(xAccel * xAccel + zAccel * zAccel)) / M_PI;*/

  angular_velocity->x = 0.5f * (angular_velocity->x + ((float)gyro->x *
                                (float)MP6050_GYRO_ERROR_ACCUMULATION - xerror) / MP6050_GYRO_VALUE_SCALING);
  angular_velocity->y = 0.5f * (angular_velocity->y + ((float)gyro->y *
                                (float)MP6050_GYRO_ERROR_ACCUMULATION - yerror) / MP6050_GYRO_VALUE_SCALING);
  angular_velocity->z = 0.5f * (angular_velocity->z + ((float)gyro->z *
                                (float)MP6050_GYRO_ERROR_ACCUMULATION - zerror) / MP6050_GYRO_VALUE_SCALING);

  angle->xroll = VIEW_GYRO_HIGHPASS * (angle->xroll + (angular_velocity->x * delta /
                                       (float)MP6050_GYRO_ERROR_ACCUMULATION)) + VIEW_ACCEL_LOWPASS * accelRoll;
  angle->ypitch = VIEW_GYRO_HIGHPASS * (angle->ypitch + (angular_velocity->y * delta /
                                        (float)MP6050_GYRO_ERROR_ACCUMULATION)) + VIEW_ACCEL_LOWPASS * accelPitch;
  angle->zyaw = (angle->zyaw + (angular_velocity->z * delta / (float)
                                MP6050_GYRO_ERROR_ACCUMULATION));
}

void view_calc_current_forces(force_t* force, const mp6050_accel_t* accel,
                              short delta_time_milliseconds)
{

}

void mp6050_print_info()
{
  short rtemp = 0;
  short rgyro[3] = {0,};
  short raccel[3] = {0,};
  char* conv = 0;
  int32_t tmp32;

  //Gyroscope: Error: (x,y,z) = (-1,2-1)
  if(base_i2c_is_ready())
  {
    base_i2c_wait();
    base_i2c_start_read(&i2c_ctx, TWI_SLA_MPU6050, 0x43, i2c_receive_buffer, 6);
    base_i2c_wait();
    // rgyro: 0,1,2 = around axis x,y,z
    rgyro[0] = (i2c_receive_buffer[0] << 8) | i2c_receive_buffer[1];
    rgyro[1] = (i2c_receive_buffer[2] << 8) | i2c_receive_buffer[3];
    rgyro[2] = (i2c_receive_buffer[4] << 8) | i2c_receive_buffer[5];

    snprintf(usart_buffer, UART_MAX_LENGTH,
             "Gyro: X(%5hi) Y(%5hi) Z(%5hi) - Raw X(%5hi) Y(%5hi) Z(%5hi)",
             (int)((float)rgyro[0] / MP6050_GYRO_VALUE_SCALING),
             (int)((float)rgyro[1] / MP6050_GYRO_VALUE_SCALING),
             (int)((float)rgyro[2] / MP6050_GYRO_VALUE_SCALING), (int)rgyro[0],
             (int)rgyro[1], (int)rgyro[2]);
    base_usart_send_string(usart_buffer);
    base_usart_send_string("\r\n");
  }
  else
  {
    snprintf(usart_buffer, UART_MAX_LENGTH, "I2C NOK: ");
    base_usart_send_string(usart_buffer);
  }

  if(base_i2c_is_ready())
  {
    //Accelerator, Error z-Axis -0.15 to -0.2g, x-Axis -0.03 to -0.07g
    base_i2c_start_read(&i2c_ctx, TWI_SLA_MPU6050, 0x3B, i2c_receive_buffer, 6);
    base_i2c_wait();

    // raccel 0,1,2 = along axis x,y,z
    raccel[0] = (i2c_receive_buffer[0] << 8) | i2c_receive_buffer[1];
    tmp32 = ((int32_t)raccel[0] * 100) / 16384;
    raccel[0] = (short)tmp32;
    raccel[1] = (i2c_receive_buffer[2] << 8) | i2c_receive_buffer[3];
    tmp32 = ((int32_t)raccel[1] * 100) / 16384;
    raccel[1] = (short)tmp32;
    raccel[2] = (i2c_receive_buffer[4] << 8) | i2c_receive_buffer[5];
    tmp32 = ((int32_t)raccel[2] * 100) / 16384;
    raccel[2] = (short)tmp32;

    //
    snprintf(usart_buffer, UART_MAX_LENGTH, "Accel: (%5hi)/100,(%5hi)/100,(%5hi)/100\r\n",
             raccel[0], raccel[1], raccel[2]);
    base_usart_send_string(usart_buffer);
  }
  else
  {
    snprintf(usart_buffer, UART_MAX_LENGTH, "I2C NOK: ");
    base_usart_send_string(usart_buffer);
  }

  if(base_i2c_is_ready())
  {
    //Temperature Error +4 degree Celsius
    base_i2c_start_read(&i2c_ctx, TWI_SLA_MPU6050, 0x41, i2c_receive_buffer, 2);
    base_i2c_wait();
    conv = (char*)&rtemp;
    conv[0] = i2c_receive_buffer[1];
    conv[1] = i2c_receive_buffer[0];
    //rtemp = ((short)recv_buf[0] << 8) | (short)recv_buf[1];
    rtemp = rtemp / 340 + 37 - 4;
    snprintf(usart_buffer, UART_MAX_LENGTH, "Temp:  %5hi\r\n", rtemp);
    base_usart_send_string(usart_buffer);
  }
  else
  {
    snprintf(usart_buffer, UART_MAX_LENGTH, "I2C NOK: ");
    base_usart_send_string(usart_buffer);
  }
}
