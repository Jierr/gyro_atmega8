#include "config.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include <util/delay.h>
#include <stdio.h>


#include "base_i2c.h"
#include "base_usart.h"
#include "base_timer.h"
#include "base_sw_pwm.h"
#include "abstract_7segment.h"
#include <avr/interrupt.h>

#define UART_MAX_LENGTH 80
#define I2C_RECEIVE_BUFFER_LENGTH 16
#define I2C_TRANSMIT_BUFFER_LENGTH 16

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
	OCR2 = (uint8_t)((F_CPU/(prescaler*fips))-1);
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
		while((ASSR&(1<<TCN2UB)) || (ASSR&(1<<OCR2UB)) || (ASSR&(1<<TCR2UB)))
		;
	}

	/* clear interrupt flags */
	TIFR &= ~((1<<OCF2) | (1<<TOV2));
	/* enable compare match interrupt */
	TIMSK |= (1<<OCIE2);
	SREG = sreg;
}



ISR(USART_RXC_vect)
{
	
	uint8_t sreg = SREG;
	cli();
	
#if 0
	DDRB |= (1<<DDB0);
	if(PORTB & (1<<PB0))
	PORTB &= ~(1<<PB0);
	else
	PORTB |= (1<<PB0);
#endif
	

	usart_data_in = UDR;
	switch (usart_data_in)
	{
		case 'h':
		timer2_hours=(timer2_hours+1)%24;
		break;
		case 'j':
		timer2_hours=(24+timer2_hours-1)%24;
		break;
		case 'm':
		timer2_minutes=(timer2_minutes+1)%60;
		break;
		case ',':
		timer2_minutes=(60+timer2_minutes-1)%60;
		break;
		case 's':
		timer2_seconds=(timer2_seconds+1)%60;
		break;
		case 'd':
		timer2_seconds=(60+timer2_seconds-1)%60;
		break;
		case 'u':
		timer2_milliseconds=(timer2_milliseconds+100)%1000;
		break;
		case 'i':
		timer2_milliseconds=(1000+timer2_milliseconds-100)%1000;
		break;
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
	uint32_t a,b;

	base_timer0_context.tick+=1;
	base_sw_pwm_timer0_callback();
	
#if 0
	a = base_timer0_context.tick;
	b = base_timer0_s_ticks(1);
	mod = a%b;
	if(mod == 0)
	{
		DDRB |= (1<<DDB0);
		if(PORTB & (1<<PB0))
		PORTB &= ~(1<<PB0);
		else
		PORTB |= (1<<PB0);
	}
#endif

	SREG = sreg;
}

ISR(TIMER2_COMP_vect)
{
#if CONFIG_TIME == 1
	static uint8_t msTimer = 0;
	msTimer+=1;
	if (msTimer == 100)
	{
		timer2_milliseconds=(timer2_milliseconds+100)%1000;
		if (!timer2_milliseconds)
		{
			timer2_seconds=(timer2_seconds+1)%60;
			if (!timer2_seconds)
			{
				timer2_minutes=(timer2_minutes+1)%60;
				if (!timer2_minutes)
				timer2_hours=(timer2_hours+1)%24;
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
	uint8_t motor[4] = {0,};
	uint16_t sms = 0;
	uint8_t ssec = 0;
	uint8_t smin = 0;
	uint8_t shour= 0;
	uint8_t ms100 = 0;
	uint16_t pwm_cycle_frequency_hz = 0;

	cli();
	DDRB = 0;
	DDRC = 0;
	DDRD = 0;
	PORTB = 0;
	PORTC = 0;
	PORTD = 0;
	//led
	DDRB |= (1<<DDB0);
	//base_timer1_init();
	/* OCR = 249, 1 Interrupt per milisecond --> 1000 ips ISR = TIMER2_COMP_vect */
	timer2_ctc_interrupt_init(1000UL, 64, 0);
	base_usart_init(MUBRR);
	//7segment
	//abstract_7segment_init();
	base_sw_pwm_ctx_init();
	base_timer0_init();
	//bottom right pins
	pwm_cycle_frequency_hz = base_sw_pwm_set_global_cycle(0);
	motor[0] = base_sw_pwm_init(BASE_PORTB, BASE_PIN2);
	motor[1] = base_sw_pwm_init(BASE_PORTB, BASE_PIN3);
	//bottom left pins
	motor[2] = base_sw_pwm_init(BASE_PORTB, BASE_PIN1);
	motor[3] = base_sw_pwm_init(BASE_PORTD, BASE_PIN8);
	sei();
	
#if CONFIG_DEBUG_GEN
	base_usart_send_string("TIMER0 Ticks pro Sekunde: '");
	base_usart_send_decimal(base_timer0_context.s_tick_count);
	base_usart_send_string("'\r\n");
	base_usart_send_string("TIMER0 Ticks pro Millisekunde: '");
	base_usart_send_decimal(base_timer0_context.ms_tick_count);
	base_usart_send_string("'\r\n");
	base_usart_send_string("PWM Frequenz: '");
	base_usart_send_decimal(pwm_cycle_frequency_hz);
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
	if(base_i2c_is_ready())
	base_i2c_start_read(&i2c_ctx, TWI_SLA_MPU6050, 0x75, i2c_receive_buffer, 1);
	base_i2c_wait();
	
#if CONFIG_DEBUG_GEN
	snprintf(usart_buffer, UART_MAX_LENGTH, "I2C Identity %X\r\n", i2c_receive_buffer[0]);
	base_usart_send_string(usart_buffer);
#endif
	base_i2c_start_read(&i2c_ctx, TWI_SLA_MPU6050, 0x6B, i2c_receive_buffer, 1);
	base_i2c_wait();
	i2c_transmit_buffer[0] = 1;
	//trans_buf[0] = (recv_buf[0] & ~(1<<6)) + 1;
	base_i2c_start_write(&i2c_ctx, TWI_SLA_MPU6050, 0x6B, i2c_transmit_buffer, 1);
	base_i2c_wait();
	base_i2c_start_read(&i2c_ctx, TWI_SLA_MPU6050, 0x6B, i2c_receive_buffer, 1);
	base_i2c_wait();

	/*while loop only does tasks every 100ms*/
	base_usart_send_string("Initialization complete.\r\n");
	_delay_ms(500);
	while (1)
	{
#if CONFIG_DEBUG_MP6050 == 1
		if (signal_100ms_event)
		{
			/*when preparing components for string no interrupts allowed*/
			++ms100;
			cli();
			signal_100ms_event = 0;
			sms=timer2_milliseconds;
			ssec = timer2_seconds;
			smin = timer2_minutes;
			shour = timer2_hours;
			sei();
		}

		if(ms100 == 1)
		{
			ms100 = 0;
			// Clear everything from cursor downwards
			base_usart_send_string("\033[0J");
			snprintf(usart_buffer, UART_MAX_LENGTH, "%02d:%02d:%02d:%03d\r\n", shour, smin, ssec, sms);
			base_usart_send_string(usart_buffer);
			mp6050_print_info();
			// Place cursor to the beginning
			base_usart_send_string("\r\033[4A");
		}
#endif
	}
	return 0;
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
		rgyro[0] = (i2c_receive_buffer[0] << 8) | i2c_receive_buffer[1]; rgyro[0] = rgyro[0] / 131 + 1;
		rgyro[1] = (i2c_receive_buffer[2] << 8) | i2c_receive_buffer[3]; rgyro[1] = rgyro[1] / 131 - 2;
		rgyro[2] = (i2c_receive_buffer[4] << 8) | i2c_receive_buffer[5]; rgyro[2] = rgyro[2] / 131 + 1;

		snprintf(usart_buffer, UART_MAX_LENGTH, "Gyro: X(%5hi) Y(%5hi) Z(%5hi) - ",
		(int)rgyro[0], (int)rgyro[1], (int)rgyro[2]);
		base_usart_send_string(usart_buffer);
		base_usart_send_byte_hex_string(i2c_receive_buffer, 6);
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
		raccel[0] = (i2c_receive_buffer[0] << 8) | i2c_receive_buffer[1]; tmp32= ((int32_t)raccel[0] * 100) / 16384; raccel[0] = (short)tmp32;
		raccel[1] = (i2c_receive_buffer[2] << 8) | i2c_receive_buffer[3]; tmp32= ((int32_t)raccel[1] * 100) / 16384; raccel[1] = (short)tmp32 + 2;
		raccel[2] = (i2c_receive_buffer[4] << 8) | i2c_receive_buffer[5]; tmp32= ((int32_t)raccel[2] * 100) / 16384; raccel[2] = (short)tmp32 + 15;

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
