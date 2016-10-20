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



volatile uint8_t dataIn = 0;
volatile uint16_t ms = 0;
volatile uint8_t sec = 0;
volatile uint8_t min = 0;
volatile uint8_t hour = 0;

/* signals, ISR */
volatile uint8_t transEn = 0;



/*fips = interrupts per second*/






inline void initTimerCTC(uint32_t fips, uint16_t prescaler, uint8_t async)
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
	
	DDRB |= (1<<DDB0);
	if(PORTB & (1<<PB0))
		PORTB &= ~(1<<PB0);
	else
		PORTB |= (1<<PB0);
		
	dataIn = UDR;
	switch (dataIn)
	{
		case 'h':
			hour=(hour+1)%24;
			break;
		case 'j':
			hour=(24+hour-1)%24;
			break;
		case 'm':
			min=(min+1)%60;
			break;
		case ',':
			min=(60+min-1)%60;
			break;
		case 's':
			sec=(sec+1)%60;
			break;
		case 'd':
			sec=(60+sec-1)%60;
			break;
		case 'u':
			ms=(ms+100)%1000;
			break;
		case 'i':
			ms=(1000+ms-100)%1000;
			break;
		case 'y':
		//base_timer1_set_pwm_change_duty_by(-1);
		base_sw_pwm_duty(0, -1);
		base_sw_pwm_duty(1, -1);
		base_sw_pwm_duty(2, -1);
		base_sw_pwm_duty(3, -1);
		base_usart_send_string(":ISR(USART_RXC_vect):> -1 pwm_duty\r\n");
		break;
		case 'x':
		//base_timer1_set_pwm_change_duty_by(1);
		base_sw_pwm_duty(0, 1);
		base_sw_pwm_duty(1, 1);
		base_sw_pwm_duty(2, 1);
		base_sw_pwm_duty(3, 1);
		base_usart_send_string(":ISR(USART_RXC_vect):> +1 pwm_duty\r\n");
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
	static uint8_t msTimer = 0;

	base_timer0_context.tick+=1;	
	//base_sw_pwm_timer0_callback();
	
#if 0
	a = base_timer0_context.tick;
	b = base_timer0_s_ticks(1)/100;
	mod = a%b; 
	if(((uint8_t)mod&0xFF) == 0)
	{
		
		ms+=10;
		//msTimer+=10;
		/*
		if (msTimer == 100)
		{
			ms=(ms+100)%1000;

				
			if (!ms)
			{
				sec=(sec+1)%60;
				if (!sec)
				{
					min=(min+1)%60;
					if (!min)
					hour=(hour+1)%24;
				}
			}

			transEn = 1;
			msTimer = 0;
		}
		*/
		
		//base_usart_send_string("tick:");
		//base_usart_send_decimal(ms);
		//base_usart_send_string(" == ");
		//base_usart_send_decimal(b);
		//base_usart_send_string("\r\n");
		
	}
#endif

	SREG = sreg;	
}

ISR(TIMER2_COMP_vect)
{
	
	static uint8_t msTimer = 0;
	msTimer+=1;
	if (msTimer == 100)
	{	
		ms=(ms+100)%1000;

		
        if (!ms)
        {
                sec=(sec+1)%60;
                if (!sec)
                {
                        min=(min+1)%60;
                        if (!min)
                                hour=(hour+1)%24;
                }
        }

		transEn = 1;
		msTimer = 0;
	}	
	
	//abstract_7segment_display(sec/10, sec%10);
}


int main()
{
	char str[15] = {0,};
	char msg[128] = {0,};
	uint8_t motor[4] = {0,};
	uint16_t sms = 0;
	uint8_t ssec = 0;
	uint8_t smin = 0;
	uint8_t shour= 0;
	uint8_t c = 0;
	uint8_t trans_buf[64] = {0,};
	uint8_t recv_buf[64] = {0,};
	short rtemp = 0;
	short rgyro[3] = {0,};
	short raccel[3] = {0,};
	char* conv = 0;
	int32_t tmp32;
	//float ftemp = 0;
	//float fgyro[3] = {0,};
	//float faccel[3] = {0,};
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
		/* OCR = 249, 1 Interrupt per milisecond --> 1000 ips */
		initTimerCTC(1000UL, 64, 0);
		base_usart_init(MUBRR);
		//7segment
		//abstract_7segment_init();
		//base_sw_pwm_ctx_init();
		//base_timer0_init();
		//bottom right pins
		pwm_cycle_frequency_hz = base_sw_pwm_set_global_cycle(0);
		motor[0] = base_sw_pwm_init(PORTB, BASE_PIN2);
		motor[1] = base_sw_pwm_init(PORTB, BASE_PIN3);
		//bottom left pins
		motor[2] = base_sw_pwm_init(PORTB, BASE_PIN1);
		motor[3] = base_sw_pwm_init(PORTD, BASE_PIN8);
		
		snprintf(str, 15, "%02d:%02d:%02d:%03d\r\n", shour, smin, ssec, sms);
		str[14] = 0;
	sei();
	
	base_usart_send_string("PWM Frequenz:");
	base_usart_send_decimal(pwm_cycle_frequency_hz);
	base_usart_send_string("\r\n");
	base_usart_send_string("PWM Cycle Ticks:");
	base_usart_send_decimal(base_sw_pwm_ctx.cycle_tick_count);
	base_usart_send_string("\r\n");
	base_usart_send_string("PWM Motoren Duty Ticks:");
	base_usart_send_string("\r\n0:");
	base_usart_send_decimal(base_sw_pwm_ctx.pin[motor[0]].pwm_duty_ticks);
	base_usart_send_string("\r\n1:");
	base_usart_send_decimal(base_sw_pwm_ctx.pin[motor[1]].pwm_duty_ticks);
	base_usart_send_string("\r\n2:");
	base_usart_send_decimal(base_sw_pwm_ctx.pin[motor[2]].pwm_duty_ticks);
	base_usart_send_string("\r\n3:");
	base_usart_send_decimal(base_sw_pwm_ctx.pin[motor[3]].pwm_duty_ticks);
	base_usart_send_string("\r\n");
	
	snprintf(msg, 64, "base_i2c_init\r\n");
	c = 0;
	while (msg[c])
	{
		base_usart_send_byte((uint8_t)msg[c]);
		++c;
	}
	base_i2c_init();
	snprintf(msg, 64, "base_i2c_set_slave\r\n");
	c = 0;
	while (msg[c])
	{
		base_usart_send_byte((uint8_t)msg[c]);
		++c;
	}
	base_i2c_set_slave(&i2c_ctx, TWI_SLA_MPU6050);
	snprintf(msg, 64, "base_i2c_start_read\r\n");
	c = 0;
	while (msg[c])
	{
		base_usart_send_byte((uint8_t)msg[c]);
		++c;
	}
	if(i2c_ctx.twi_state == STATE_TWI_IDLE)
		base_i2c_start_read(&i2c_ctx, TWI_SLA_MPU6050, 0x75, recv_buf, 1);
		
	while(i2c_ctx.twi_state != STATE_TWI_IDLE)
	;
	
	snprintf(msg, 64, "Who am I? %X\r\n", recv_buf[0]);
	c = 0;
	while (msg[c])
	{
		base_usart_send_byte((uint8_t)msg[c]);
		++c;
	}
	
	base_i2c_start_read(&i2c_ctx, TWI_SLA_MPU6050, 0x6B, recv_buf, 1);
	
	while(i2c_ctx.twi_state != STATE_TWI_IDLE)
	;
	
	snprintf(msg, 64, "PWR_MGMT_1 = %X\r\nWake UP!\r\n", recv_buf[0]);
	c = 0;
	while (msg[c])
	{
		base_usart_send_byte((uint8_t)msg[c]);
		++c;
	}
	trans_buf[0] = 1;
	//trans_buf[0] = (recv_buf[0] & ~(1<<6)) + 1;
	base_i2c_start_write(&i2c_ctx, TWI_SLA_MPU6050, 0x6B, trans_buf, 1);

	while(i2c_ctx.twi_state != STATE_TWI_IDLE)
	;
	base_i2c_start_read(&i2c_ctx, TWI_SLA_MPU6050, 0x6B, recv_buf, 1);
	
	while(i2c_ctx.twi_state != STATE_TWI_IDLE)
	;		
			snprintf(msg, 64, "PWR_MGMT_1 = %X\r\n", recv_buf[0]);
			c = 0;
			while (msg[c])
			{
				base_usart_send_byte((uint8_t)msg[c]);
				++c;
			}
		

	/*time(while) < 100ms
 	  does only smth every 100ms*/
	//a = 0;
	_delay_ms(500);
	while (1)
	{
		
#if CONFIG_DEBUG_MP6050 == 1
		if (transEn)
		{
			/*when preparing components for string no interrupts allowed*/
			++ms100;
			cli();
			transEn = 0;
			sms=ms;
			ssec = sec;
			smin = min;
			shour = hour;
			sei();
			snprintf(str, 15, "%02d:%02d:%02d:%03d", shour, smin, ssec, sms);
			str[14] = 0;
			c = 0;
			
			while (str[c])
			{

				base_usart_send_byte((uint8_t)str[c]);
				++c;
			}
		}


		
		//_delay_ms(500);
		if(ms100 == 5)
		{

			ms100 = 0;
			//Gyroscope: Error: (x,<,z) = (-1,2-1)
			while(i2c_ctx.twi_state != STATE_TWI_IDLE)
			;
			base_i2c_start_read(&i2c_ctx, TWI_SLA_MPU6050, 0x43, recv_buf, 6);
			while(i2c_ctx.twi_state != STATE_TWI_IDLE)
			;
			rgyro[0] = (recv_buf[0] << 8) | recv_buf[1]; rgyro[0] = rgyro[0] / 131 + 1;
			rgyro[1] = (recv_buf[2] << 8) | recv_buf[3]; rgyro[1] = rgyro[1] / 131 - 2;
			rgyro[2] = (recv_buf[4] << 8) | recv_buf[5]; rgyro[2] = rgyro[2] / 131 + 1;
			
			snprintf(msg, 128, "\r\nGyro:  (%3hhu,%3hhu),(%3hhu,%3hhu),(%3hhu,%3hhu)=(%5hi),(%5hi),(%5hi)    \r\n",
			recv_buf[0], recv_buf[1], recv_buf[2], recv_buf[3], recv_buf[4], recv_buf[5], rgyro[0],rgyro[1],rgyro[2]);
			c = 0;
			while (msg[c])
			{
				base_usart_send_byte((uint8_t)msg[c]);
				++c;
			}
			
			//Accelerator, Error z-Axis -0.15 to -0.2g, x-Axis -0.03 to -0.07g
			base_i2c_start_read(&i2c_ctx, TWI_SLA_MPU6050, 0x3B, recv_buf, 6);
			while(i2c_ctx.twi_state != STATE_TWI_IDLE)
			;
			raccel[0] = (recv_buf[0] << 8) | recv_buf[1]; tmp32= ((int32_t)raccel[0] * 100) / 16384; raccel[0] = (short)tmp32;
			raccel[1] = (recv_buf[2] << 8) | recv_buf[3]; tmp32= ((int32_t)raccel[1] * 100) / 16384; raccel[1] = (short)tmp32 + 2;
			raccel[2] = (recv_buf[4] << 8) | recv_buf[5]; tmp32= ((int32_t)raccel[2] * 100) / 16384; raccel[2] = (short)tmp32 + 15;

			snprintf(msg, 128, "Accel: (%3hhu,%3hhu),(%3hhu,%3hhu),(%3hhu,%3hhu)=(%5hi)/100,(%5hi)/100,(%5hi)/100   \r\n",
			recv_buf[0], recv_buf[1], recv_buf[2], recv_buf[3], recv_buf[4], recv_buf[5], raccel[0],raccel[1],raccel[2]);
			c = 0;
			while (msg[c])
			{
				base_usart_send_byte((uint8_t)msg[c]);
				++c;
			}
			
			//Temperature Error +4 degree Celsius
			base_i2c_start_read(&i2c_ctx, TWI_SLA_MPU6050, 0x41, recv_buf, 2);
			while(i2c_ctx.twi_state != STATE_TWI_IDLE)
			;
			conv = (char*)&rtemp;
			conv[0] = recv_buf[1];
			conv[1] = recv_buf[0];
			//rtemp = ((short)recv_buf[0] << 8) | (short)recv_buf[1];
			rtemp = rtemp / 340 + 37 - 4;
			snprintf(msg, 128, "Temp:  (%3hhu,%3hhu) = %5hi   \r\033[3A",
			recv_buf[0], recv_buf[1], rtemp);
			c = 0;
			while (msg[c])
			{
				base_usart_send_byte((uint8_t)msg[c]);
				++c;
			}
			
		}
		else
		{					
			snprintf(msg, 128, "\r");
			c = 0;
			while (msg[c])
			{
				base_usart_send_byte((uint8_t)msg[c]);
				++c;
			}			
		}
		/*
		if(i2c_ctx.twi_state == STATE_TWI_IDLE && (a < 0x7E))		
		{
			snprintf(msg, 64, "slave:%X =====================\r\n", a);
			c = 0;
			while (msg[c])
			{
				base_usart_send_byte((uint8_t)msg[c]);
				++c;
			}
				
			base_i2c_start_read(&i2c_ctx, a, 0x75);
			a+=2;
		}
		*/
#endif
	}
	return 0;
}
