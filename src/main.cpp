/*
 * main.cpp
 *
 *  Created on: 29 груд. 2015
 *      Author: sd
 */

#include <avr/io.h>
#include <util/atomic.h>
#include "ADC/Analog.h"


inline uint16_t getSpeed()
{
	return analog[0];
}

inline uint16_t getDuty()
{
	return analog[1];
}

inline bool isInverse()
{
	return ! (PIND >> 7) && 0x01;
}

inline bool isReverse()
{
	return ! (PINB >> 0) && 0x01;
}

inline void setOutA(uint8_t pwm)
{
	OCR2 = pwm;
}

inline void setOutB(uint8_t pwm)
{
	if (isReverse())
		OCR1AL = pwm;
	else
		OCR1BL = pwm;
}

inline void setOutC(uint8_t pwm)
{
	if (isReverse())
		OCR1BL = pwm;
	else
		OCR1AL = pwm;
}

void setInverse(bool cond)
{
	if (cond) // output: Non-Inverted PWM (inverted output)
	{
		TCCR1A &= ~((1 << COM1A0)
					|(1 << COM1B0));
		TCCR2  &= ~ (1 << COM20);
	}
	else  // output: Inverted PWM (normal output)
	{
		TCCR1A |= ((1 << COM1A0)
				   |(1 << COM1B0));
		TCCR2  |=  (1 << COM20);
	}
}

void init()
{
	// Input/Output Ports initialization
	DDRB = (1 << DDB3) | (1 << DDB2) | (1 << DDB1);
	PORTB = _BV(PORT0);
	DDRC = 0;
	PORTC = 0;
	DDRD = 0;
	PORTD = _BV(PORT7);

	// Timer/Counter initialization
	TCCR1A =  (1 << COM1A1) | (1 << COM1A0) // OC0A output: Inverted PWM(normal output)
			| (1 << COM1B1) | (1 << COM1B0) // OC0B output: Inverted PWM(normal output)

			|(1 << WGM10);
	TCCR1B = (1 << WGM12)  // Mode: Fast PWM top=0xFF

			| (0 << CS02) | (1 << CS01) | (0 << CS00); // Set clock
	// Clock value: 2000,000 kHz
	// Timer Period: 0,128 ms

	TCCR2 = (1 << COM21) | (1 << COM20) // OC2 output: Inverted PWM(normal output)

			| (1 << WGM21) | (1 << WGM20) // Mode: Fast PWM top=0xFF

			| (0 << CS22) | (1 << CS21) | (0 << CS20); //set clock
	// Clock value: 2000,000 kHz
	// Timer Period: 0,128 ms


	// Timer/Counter Interrupt(s) initialization
	TIMSK = 0;

	// Global enable interrupts
	sei();

}



/*
 * Logic
 */

#include <math.h>
#include <util/delay.h>

#define USE_SERIAL 0
#define LIMITER 1
#define SPEED_RATE 1/10000


#if USE_SERIAL
#include <stdio.h>
#include <avr/interrupt.h>
#include "UART/UART.h"

UART * _port;
ISR(USART_RX_vect)
{
	_port->rx_byte_int();
}
ISR(USART_TX_vect)
{
	_port->tx_byte_int();
}
#endif

void calcPhase(double value, double level) // phaseValue: grade of anlge; dutyLevel: -1...0...1; 0 it is neutral full amplitude
{
	if (level < -1) level = -1;  else if (level > 1) level = 1;

	double cutterA = sin(value + M_PI*0/3);
	double cutterB = sin(value + M_PI*2/3);
	double cutterC = sin(value + M_PI*4/3);

	cutterA += level;
	cutterB += level;
	cutterC += level;

#if LIMITER
	if (cutterA < -1) cutterA = -1; else if (cutterA > 1) cutterA = 1;
	if (cutterB < -1) cutterB = -1; else if (cutterB > 1) cutterB = 1;
	if (cutterC < -1) cutterC = -1; else if (cutterC > 1) cutterC = 1;
#else
	cutterA = cutterA % 1;
	cutterB = cutterB % 1;
	cutterC = cutterC % 1;
#endif

	setOutA((cutterA + 1) * 0x7F);
	setOutB((cutterB + 1) * 0x7F);
	setOutC((cutterC + 1) * 0x7F);
}


int main()
{
	init();

#if USE_SERIAL
	UART port(UDR, 115200, 128, 8);
	_port = &port;
	char buff[150];
#endif

	double counter = 0;

	while (true)
	{
		double duty = ((double)getDuty()/512)-1;

		calcPhase(counter, duty);

		counter += ((double)getSpeed()*SPEED_RATE);
		if (counter >= M_PI*2)
			counter = 0;

		setInverse(isInverse());

#if USE_SERIAL
		sprintf(buff,
				"ICR: %u,\t cont: %u,\t ang0: %u, ang1: %u, A: %u, B: %u, C: %u\r\n",
				ICR1, counter, analog[0] / 4, analog[1] / 4, OCR1AL, OCR1BL,
				OCR2);
		port(buff);
#endif

		_delay_ms(1);

	}
	return 0;
}

