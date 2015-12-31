/*
 * main.cpp
 *
 *  Created on: 29 груд. 2015
 *      Author: sd
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdio.h>

#include "ADC/Analog.h"
#include "UART/UART.h"

#define  MAX_VAL 5000

UART * _port;

ISR(USART_RX_vect)
{
	_port->rx_byte_int();
}
ISR(USART_TX_vect)
{
	_port->tx_byte_int();
}

uint8_t counter = 0;

// Timer1 input capture interrupt service routine
ISR(TIMER1_CAPT_vect)
{
// Place your code here
	cli();
	counter++;
	if(counter>=MAX_VAL)counter = 0;
	sei();
}

inline void setOutA(uint16_t pwm)
{
	OCR0A = (uint8_t) (pwm / 0xff);
}

inline void setOutB(uint16_t pwm)
{
	OCR0B = (uint8_t) (pwm / 0xff);
}

inline void setOutC(uint16_t pwm)
{
	OCR2B = (uint8_t) (pwm / 0xff);
}

void calcPhase(uint16_t value, uint16_t level)
{
//	double lev = ((double) level * 2 / (sizeof(level) * 8)) - 1;
//
//	setOutA((sin((value * M_PI) / (sizeof(value) * 8)) + lev)
//			/ (2 * (sizeof(value) * 8)));
//	setOutA((sin(((value * M_PI) / (sizeof(value) * 8)) + M_PI / 3) + lev)
//			/ (2 * (sizeof(value) * 8)));
//	setOutA((sin(((value * M_PI) / (sizeof(value) * 8)) + (2 * M_PI) / 3)
//			+ lev) / (2 * (sizeof(value) * 8)));

	setOutA((sin(((double) value * M_PI) / 0xffff) + 1) * 0xffff);
	setOutB(counter);
	setOutC(value);
}

void init()
{
	// Input/Output Ports initialization
	// Port B initialization
	// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=Out Bit2=In Bit1=In Bit0=In
	DDRB = (0 << DDB7) | (0 << DDB6) | (0 << DDB5) | (0 << DDB4) | (1 << DDB3)
			| (0 << DDB2) | (0 << DDB1) | (0 << DDB0);
	// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=0 Bit2=T Bit1=T Bit0=T
	PORTB = (0 << PORTB7) | (0 << PORTB6) | (0 << PORTB5) | (0 << PORTB4)
			| (0 << PORTB3) | (0 << PORTB2) | (0 << PORTB1) | (0 << PORTB0);

	// Port C initialization
	// Function: Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
	DDRC = (0 << DDC6) | (0 << DDC5) | (0 << DDC4) | (0 << DDC3) | (0 << DDC2)
			| (0 << DDC1) | (0 << DDC0);
	// State: Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
	PORTC = (0 << PORTC6) | (0 << PORTC5) | (0 << PORTC4) | (0 << PORTC3)
			| (0 << PORTC2) | (0 << PORTC1) | (0 << PORTC0);

	// Port D initialization
	// Function: Bit7=In Bit6=Out Bit5=Out Bit4=In Bit3=Out Bit2=In Bit1=In Bit0=In
	DDRD = (0 << DDD7) | (1 << DDD6) | (1 << DDD5) | (0 << DDD4) | (1 << DDD3)
			| (0 << DDD2) | (0 << DDD1) | (0 << DDD0);
	// State: Bit7=T Bit6=0 Bit5=0 Bit4=T Bit3=0 Bit2=T Bit1=T Bit0=T
	PORTD = (0 << PORTD7) | (0 << PORTD6) | (0 << PORTD5) | (0 << PORTD4)
			| (0 << PORTD3) | (0 << PORTD2) | (0 << PORTD1) | (0 << PORTD0);

	// Timer/Counter 0 initialization
	// Clock source: System Clock
	// Clock value: 2000,000 kHz
	// Mode: Fast PWM top=0xFF
	// OC0A output: Non-Inverted PWM
	// OC0B output: Non-Inverted PWM
	// Timer Period: 0,128 ms
	// Output Pulse(s):
	// OC0A Period: 0,128 ms Width: 0,063749 ms
	// OC0B Period: 0,128 ms Width: 0,063749 ms
	TCCR0A = (1 << COM0A1) | (0 << COM0A0) | (1 << COM0B1) | (0 << COM0B0)
			| (1 << WGM01) | (1 << WGM00);
	TCCR0B = (0 << WGM02) | (0 << CS02) | (1 << CS01) | (0 << CS00);
	TCNT0 = 0x00;
	OCR0A = 0x00;
	OCR0B = 0x00;

	// Timer/Counter 1 initialization
	// Clock source: System Clock
	// Clock value: 2000,000 kHz
	// Mode: CTC top=ICR1
	// OC1A output: Disconnected
	// OC1B output: Disconnected
	// Noise Canceler: Off
	// Input Capture on Falling Edge
	// Timer Period: 32,768 ms
	// Timer1 Overflow Interrupt: Off
	// Input Capture Interrupt: On
	// Compare A Match Interrupt: Off
	// Compare B Match Interrupt: Off
	TCCR1A = (0 << COM1A1) | (0 << COM1A0) | (0 << COM1B1) | (0 << COM1B0)
			| (0 << WGM11) | (0 << WGM10);
	TCCR1B = (0 << ICNC1) | (0 << ICES1) | (1 << WGM13) | (1 << WGM12)
			| (0 << CS12) | (1 << CS11) | (0 << CS10);
	TCNT1H = 0x00;
	TCNT1L = 0x00;
	ICR1H = 0xFF;
	ICR1L = 0xFF;
	OCR1AH = 0x00;
	OCR1AL = 0x00;
	OCR1BH = 0x00;
	OCR1BL = 0x00;

	// Timer/Counter 2 initialization
	// Clock source: System Clock
	// Clock value: 2000,000 kHz
	// Mode: Fast PWM top=0xFF
	// OC2A output: Non-Inverted PWM
	// OC2B output: Non-Inverted PWM
	// Timer Period: 0,128 ms
	// Output Pulse(s):
	// OC2A Period: 0,128 ms Width: 0,063749 ms
	// OC2B Period: 0,128 ms Width: 0,063749 ms
	ASSR = (0 << EXCLK) | (0 << AS2);
	TCCR2A = (1 << COM2A1) | (0 << COM2A0) | (1 << COM2B1) | (0 << COM2B0)
			| (1 << WGM21) | (1 << WGM20);
	TCCR2B = (0 << WGM22) | (0 << CS22) | (1 << CS21) | (0 << CS20);
	TCNT2 = 0x00;
	OCR2A = 0x00;
	OCR2B = 0x00;

	// Timer/Counter 0 Interrupt(s) initialization
	TIMSK0 = (0 << OCIE0B) | (0 << OCIE0A) | (0 << TOIE0);

	// Timer/Counter 1 Interrupt(s) initialization
	TIMSK1 = (1 << ICIE1) | (0 << OCIE1B) | (0 << OCIE1A) | (0 << TOIE1);

	// Timer/Counter 2 Interrupt(s) initialization
	TIMSK2 = (0 << OCIE2B) | (0 << OCIE2A) | (0 << TOIE2);

	// External Interrupt(s) initialization
	// INT0: Off
	// INT1: Off
	// Interrupt on any change on pins PCINT0-7: Off
	// Interrupt on any change on pins PCINT8-14: Off
	// Interrupt on any change on pins PCINT16-23: Off
	EICRA = (0 << ISC11) | (0 << ISC10) | (0 << ISC01) | (0 << ISC00);
	EIMSK = (0 << INT1) | (0 << INT0);
	PCICR = (0 << PCIE2) | (0 << PCIE1) | (0 << PCIE0);

	// USART initialization
	// USART disabled
	UCSR0B = (0 << RXCIE0) | (0 << TXCIE0) | (0 << UDRIE0) | (0 << RXEN0)
			| (0 << TXEN0) | (0 << UCSZ02) | (0 << RXB80) | (0 << TXB80);

//	// Analog Comparator initialization
//	// Analog Comparator: Off
//	// The Analog Comparator's positive input is
//	// connected to the AIN0 pin
//	// The Analog Comparator's negative input is
//	// connected to the AIN1 pin
//	ACSR = (1 << ACD) | (0 << ACBG) | (0 << ACO) | (0 << ACI) | (0 << ACIE)
//			| (0 << ACIC) | (0 << ACIS1) | (0 << ACIS0);
//	ADCSRB = (0 << ACME);
//	// Digital input buffer on AIN0: On
//	// Digital input buffer on AIN1: On
//	DIDR1 = (0 << AIN0D) | (0 << AIN1D);
//
//	// ADC initialization
//	// ADC disabled
//	ADCSRA = (0 << ADEN) | (0 << ADSC) | (0 << ADATE) | (0 << ADIF)
//			| (0 << ADIE) | (0 << ADPS2) | (0 << ADPS1) | (0 << ADPS0);

	// SPI initialization
	// SPI disabled
	SPCR = (0 << SPIE) | (0 << SPE) | (0 << DORD) | (0 << MSTR) | (0 << CPOL)
			| (0 << CPHA) | (0 << SPR1) | (0 << SPR0);

	// TWI initialization
	// TWI disabled
	TWCR = (0 << TWEA) | (0 << TWSTA) | (0 << TWSTO) | (0 << TWEN)
			| (0 << TWIE);

	// Global enable interrupts
	sei();

}

int main()
{
	init();

	UART port(UDR0,115200, 128,8);
	_port = &port;

	char buff[150];

	while (true)
	{
		ICR1 = analog[0] * 64;
		calcPhase(analog[0] * 64, analog[1] * 64);
//		setOutA(0x4fff);//analog[0]*64);
//		setOutB(0x7fff);//analog[1]*64);
//		setOutC(analog[0]*64);
		sprintf(buff,
				"ICR: %ud, cont: %ud, ang0: %ud, ang1: %ud, A: %ud, B: %ud, C: %ud\r\n",
				ICR1, counter, analog[0], analog[1], OCR0A, OCR0B, OCR2B);
		port(buff);
	}
	return 0;
}
