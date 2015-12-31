/*
 * uarts_class.cpp
 *
 * Created: 30.07.2015 16:10:35
 *  Author: sd
 */ 

#include "UART.h"

#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>

#include <util/atomic.h>
#include <util/delay.h>


#define CYCLE_COUNT_WAIT 100000


#define RXC    7
#define TXC    6
#define UDRE   5
#define FE     4
#define DOR    3
#define UPE    2
#define U2X    1
#define MPCM   0

#define RXCIE  7
#define TXCIE  6
#define UDRIE  5
#define RXEN   4
#define TXEN   3
#define UCSZ2  2
#define RXB8   1
#define TXB8   0

#define UMSEL1 7
#define UMSEL0 6
#define UPM1   5
#define UPM0   4
#define USBS   3
#define UCSZ1  2
#define UCSZ0  1
#define UCPOL  0

#define DATA_REGISTER_EMPTY (1<<UDRE)
#define RX_COMPLETE (1<<RXC)
#define FRAMING_ERROR (1<<FE)
#define PARITY_ERROR (1<<UPE)
#define DATA_OVERRUN (1<<DOR)

#define UCSRA_OFFSET -6
#define UCSRB_OFFSET -5
#define UCSRC_OFFSET -4
#define UBRR_OFFSET -2

UART::UART(volatile unsigned char& udr, unsigned long int baud, int txBuffersize, int rxBuffersize) :
_udr(&udr), tx_buffer_size(txBuffersize), rx_buffer_size(rxBuffersize)
{
	_ucsra = _udr+UCSRA_OFFSET;
	_ucsrb = _udr+UCSRB_OFFSET;
	_ucsrc = _udr+UCSRC_OFFSET;
	_ubrr = _udr+UBRR_OFFSET;

	tx_buffer = (char*)calloc(tx_buffer_size, sizeof(char));
	rx_buffer = (char*)calloc(rx_buffer_size, sizeof(char));

	//rx_buffer_overflow = 0;
	tx_wr_index = 0;
	tx_rd_index = 0;
	tx_counter = 0;
	rx_wr_index = 0;
	rx_rd_index = 0;
	rx_counter = 0;

	begin(baud);
}

UART::~UART()
{
	end();
	free(tx_buffer);
	free(rx_buffer);
}

void UART::setBaud(unsigned long int baud)
{
	*_ubrr = (((F_CPU) + 8UL * (baud)) / (16UL * (baud)) -1UL);
}

void UART::tx_byte_int()
{
	if (tx_counter > 0)
	{
		tx_counter--;
		*_udr = tx_buffer[tx_rd_index++];
		if (tx_rd_index == tx_buffer_size) tx_rd_index=0;
	}
}

void UART::putch(char c)
{
	while (tx_counter == tx_buffer_size);
	cli();
	if (tx_counter || ((*_ucsra & DATA_REGISTER_EMPTY)==0))
	{
		tx_buffer[tx_wr_index++]=c;
		if (tx_wr_index == tx_buffer_size) tx_wr_index=0;
		tx_counter++;
	}
	else
		*_udr=c;
	sei();
}

int  UART::write(const unsigned char *outbuf, unsigned int outlen)
{
	for(unsigned int i = 0; i < outlen; i++)
		putch(outbuf[i]);
	return outlen;
}

void UART::rx_byte_int()
{
	char status = *_ucsra;
	char data = *_udr;

	if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
	{
		byteRecived(data);

		rx_buffer[rx_wr_index++]=data;
		if (rx_wr_index == rx_buffer_size) rx_wr_index=0;
		if (++rx_counter == rx_buffer_size)	rx_counter=rx_buffer_size;
	}
}

char UART::getch()
{
	char data;
	while (rx_counter==0);
	cli();
	data=rx_buffer[rx_rd_index++];
	if (rx_rd_index == rx_buffer_size) rx_rd_index=0;
	--rx_counter;
	sei();
	return data;
}

int  UART::read(unsigned char *inbuf, unsigned int inlen)
{
	long int delay_counter = 0;
	while((rx_counter < inlen)&&(delay_counter < CYCLE_COUNT_WAIT))
		delay_counter++;

	unsigned int buffCount = rx_counter;
	if(buffCount > inlen)
		buffCount = inlen;
	for(unsigned int i = 0; i < buffCount; i++)
		inbuf[i] = getch();
	return buffCount;
}

void UART::byteRecived(char rxByte)
{

}

void UART::flush(void) // erase rx circular buffer
{
	rx_rd_index = rx_wr_index;
	rx_counter = 0;
}

void UART::doBreak(void)
{
	while(tx_counter > 0);
	unsigned short int tempUBRR = *_ubrr;
	setBaud(4800);
	putch(0x00);
	while(tx_counter > 0);
	*_ubrr = tempUBRR;
}

int UART::available(){
	return rx_counter;
}

char UART::peek()
{
	return rx_buffer[rx_rd_index];
}

void UART::end()
{
	*_ucsra = 0x00;
	*_ucsrb = 0x00;
	*_ucsrc = 0x00;
	flush();
}

void UART::begin()
{
	// USART initialization
	// Communication Parameters: 8 Data, 1 Stop, No Parity
	// USART Receiver: On
	// USART Transmitter: On
	// USART Mode: Asynchronous

	*_ucsra=(0<<RXC) | (0<<TXC) | (0<<UDRE) | (0<<FE) | (0<<DOR) | (0<<UPE) | (0<<U2X) | (0<<MPCM);
	*_ucsrb=(1<<RXCIE) | (1<<TXCIE) | (0<<UDRIE) | (1<<RXEN) | (1<<TXEN) | (0<<UCSZ2) | (0<<RXB8) | (0<<TXB8);
	*_ucsrc=(0<<UMSEL1) | (0<<UMSEL0) | (0<<UPM1) | (0<<UPM0) | (0<<USBS) | (1<<UCSZ1) | (1<<UCSZ0) | (0<<UCPOL);
}

void UART::begin(long baud_rate)
{
	begin();
	setBaud(baud_rate);
}

