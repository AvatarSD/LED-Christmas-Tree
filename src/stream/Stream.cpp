/*
 * Stream.cpp
 *
 *  Created on: 27 лист. 2015
 *      Author: sd
 */

#include "Stream.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

Stream::Stream()
{
	// TODO Auto-generated constructor stub

}

Stream::~Stream()
{
	// TODO Auto-generated destructor stub
}


void * operator new(size_t size)
{
	return malloc(size);
}

void operator delete(void * ptr)
{
	free(ptr);
}

void Stream::print(const char * str)
{
	if (preSendHandler())
		this->write((const unsigned char*) str, strlen(str));
	postSendHandler();
}

void Stream::print(long int num)
{
	char buf[12];
	sprintf(buf, "%ld", num);
	print(buf);
}

void Stream::print(int num)
{
	char buf[12];
	sprintf(buf, "%d", num);
	print(buf);
}

void Stream::print(unsigned long int num)
{
	char buf[12];
	sprintf(buf, "%lu", num);
	print(buf);
}

void Stream::print(unsigned int num)
{
	char buf[12];
	sprintf(buf, "%u", num);
	print(buf);
}

void Stream::print(double num)
{
	char buf[12];
	sprintf(buf, "%f", num);
	print(buf);
}

void Stream::print(const __FlashStringHelper * flashStr)
{
	if (preSendHandler())
	{
		char readedByte, *flashPointer = (char*) flashStr;
		readedByte = pgm_read_byte(flashPointer);
		while (readedByte)
		{
			putch(readedByte);
			flashPointer++;
			readedByte = pgm_read_byte(flashPointer);
		}
	}
	postSendHandler();
}

bool Stream::preSendHandler()
{
	return true;
}
bool Stream::postSendHandler()
{
	return true;
}

void Stream::putch(char)
{

}

int Stream::write(const unsigned char *outbuf, unsigned int outlen)
{
	return 0;
}

