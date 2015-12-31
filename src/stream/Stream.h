/*
 * Stream.h
 *
 *  Created on: 27 лист. 2015
 *      Author: sd
 */

#ifndef STREAM_STREAM_H_
#define STREAM_STREAM_H_

#include "avr/pgmspace.h"

class __FlashStringHelper;
#define F(string_literal) (reinterpret_cast<const __FlashStringHelper *>(PSTR(string_literal)))

class Stream
{
public:
	Stream();
	virtual ~Stream();

	/*output*/
	virtual void putch(char);
	virtual int  write(const unsigned char *outbuf, unsigned int outlen);

	/*input*/
	void print(const char * str);
	void print(const __FlashStringHelper * flashStr);
	void print(long int num);
	void print(int num);
	void print(unsigned long int num);
	void print(unsigned int num);
	void print(double num);
	template<class T>void (* operator()(T str)){print(str);return 0;}

	/*Additional routine*/
	virtual bool preSendHandler();
	virtual bool postSendHandler();
};

#endif /* STREAM_STREAM_H_ */
