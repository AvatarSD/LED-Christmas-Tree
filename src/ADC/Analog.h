/*
 * ADC.h
 *
 *  Created on: 28 вер. 2015 р.
 *      Author: sd
 */

#ifndef ADC_ANALOG_H_
#define ADC_ANALOG_H_


/**********************/
#define FIRST_ADC_INPUT 0
#define LAST_ADC_INPUT 7
// Voltage Reference: AVCC pin
#define ADC_VREF_TYPE ((0<<REFS1) | (1<<REFS0) | (0<<ADLAR))

/**********************/


class Analog
{
public:
	Analog();
	void begin();
	void ADCint();
	unsigned int dataAt(char pin);
	unsigned int operator[](char pin);

private:
	volatile unsigned int adc_data[LAST_ADC_INPUT-FIRST_ADC_INPUT+1];
};

extern Analog analog;

#endif /* ADC_ANALOG_H_ */
