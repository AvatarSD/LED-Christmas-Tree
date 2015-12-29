/*
 * ADC.cpp
 *
 *  Created on: 28 вер. 2015 р.
 *      Author: sd
 */

#include "Analog.h"
#include "avr/io.h"
#include "avr/interrupt.h"
#include "util/delay.h"

Analog analog;

ISR(ADC_vect)
{
	analog.ADCint();
}

Analog::Analog()
{
	begin();
}

void Analog::begin()
{
	// ADC initialization
	// ADC Clock frequency: 1000,000 kHz
	// ADC Voltage Reference: AVCC pin
	// ADC Auto Trigger Source: Free Running
	// Digital input buffers on ADC0: On, ADC1: On, ADC2: On, ADC3: On
	// ADC4: On, ADC5: On
	DIDR0 = (0 << ADC5D) | (0 << ADC4D) | (0 << ADC3D) | (0 << ADC2D)
			| (0 << ADC1D) | (0 << ADC0D);
	ADMUX = FIRST_ADC_INPUT | ADC_VREF_TYPE;
	ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (0 << ADIF)
			| (1 << ADIE) | (1 << ADPS2) | (0 << ADPS1) | (0 << ADPS0);
	ADCSRB = (0 << ADTS2) | (0 << ADTS1) | (0 << ADTS0);

	// Analog Comparator: Off
	ACSR = (1 << ACD);

}

// ADC interrupt service routine
// with auto input scanning
void Analog::ADCint()
{
	static unsigned char input_index = 0;
	// Read the AD conversion result
	adc_data[input_index] = ADCW;
	// Select next ADC input
	if (++input_index > (LAST_ADC_INPUT - FIRST_ADC_INPUT))
		input_index = 0;
	ADMUX = (FIRST_ADC_INPUT | ADC_VREF_TYPE) + input_index;
	// Delay needed for the stabilization of the ADC input voltage
	//_delay_us(10);
	// Start the AD conversion
	ADCSRA |= (1 << ADSC);
}

unsigned int Analog::dataAt(char pin)
{
	if ((pin >= FIRST_ADC_INPUT) && (pin <= LAST_ADC_INPUT))
		return adc_data[pin - FIRST_ADC_INPUT];
	else
		return 0;
}

unsigned int Analog::operator[](char pin)
{
	return dataAt(pin);
}

