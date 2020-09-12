//
// Created by pimi on 24.05.19.
//

#include "adc.h"
#include <avr/interrupt.h>

uint16_t adc_sum = 0;

void adc_init()
{
    // AREF = AVcc
    ADMUX = (0 << REFS0);
    //    // AREF = AVccInternal 1.1V Voltage Reference with external capacitor
    //    at AREF pin
    //    ADMUX = (1<<REFS1) | (1<<REFS0);

    // set chanel 6
    ADMUX = (ADMUX & 0xF8) | 6;

    // ADC Enable and prescaler of 128
    // 16000000/128 = 125000
    ADCSRA = (1 << ADEN) | (1 << ADATE) | (1 << ADIE) | (1 << ADPS2) |
             (1 << ADPS1) | (1 << ADPS0);
    ADCSRB |= (0 << ADTS2) | (1 << ADTS1) | (1 << ADTS0);
}

ISR(ADC_vect) { adc_sum = ADC; }

uint16_t adc_read() { return adc_sum; }