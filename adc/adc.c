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
    ADMUX = (ADMUX & 0xF0) | 6;

    // ADC Enable and prescaler of 128
    // 16000000/128 = 125000
    ADCSRA = (1 << ADEN) | (1 << ADATE) | (1 << ADIE) | (0 << ADPS2) |
             (1 << ADPS1) | (0 << ADPS0);

    //    // Timer/Counter0 Compare Match A
    //    ADCSRB |= (0 << ADTS2) | (1 << ADTS1) | (1 << ADTS0);

    // Timer/counter1 overflow
    ADCSRB |= (1 << ADTS2) | (1 << ADTS1) | (0 << ADTS0);
}

ISR(ADC_vect)
{
    static uint8_t act_channel = 0;
    adc_sum = ADC;
    //    // set chanel
    //    act_channel++;
    //    ADMUX = (ADMUX & 0xF0) | (6 + (act_channel & 0x1));
}

uint16_t adc_read() { return adc_sum; }