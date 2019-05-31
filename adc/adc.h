//
// Created by pimi on 24.05.19.
//

#ifndef SOLDER_STATION_ADC_H
#define SOLDER_STATION_ADC_H
#include <avr/io.h>

void adc_init();
uint16_t adc_read(uint8_t ch);

#endif //SOLDER_STATION_ADC_H
