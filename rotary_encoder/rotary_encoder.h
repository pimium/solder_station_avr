//
// Created by pimi on 19.11.20.
//

#ifndef SOLDER_STATION_ROTARY_ENCODER_H
#define SOLDER_STATION_ROTARY_ENCODER_H

#include <avr/io.h>

void initEncoder(void);
uint16_t get_rotate_counter(void);

#endif // SOLDER_STATION_ROTARY_ENCODER_H
