//
// Created by pimi on 29.03.19.
//

#include <avr/io.h>

#ifndef SEVEN_SEG_H
#define SEVEN_SEG_H

void seven_seg_init(void);
uint8_t write_byte(uint8_t value);
uint8_t write_loop_byte(uint8_t pos, uint8_t value);
uint8_t seven_seg_write_display(uint8_t value);
// uint8_t get_timer_count(void);
// void set_timer_count(uint8_t value);
void seven_seg_reset(void);
void seven_seg_handle(void);
uint8_t crc8(uint8_t data, int8_t crc);
#endif // SEVEN_SEG_H
