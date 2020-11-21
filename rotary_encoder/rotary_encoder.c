//
// Created by pimi on 24.05.19.
//

#include "rotary_encoder.h"
#include <avr/interrupt.h>
#ifndef ROTARY_ENCODER_DIR
#define ROTARY_ENCODER_DIR PD5 // DIR
#endif
#ifndef ROTARY_ENCODER_SCK
#define ROTARY_ENCODER_SCK PD4 // SCK
#endif

#define PHASE_DIR (PIND & (1 << ROTARY_ENCODER_DIR))
#define PHASE_SCK (PIND & (1 << ROTARY_ENCODER_SCK))

volatile uint16_t rotate_counter = 0;

ISR(PCINT2_vect)
{
    static uint8_t last = 0;
    if (!(last & (1 << PD4)))
    {
        if (PHASE_SCK)
        {
            if (PHASE_DIR)
            {
                rotate_counter++;
            }
            else
            {
                rotate_counter--;
            }
        }
    }
    last = PIND;
    rotate_counter = rotate_counter & 0xFFFF;
}

void initEncoder(void)
{
    DDRD &= ~((1 << PD4) | (1 << PD5));
    PORTD |= ((1 << PD4) | (1 << PD5));
    PCMSK2 |= (1 << PD4);
    PCICR |= (1 << PCIE2);
}

uint16_t get_rotate_counter(void) { return rotate_counter; }
