//
// Created by pimi on 29.03.19.
//

#include "seven_seg.h"
#include <avr/interrupt.h>
#include <stdint.h>

#define SEVEN_SEG_DIO PC1
#define SEVEN_SEG_MCLR PC2

enum states_enum
{
    IDLE_STATE,
    INIT_STATE,
    START_BIT_STATE,
    SET_BIT_STATE,
    NEW_BIT_BIT_STATE,
    WAIT_STATE,
    RELEASE_BIT_STATE,
    BYTE_SENT_STATE
};

volatile enum states_enum states = IDLE_STATE;

volatile uint8_t value_in = 0x30;
volatile uint8_t count_bit = 0;

#define A (1 << 0)
#define B (1 << 1)
#define C (1 << 2)
#define D (1 << 3)
#define E (1 << 4)
#define F (1 << 5)
#define G (1 << 6)
#define DP (1 << 7)

uint8_t hexa[16] = {
    A | B | C | D | E | F,     // 0
    B | C,                     // 1
    A | B | D | E | G,         // 2
    A | B | C | D | G,         // 3
    B | C | F | G,             // 4
    A | C | D | F | G,         // 5
    A | C | D | E | F | G,     // 6
    A | B | C,                 // 7
    A | B | C | D | E | F | G, // 8
    A | B | C | D | F | G,     // 9
    A | B | C | E | F | G,     // A
    C | D | E | F | G,         // b
    A | D | E | F,             // C
    B | C | D | E | G,         // d
    A | D | E | F | G,         // e
    A | E | F | G,             // f
};

// ISR(TIMER0_OVF_vect) // Timer1 ISR
void seven_seg_handle(void)
{

    switch (states)
    {
    case INIT_STATE:
        states = START_BIT_STATE;
        count_bit = 8;
        break;
    case START_BIT_STATE:
        PORTC &= ~(1 << SEVEN_SEG_DIO);
        states = SET_BIT_STATE;
        break;
    case SET_BIT_STATE:
        if (value_in & 0x80)
        {
            PORTC |= (1 << SEVEN_SEG_DIO);
        }
        else
        {
            PORTC &= ~(1 << SEVEN_SEG_DIO);
        }
        count_bit--;
        states = NEW_BIT_BIT_STATE;
        break;
    case NEW_BIT_BIT_STATE:
        value_in = value_in << 1;
        states = WAIT_STATE;
        break;
    case WAIT_STATE:
        states = RELEASE_BIT_STATE;
        break;

    case RELEASE_BIT_STATE:
        PORTC |= (1 << SEVEN_SEG_DIO);
        if (count_bit == 0)
            states = BYTE_SENT_STATE;
        else
            states = START_BIT_STATE;
        break;
    case BYTE_SENT_STATE:
        PORTC |= (1 << SEVEN_SEG_DIO);
        states = IDLE_STATE;
        break;
    default:
        PORTC |= (1 << SEVEN_SEG_DIO);
        states = IDLE_STATE;
    }
    //    PORTC ^= (1 << PC3);
}

void seven_seg_init(void)
{
    DDRC |= (1 << SEVEN_SEG_DIO);
    DDRC |= (1 << SEVEN_SEG_MCLR);
    PORTC &= ~(1 << SEVEN_SEG_MCLR);
    PORTC |= (1 << SEVEN_SEG_DIO) | (1 << SEVEN_SEG_MCLR);
}

void seven_seg_reset(void)
{
    PORTC &= ~(1 << SEVEN_SEG_MCLR);
    PORTC &= ~(1 << SEVEN_SEG_MCLR);
    PORTC &= ~(1 << SEVEN_SEG_MCLR);
    PORTC |= (1 << SEVEN_SEG_MCLR);
}

uint8_t write_byte(uint8_t value)
{
    while (states)
        ;
    //    if(states)
    //        return states;
    states = INIT_STATE;
    value_in = value;
    return 0;
}

// uint8_t write_byte(uint8_t value) {
//    for (uint8_t j = 0; j < 0x08; ++j) {
//        PORTC &= ~(1 << SEVEN_SEG_DIO);
//        for (volatile uint8_t j = 0; j < 0xFF; ++j)
//            ;
//        if (value & 0x80) {
//            PORTC |= (1 << SEVEN_SEG_DIO);
//        } else {
//            PORTC &= ~(1 << SEVEN_SEG_DIO);
//        }
//        for (volatile uint8_t j = 0; j < 0xFF; ++j)
//            ;
//        PORTC |= (1 << SEVEN_SEG_DIO);
//        for (volatile uint8_t j = 0; j < 0x80; ++j)
//            ;
//        value = value << 1;
//    }
//    PORTC |= (1 << SEVEN_SEG_DIO);
//
//    for (volatile uint8_t j = 0; j < 0x80; ++j)
//        ;
//    return 0;
//}

uint8_t write_loop_byte(uint8_t pos, uint8_t value)
{

    //    uint8_t crc = 0x00;

    //    crc = crc8(pos & 0x03, crc);
    //    crc = crc8(value, crc);

    write_byte(pos & 0x03);
    write_byte(value);
    //    write_byte(crc);

    return 0;
}

uint8_t seven_seg_write_display(uint8_t value)
{

    write_loop_byte(3, hexa[value & 0xf]);
    write_loop_byte(2, hexa[value >> 4]);

    return 0;
}

uint8_t crc8(uint8_t data, int8_t crc)
{
    char sum;

    for (uint8_t i = 8; i; i--)
    {
        sum = (crc ^ data) & 0x80;
        crc <<= 1;
        if (sum)
            crc ^= 0x1D;
        data <<= 1;
    }

    return crc;
}