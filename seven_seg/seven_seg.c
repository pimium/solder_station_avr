//
// Created by pimi on 29.03.19.
//

#include "seven_seg.h"
#include <avr/interrupt.h>
#include <stdint.h>

#define SEVEN_SEG_DIO PC1
#define SEVEN_SEG_MCLR PC2

enum byte_states_enum
{
    INIT_BYTE_STATE,
    START_BIT_BYTE_STATE,
    SET_BIT_BYTE_STATE,
    NEW_BIT_BIT_BYTE_STATE,
    WAIT_BYTE_STATE,
    RELEASE_BIT_BYTE_STATE,
    BYTE_SENT_BYTE_STATE
};

enum word_states_enum
{
    IDLE_WORD_STATE,
    SEND_WORD_POSITION,
    SEND_WORD_VALUE,
    WAIT_WORD_STATE
};

volatile uint8_t value_in = 0x30;
volatile uint8_t count_bit = 0;
uint8_t intern_register[4] = {0, 0xff, 0, 0};
uint8_t new_data = 0;

volatile enum byte_states_enum new_byte_state = INIT_BYTE_STATE;

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

void seven_handle_word(void)
{
    static enum word_states_enum word_states = IDLE_WORD_STATE;
    static uint8_t position = 0;
    static uint8_t time_out = 0;
    uint8_t value = 0;
    uint8_t write_result = 0;
    switch (word_states)
    {
    case IDLE_WORD_STATE:
        if (new_data == 1)
        {
            word_states = SEND_WORD_POSITION;
            position = 0;
        }
        break;

    case SEND_WORD_POSITION:
        write_result = write_byte(position);
        if (write_result == INIT_BYTE_STATE)
        {
            time_out = 0x1F;
            value = intern_register[position];
            word_states = SEND_WORD_VALUE;
        }
        break;

    case SEND_WORD_VALUE:
        write_result = write_byte(value);
        if (write_result == INIT_BYTE_STATE)
        {
            position++;
            word_states = WAIT_WORD_STATE;
        }
        break;

    case WAIT_WORD_STATE:
        if (time_out)
            time_out--;
        else
        {
            if (position > 3)
            {
                new_data = 0;
                word_states = IDLE_WORD_STATE;
            }
            else
                word_states = SEND_WORD_POSITION;
        }
        break;

    default:
        word_states = IDLE_WORD_STATE;
        break;
    }
}

// ISR(TIMER0_OVF_vect) // Timer1 ISR
void seven_seg_handle_byte(void)
{
    static enum byte_states_enum states = INIT_BYTE_STATE;
    switch (states)
    {
    case INIT_BYTE_STATE:
        if (new_byte_state == START_BIT_BYTE_STATE)
        {
            states = START_BIT_BYTE_STATE;
        }
        count_bit = 8;
        break;
    case START_BIT_BYTE_STATE:
        PORTC &= ~(1 << SEVEN_SEG_DIO);
        states = SET_BIT_BYTE_STATE;
        break;
    case SET_BIT_BYTE_STATE:
        if (value_in & 0x80)
        {
            PORTC |= (1 << SEVEN_SEG_DIO);
        }
        else
        {
            PORTC &= ~(1 << SEVEN_SEG_DIO);
        }
        count_bit--;
        states = NEW_BIT_BIT_BYTE_STATE;
        break;
    case NEW_BIT_BIT_BYTE_STATE:
        value_in = value_in << 1;
        states = WAIT_BYTE_STATE;
        break;
    case WAIT_BYTE_STATE:
        states = RELEASE_BIT_BYTE_STATE;
        break;

    case RELEASE_BIT_BYTE_STATE:
        PORTC |= (1 << SEVEN_SEG_DIO);
        if (count_bit == 0)
            states = BYTE_SENT_BYTE_STATE;
        else
            states = START_BIT_BYTE_STATE;
        break;
    case BYTE_SENT_BYTE_STATE:
        PORTC |= (1 << SEVEN_SEG_DIO);
        new_byte_state = INIT_BYTE_STATE;
        states = INIT_BYTE_STATE;
        break;
    default:
        PORTC |= (1 << SEVEN_SEG_DIO);
        states = INIT_BYTE_STATE;
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
    if (new_byte_state != INIT_BYTE_STATE)
        return new_byte_state;

    value_in = value;
    new_byte_state = START_BIT_BYTE_STATE;

    return 0;
}

uint8_t write_byte_to_register(uint8_t pos, uint8_t value)
{
    if (pos < 4)
    {
        intern_register[pos] = value;
        new_data = 1;
    }

    return 0;
}

uint8_t seven_seg_write_display(uint8_t value)
{
    write_byte_to_register(3, hexa[value & 0xf]);
    write_byte_to_register(2, hexa[value >> 4]);

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