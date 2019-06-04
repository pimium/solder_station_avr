//
// Created by pimi on 29.03.19.
//

#include "vfd.h"
#include <avr/interrupt.h>
#include <stdint.h>

#define MESSAGE_LENGTH 8

// States
void vfd_states_start_write_byte();
void vfd_states_set_srclr();
void vfd_states_set_clk();
void vfd_states_clear_clk();
void vfd_states_set_word();

volatile uint8_t timer_counter = 0;

// State pointer
void (*statefunc)() = vfd_states_set_srclr;

ISR(TIMER0_OVF_vect)
{
    if (timer_counter > 0)
        timer_counter--;
    (*statefunc)();
}

static uint8_t value_position[7] = {0xa, 0xa, 0xa, 0xa, 0xa, 0xa, 0xa};
static uint8_t register_value[4] = {0x0, 0x0, 0x0, 0x3};
static uint8_t data_byte = 0;
static uint8_t byte_count = 0;
static uint8_t actual_register = 3;
static uint8_t actual_segment = 0;

const uint8_t decimal_list[][2] = {
    {0x80, 0x6D}, // 0
    {0x00, 0x28}, // 1
    {0x20, 0x65}, // 2
    {0x20, 0x6C}, // 3
    {0xA0, 0x28}, // 4
    {0xA0, 0x4C}, // 5
    {0xA0, 0x4D}, // 6
    {0x00, 0x68}, // 7
    {0xA0, 0x6D}, // 8
    {0xA0, 0x6C}, // 9
    {0, 0}        // Blank
};

const uint8_t position_list[][2] = {
    {0x02, 0x04}, // G7
    {0x02, 0x20}, // G6
    {0x01, 0x01}, // G5
    {0x01, 0x10}, // G4
    {0x01, 0x40}, // G3
    {0x00, 0x02}, // G2
    {0x00, 0x10}  // G1
};

const uint8_t symbole_list[][2] = {
    {0x21, 0x80}, // 0 Glocke
    {0x22, 0x40}, // 1 Ofen
    {0x10, 0x02}, // 2 Auto
    {0x23, 0x08}, // 3 Stop
    {0x24, 0x02}, // 4 Kolben
    {0x25, 0x01}, // 5 Kasten
    {0x26, 0x08}, // 6 Start
    {0x06, 0x80}, // 7 Celcius
    //    {0x10, 0x08}, // 3 A am
    //    {0x10, 0x04}, // 4 P pm
    //    {0x22, 0x10}, // 6 :
    //    {0x22, 0x08}, // 2 D
};

static uint8_t value_Symbol[7][2] = {{0x0, 0x0}, {0x0, 0x0}, {0x0, 0x0},
                                     {0x0, 0x0}, {0x0, 0x0}, {0x0, 0x0},
                                     {0x0, 0x0}};

void vfd_write_special_character(uint8_t symb)
{
    uint8_t filament_byte;
    uint8_t symb_value;
    uint8_t byte_position;
    uint8_t filament_nr;

    symb_value = symbole_list[symb][1];
    filament_byte = symbole_list[symb][0];
    byte_position = filament_byte >> 4;
    filament_nr = filament_byte & 0xf;
    value_Symbol[filament_nr][0] = byte_position;
    value_Symbol[filament_nr][1] = symb_value;
}

void vfd_states_set_word(void)
{
    uint8_t position_value = position_list[actual_segment][1];
    uint8_t position = position_list[actual_segment][0];
    uint8_t value = value_position[actual_segment];

    uint8_t symbol = value_Symbol[actual_segment][1];
    uint8_t symbol_reg = value_Symbol[actual_segment][0];

    PORTC &= ~(1 << SRCLK);

    register_value[0] = decimal_list[value][1];
    register_value[1] = decimal_list[value][0];
    register_value[2] = 0x00;
    register_value[3] = 0x02;
    register_value[position] = register_value[position] | position_value;
    register_value[symbol_reg] = register_value[symbol_reg] | symbol;

    if (actual_segment > 5)
    {
        actual_segment = 0;
    }
    else
    {
        actual_segment++;
    }
    statefunc = vfd_states_set_srclr;
}

void set_register_value(uint8_t value1, uint8_t value2, uint8_t value3)
{
    register_value[0] = value1;
    register_value[1] = value2;
    register_value[2] = value3;
}

void get_register_value(uint8_t *reg)
{
    reg[0] = register_value[0];
    reg[1] = register_value[1];
    reg[2] = register_value[2];
}

void vfd_init(void)
{
    DDRC |= (1 << RCLK) | (1 << DATA) | (1 << SRCLK) | (1 << SRCLR);
    PORTC &= ~(1 << SRCLR);
    PORTC &= ~(1 << SRCLK);
    PORTC &= ~(1 << RCLK);

    PORTC |= (1 << SRCLR);
    statefunc = vfd_states_set_word;
}

void vfd_set_rclK()
{
    PORTC &= ~(1 << SRCLK);
    if ((actual_register & 0x3) == 0)
    {
        statefunc = vfd_states_set_word;
    }
    else if ((actual_register & 0x3) == 3)
    {
        PORTC |= (1 << RCLK);
        statefunc = vfd_states_start_write_byte;
    }
    else
    {
        statefunc = vfd_states_start_write_byte;
    }
}

void vfd_states_set_srclr()
{
    PORTC |= (1 << SRCLR);
    statefunc = vfd_states_start_write_byte;
}

void vfd_states_start_write_byte()
{
    data_byte = register_value[actual_register & 0x3];
    actual_register++;
    byte_count = MESSAGE_LENGTH;

    statefunc = vfd_states_clear_clk;
}

void vfd_states_set_clk()
{
    data_byte = data_byte >> 1;
    byte_count--;

    PORTC |= (1 << SRCLK);

    if (byte_count)
        statefunc = vfd_states_clear_clk;
    else
    {
        PORTC &= ~(1 << RCLK);
        statefunc = vfd_set_rclK;
    }
}

void vfd_states_clear_clk()
{
    if (data_byte & 0x01)
        PORTC |= (1 << DATA);
    else
        PORTC &= ~(1 << DATA);

    PORTC &= ~(1 << SRCLK);
    statefunc = vfd_states_set_clk;
}

void vfd_write_word(uint8_t pos, uint8_t value) { value_position[pos] = value; }

void vfd_blank(void)
{
    for (int i = 0; i < 7; ++i)
    {
        value_position[i] = 0xa;
        value_Symbol[i][0] = 0;
        value_Symbol[i][1] = 0;
    }
}

uint16_t vfd_convert_bcd(uint16_t binaryInput)
{
    uint16_t bcdResult = 0;
    uint8_t shift = 0;

    while (binaryInput > 0)
    {
        bcdResult |= (binaryInput % 10) << (shift++ << 2);
        binaryInput /= 10;
    }
    return bcdResult;
}
