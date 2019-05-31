//
// Created by pimi on 29.03.19.
//

#include "vfd.h"
//#include <avr/pgmspace.h>
#include <stdint.h>

#define MESSAGE_LENGTH 8

// States
void led_on();
void led_off();

// State pointer
void (*statefunc)() = led_on;
//(*statefunc)();

void led_on() {
    PORTB |= 1;
    statefunc = led_off;
}

void led_off() {
    PORTB &= ~1;
    statefunc = led_on;
}


static uint8_t register_value[3] = {0, 0, 0};

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
    {0x26, 0x08} // 6 Start
//    {0x06, 0x03}, // 7 Celcius
//    {0x10, 0x08}, // 3 A am
//    {0x10, 0x04}, // 4 P pm
//    {0x22, 0x10}, // 6 :
//    {0x22, 0x08}, // 2 D
};

void vfd_write_special_character(uint8_t symb)
{
    PORTC &= ~(1 << SRCLR);
    uint8_t filament_byte = symbole_list[symb][0];
    uint8_t symb_value = symbole_list[symb][1];
    register_value[0] = 0;
    register_value[1] = 0;
    register_value[2] = 0;

    PORTC |= (1 << SRCLR);
    uint8_t byte_position = (filament_byte >> 4);
    uint8_t filament_nr = filament_byte & 0xf;

    uint8_t posit = position_list[filament_nr][0];

    register_value[byte_position] = symb_value;
    register_value[posit] |= position_list[filament_nr][1];

    vfd_write_byte(0x03);
    PORTC &= ~(1 << RCLK);
    vfd_write_byte(register_value[0]);
    vfd_write_byte(register_value[1]);
    vfd_write_byte(register_value[2]);
    PORTC |= (1 << RCLK);
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

}

void vfd_write_byte(uint8_t byte)
{
    int i = MESSAGE_LENGTH;
    do
    {
        if (byte & 0x01)
            PORTC |= (1 << DATA);
        else
            PORTC &= ~(1 << DATA);

        PORTC &= ~(1 << SRCLK);
        byte = byte >> 1;
        i--;

        PORTC |= (1 << SRCLK);
    } while (i > 0);
}

void vfd_write_word(uint8_t pos, uint8_t value)
{
    PORTC &= ~(1 << SRCLR);
    uint8_t position_value = position_list[pos][1];
    uint8_t position = position_list[pos][0];

    register_value[0] = decimal_list[value][1];
    register_value[1] = decimal_list[value][0];
    register_value[2] = 0;
    register_value[position] = register_value[position] | position_value;

    PORTC |= (1 << SRCLR);

    vfd_write_byte(0x03);
    PORTC &= ~(1 << RCLK);
    vfd_write_byte(register_value[0]);
    vfd_write_byte(register_value[1]);
    vfd_write_byte(register_value[2]);
    PORTC |= (1 << RCLK);
}

void vfd_blank(void)
{
    PORTC |= (1 << SRCLR);
    vfd_write_byte(0x00);
    PORTC &= ~(1 << RCLK);
    vfd_write_byte(0x00);
    vfd_write_byte(0x00);
    vfd_write_byte(0x00);
    PORTC |= (1 << RCLK);
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
