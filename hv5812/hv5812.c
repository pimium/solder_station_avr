//
// Created by pimi on 15.03.18.
//

#include <avr/io.h>
#include <stdlib.h>

#include "hv5812.h"

uint8_t characterArray[] = {
//  ABCDEFG  Segments      7-segment map:
        0x7E, // 0   "0"          AAA
        0x30, // 1   "1"         F   B
        0x6D, // 2   "2"         F   B
        0x79, // 3   "3"          GGG
        0x33, // 4   "4"         E   C
        0x5B, // 5   "5"         E   C
        0x5F, // 6   "6"          DDD
        0x70, // 7   "7"
        0x7F, // 8   "8"
        0x7B, // 9   "9"
        0x77, // 10  "A"
        0x1F, // 11  "b"
        0x4E, // 12  "C"
        0x3D, // 13  "d"
        0x4F, // 14  "E"
        0x47  // 15  "F"
};

// Convenience definitions for PORTC
#define DIGITAL_WRITE_HIGH(PORT) PORTC |= (1 << PORT)
#define DIGITAL_WRITE_LOW(PORT) PORTC &= ~(1 << PORT)

void hv5812_write_char(uint8_t position, uint8_t value)
{
    uint8_t display_value = characterArray[value & 0xF];
    uint16_t byte23 = 0;
    uint8_t byte1 = (1 << 3) | (0 << 2) | (0 << 1) | (0 << 0);

    if (display_value & (0x40))
        byte23 |= (1 << FI_A);

    if (display_value & (0x20))
            byte23 |= (1 << FI_B);

    if (display_value & (0x10))
            byte23 |= (1 << FIC);

    if (display_value & (0x08))
            byte23 |= (1 << FI_D);

    if (display_value & (0x04))
            byte23 |= (1 << FI_E);

    if (display_value & (0x02))
            byte23 |= (1 << FI_F);
//
//    if (display_value & (0x01))
//            byte1 |= (1 << FI_G);

    byte23 = 0;
    byte23 |= (1 << 8) | (1 << FIC);

//    byte23 = 0xFFFFFF;
//
//    hv5812_send_byte(0x0F); //
//    hv5812_send_byte(0x02); //
//    hv5812_send_byte((1 << 7) | (1 << 6)| (1 << 5) | (1 << 4)
//                     | (1 << 3) | (0 << 2)| (1 << 1) | (1 << 0)); //B
    uint16_t current_byte = (byte23 >> 16);
    hv5812_send_byte(byte1 & 0xFF); //
    hv5812_send_byte((byte23 >> 8) & 0xFF); //
    hv5812_send_byte((byte23 >> 0) & 0xFF); //B
}

void hv5812_init(void)
{
    HV5812_DDR |= (1 << HV5812_CLK); // Set port as output
    HV5812_DDR |= (1 << HV5812_DAT); // Set port as output
    HV5812_DDR |= (1 << HV5812_STR); // Set port as output
    HV5812_DDR |= (1 << HV5812_BLA); // Set port as output
    DIGITAL_WRITE_LOW(HV5812_STR);
    DIGITAL_WRITE_LOW(HV5812_BLA);
}

void hv5812_send_byte(uint8_t byte)
{
  uint8_t i;
  for (i = 0; i < 8; i++)
  {
    if ((byte << i) & 0x80)
        DIGITAL_WRITE_HIGH(HV5812_DAT);
    else
        DIGITAL_WRITE_LOW(HV5812_DAT);

    DIGITAL_WRITE_HIGH(HV5812_CLK);
    DIGITAL_WRITE_HIGH(HV5812_STR);
    DIGITAL_WRITE_LOW(HV5812_CLK);
    DIGITAL_WRITE_LOW(HV5812_STR);
  }
}

void hv5812_blank(uint8_t clear)
{
  if (clear != 0)
    DIGITAL_WRITE_LOW(HV5812_BLA);
  else
    DIGITAL_WRITE_HIGH(HV5812_BLA);
}