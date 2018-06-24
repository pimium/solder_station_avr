//
// Created by pimi on 15.03.18.
//
// Convenience definitions for PORTB
#include <avr/io.h>
#include <stdlib.h>

#include "hv5812.h"

#define DIGITAL_WRITE_HIGH(PORT) PORTB |= (1 << PORT)
#define DIGITAL_WRITE_LOW(PORT) PORTB &= ~(1 << PORT)

void hv5812_init(void)
{
  DDRB |= (1 << HV5812_CLK); // Set port as output
  DDRB |= (1 << HV5812_DAT); // Set port as output
  DDRB |= (1 << HV5812_STR); // Set port as output
  DDRB |= (1 << HV5812_BLA); // Set port as output
  DIGITAL_WRITE_LOW(HV5812_STR);
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