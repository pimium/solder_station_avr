/**
 * SSD1306xLED - Drivers for SSD1306 controlled dot matrix OLED/PLED 128x64
 * displays
 *
 * @created: 2014-08-08
 * @author: Neven Boyanov
 *
 * This is part of the Tinusaur/SSD1306xLED project.
 *
 * Copyright (c) 2016 Neven Boyanov, Tinusaur Team. All Rights Reserved.
 * Distributed as open source software under MIT License, see LICENSE.txt file.
 * Please, as a favor, retain the link http://tinusaur.org to The Tinusaur
 * Project.
 *
 * Source code available at: https://bitbucket.org/tinusaur/ssd1306xled
 *
 */

// ============================================================================

// #define F_CPU 1000000UL
// NOTE: The F_CPU (CPU frequency) should not be defined in the source code.
//       It should be defined in either (1) Makefile; or (2) in the IDE.

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>

#include "cpufreq.h"
#include "hv5812.h"
//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//                ATtiny
//               25/45/85
//              +----------+
//      (RST)---+ PB5  Vcc +---(+)-------
// --[OWOWOD]---+ PB3  PB2 +---[TWI/SCL]-   -sck        -sck
//           ---+ PB4  PB1 +---             -strobe     -miso
// -------(-)---+ GND  PB0 +---[TWI/SDA]-   -data_in    -mosi
//              +----------+
//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// -----(+)--------------->	// Vcc,	Pin 1 on SSD1306 Board
// -----(-)--------------->	// GND,	Pin 2 on SSD1306 Board

// ----------------------------------------------------------------------------

#ifndef ZERO_CROSSING
#define ZERO_CROSSING PB4
#endif
#ifndef TRIAC_GATE
#define TRIAC_GATE PD6
#endif

volatile uint16_t counter_interrupt = 0;
volatile uint8_t armed = 0;

// ----------------------------------------------------------------------------

ISR(PCINT0_vect)
{
  cli();
  counter_interrupt = 0;
  armed = 1;
  sei();
}

ISR(TIMER0_OVF_vect)
{
  cli();

  if ((counter_interrupt > 0x1C) && (armed == 1))
  {
    counter_interrupt = 0;
    armed = 0;
    PORTD |= (1 << TRIAC_GATE);
  }
  else
  {
    counter_interrupt++;
    PORTD &= ~(1 << TRIAC_GATE);
  }

  sei();
}

static inline void initTimer0(void)
{
  // Timer 0 konfigurieren
  TCCR0B |= (0 << CS02) | (0 << CS01) | (1 << CS00); // Prescaler

  // Overflow Interrupt erlauben
  TIMSK0 |= (1 << TOIE0);
}

static inline void init_PCIE_Interrupt(void)
{
  PCICR = (1 << PCIE0);            // Enable Pin Change Interrupt
  PCMSK0 |= (1 << PCINT4);         // pin change interrupt enabled for PCINT4
  DDRB &= ~(1 << ZERO_CROSSING);  // set as input
  PORTB &= ~(1 << ZERO_CROSSING); // disable pull-up
}

int main(void)
{

  // ---- Initialization ----

  //    // ---- CPU Frequency Setup ----
  //#if F_CPU == 1000000UL
  //    #pragma message "F_CPU=1MHZ"
  //    CLKPR_SET(CLKPR_1MHZ);
  //#elif F_CPU == 8000000UL
  //
  //#pragma message "F_CPU=8MHZ"
  //    CLKPR_SET(CLKPR_8MHZ);
  //#else
  //    #pragma message "F_CPU=????"
  //#error "CPU frequency should be either 1 MHz or 8 MHz"
  //#endif

  //  _delay_ms(100);

  DDRD |= (1 << TRIAC_GATE);
  //  DDRB |= (1 << PB2);
  //  PORTB &= ~(1 << PB2);

  init_PCIE_Interrupt();
  initTimer0();
  hv5812_init();
  sei();

  //  PORTB |= (1 << PB1);
  // ---- Main Loop ----

  while (1)
  {
    for (uint8_t i = 0; i < 0x40; ++i)
    {
      hv5812_send_byte(0xFF);
      _delay_ms(700);
    }
    //      hv5812_blank(0);
    //                _delay_ms(700);
    //      hv5812_blank(1);
    //                _delay_ms(500);
  }

  return 0;
}

// ============================================================================
