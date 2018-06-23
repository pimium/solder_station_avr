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

#include "i2c_bitbang.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>

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

#include "cpufreq.h"

// ----------------------------------------------------------------------------

#include <util/delay.h>

volatile uint16_t counter_interrupt = 0;
volatile uint8_t armed = 0;

// ----------------------------------------------------------------------------

ISR(PCINT_vect)
{
  cli();
  TCCR0B |= (0 << CS02) | (0 << CS01) | (1 << CS00);
  counter_interrupt = 0;
  armed = 1;
  sei();
}

ISR(TIMER0_OVF_vect)
{
  cli();
  counter_interrupt++;
  PORTB &= ~(1 << PB3);
  sei();
}

static inline void initTimer0(void)
{
  // Timer 0 konfigurieren
  TCCR0B |= (0 << CS02) | (0 << CS01) | (1 << CS00); // Prescaler
  // TCCR0B = 0; // Disable timer interrup

  // Overflow Interrupt erlauben
  TIMSK |= (1 << TOIE0);
}

static inline void init_PCINT_Interrupt(void)
{
  GIMSK = (1 << PCIE);    // Enable INT0
  PCMSK |= (1 << PCINT4); // pin change interrupt enabled for PCINT4
  DDRB &= ~(1 << PB4);    // set as input
  PORTB &= ~(1 << PB4);   // disable pull-up
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

  _delay_ms(100);

  DDRB &= ~(1 << PB4);
  DDRB |= (1 << PB3) | (1 << PB1);
  PORTB &= ~(1 << PB1);

  init_PCINT_Interrupt();
  initTimer0();
  sei();

  // ---- Main Loop ----
  while (1)
  {
    if ((counter_interrupt > 0x1D) && (armed == 1))
    {
      TCCR0B |= (0 << CS12) | (0 << CS11) | (0 << CS10);
      counter_interrupt = 0;
      armed = 0;
      PORTB = (1 << PB3);
    }
  }

  return 0;
}

// ============================================================================
