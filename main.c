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
#define ZERO_CROSSING PD2
#endif
#ifndef TRIAC_GATE_P
#define TRIAC_GATE_P PD6
#endif
#ifndef TRIAC_GATE_N
#define TRIAC_GATE_N PD7
#endif

volatile uint8_t counter_interrupt = 0;
volatile uint8_t counter_max = 80;
volatile uint8_t armed = 0;
static uint8_t percentage = 28;

// ----------------------------------------------------------------------------

ISR(INT0_vect)
{
  cli();
  //    counter_max = counter_interrupt;
  counter_interrupt = 0;
  armed = 2;
  PORTD &= ~(1 << TRIAC_GATE_P);
  PORTD ^= (1 << TRIAC_GATE_N);
  sei();
}

ISR(TIMER0_OVF_vect)
{
  cli();
  if ((armed == 2) && (counter_interrupt > percentage))
  {
    PORTD |= (1 << TRIAC_GATE_P);
    armed = 1;
  }
  else if (((armed == 1) &&
            counter_interrupt > (percentage + (counter_max >> 1))))
  {
    PORTD |= (1 << TRIAC_GATE_P);
    armed = 0;
  }
  else
  {
    PORTD &= ~(1 << TRIAC_GATE_P);
  }
  counter_interrupt++;
  sei();
}

ISR(TIMER0_COMPB_vect)
{
  // cli();
  // PORTD &= ~(1 << TRIAC_GATE_P);
  // PORTD &= ~(1 << TRIAC_GATE_N);
  // sei();
}

static inline void initTimer0(void)
{
  // Timer 0 konfigurieren
  TCCR0B |= (0 << CS02) | (0 << CS01) | (1 << CS00); // Prescaler

  // Overflow Interrupt erlauben // Output Compare A Match Interrupt Enable
  TIMSK0 |= ((1 << TOIE0) | (1 << OCIE0B));
  OCR0B = 0xF0;
}

static inline void init_IRQ0(void)
{
  EICRA |= (1 << ISC01);
  EIMSK |= (1 << INT0);
  DDRD &= ~(1 << ZERO_CROSSING);  // set as input
  PORTD &= ~(1 << ZERO_CROSSING); // disable pull-up
}

static inline void init_PCIE_Interrupt(void)
{
  PCICR = (1 << PCIE0);           // Enable Pin Change Interrupt
  PCMSK0 |= (1 << PCINT4);        // pin change interrupt enabled for PCINT4
  DDRB &= ~(1 << ZERO_CROSSING);  // set as input
  PORTB &= ~(1 << ZERO_CROSSING); // disable pull-up
}

int main(void)
{
  //  _delay_ms(100);

  DDRD |= ((1 << TRIAC_GATE_N) | (1 << TRIAC_GATE_P));
  //  DDRB |= (1 << PB2);
  //  PORTB &= ~(1 << PB2);

  init_IRQ0();
  initTimer0();
  //  hv5812_init();
  sei();

  //  PORTB |= (1 << PB1);
  // ---- Main Loop ----

  while (1)
  {
    // PORTD &= ~(1 << TRIAC_GATE_P);
  }

  return 0;
}

// ============================================================================
