/**
 * SSD1306xLED - Drivers for SSD1306 controlled dot matrix OLED/PLED 128x64 displays
 *
 * @created: 2014-08-08
 * @author: Neven Boyanov
 *
 * This is part of the Tinusaur/SSD1306xLED project.
 *
 * Copyright (c) 2016 Neven Boyanov, Tinusaur Team. All Rights Reserved.
 * Distributed as open source software under MIT License, see LICENSE.txt file.
 * Please, as a favor, retain the link http://tinusaur.org to The Tinusaur Project.
 *
 * Source code available at: https://bitbucket.org/tinusaur/ssd1306xled
 *
 */

// ============================================================================

// #define F_CPU 1000000UL
// NOTE: The F_CPU (CPU frequency) should not be defined in the source code.
//       It should be defined in either (1) Makefile; or (2) in the IDE. 

#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//                ATtiny
//               25/45/85
//              +----------+   (-)-------
//      (RST)---+ PB5  Vcc +---(+)-------
// --[OWOWOD]---+ PB3  PB2 +---[TWI/SCL]-
//           ---+ PB4  PB1 +---
// -------(-)---+ GND  PB0 +---[TWI/SDA]-
//              +----------+
//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// -----(+)--------------->	// Vcc,	Pin 1 on SSD1306 Board
// -----(-)--------------->	// GND,	Pin 2 on SSD1306 Board
#define SSD1306_SCL        PB2    // SCL,	Pin 3 on SSD1306 Board
#define SSD1306_SDA        PB0    // SDA,	Pin 4 on SSD1306 Board

#define SSD1306_SA        0x78    // Slave address

// ----------------------------------------------------------------------------

#include "cpufreq.h"

// ----------------------------------------------------------------------------

#define IDLE 0
#define ARMED 1
#define COUNTING 2
#define WAIT 3

#include <util/delay.h>

volatile uint16_t counter_interrupt = 0;

// ----------------------------------------------------------------------------

ISR(PCINT0_vect) {
//    PORTB ^= (1 << PB3);
    cli();
    PORTB &= ~(1 << PB3);
    TCCR1 |= (0 << CS13) | (0 << CS12) | (1 << CS11) | (0<< CS10);
    counter_interrupt = 0;
    sei();
}

ISR(TIMER1_OVF_vect) {
    cli();
    counter_interrupt++;
    PORTB &= ~(1 << PB3);
//    TCCR1 |= (0 << CS13) | (0 << CS12) | (0 << CS11) | (0 << CS10);
//    PORTB &= ~(1 << PB3);
    sei();
}


static inline void initTimer1(void) {
//    TCCR1 |= (0 << CTC1);  // clear timer on compare match
    TCCR1 |= (0 << CS13) | (0 << CS12) | (1 << CS11) | (0 << CS10);
    OCR1C = 0xff; // compare match value
    TIMSK |= (1 << TOIE1); // enable overflow interrupt
}

static inline void init_PCINT_Interrupt(void) {
    GIMSK |= (1 << PCIE);   // pin change interrupt enable
    PCMSK |= (1 << PCINT4); // pin change interrupt enabled for PCINT4
    sei();                  // enable interrupts
}

int main(void) {

    // ---- Initialization ----


    // ---- CPU Frequency Setup ----
#if F_CPU == 1000000UL
    #pragma message "F_CPU=1MHZ"
    CLKPR_SET(CLKPR_1MHZ);
#elif F_CPU == 8000000UL
#pragma message "F_CPU=8MHZ"
    CLKPR_SET(CLKPR_8MHZ);
#else
    #pragma message "F_CPU=????"
#error "CPU frequency should be either 1 MHz or 8 MHz"
#endif

    _delay_ms(100);

    DDRB |= (1 << PB3);
    init_PCINT_Interrupt();
    initTimer1();
    sei();

    // ---- Main Loop ----
    while (1)
    {
        if(counter_interrupt > 0x65)
        {
            counter_interrupt = 0;
            TCCR1 |= (0 << CS13) | (0 << CS12) | (0 << CS11) | (0 << CS10);
            PORTB = (1 << PB3);
        }
    }

    return 0;
}

// ============================================================================
