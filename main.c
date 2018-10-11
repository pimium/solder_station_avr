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

#include "hv5812.h"

// define some macros
#define BAUD 9600                                   // define baud
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)            // set baud rate value for UBRR

#ifndef LED
#define LED PB2
#endif
#ifndef SOLDER
#define SOLDER PB1
#endif


volatile static uint8_t armed = 0;
volatile static uint16_t value;

uint16_t adc_read(uint8_t channel);

// ----------------------------------------------------------------------------


ISR(TIMER0_OVF_vect)
{
  cli();
//    PORTB ^= (1 << LED);
    PORTB |= (1 << SOLDER);
  sei();
}

ISR(TIMER0_COMPB_vect)
{
    armed = 1;
    PORTB |= (1 << LED);
    value = adc_read(0);
}

ISR(TIMER0_COMPA_vect)
{
    cli();
    PORTB &= ~(1 << SOLDER);
    sei();
}

static inline void initTimer0(void)
{
  // Timer 0 konfigurieren
  TCCR0B |= (1 << CS02) | (0 << CS01) | (1 << CS00); // Prescaler

  // Overflow Interrupt erlauben // Output Compare A Match Interrupt Enable
  TIMSK0 |= ((1 << TOIE0) | (1 << OCIE0A) | (1 << OCIE0B));
  OCR0B = 0x80;
  OCR0A = 0x10;
}

static inline void adc_init()
{
    // AREF = AVcc
//    ADMUX = (1<<REFS0);

    // ADC Enable and prescaler of 128
    // 16000000/128 = 125000
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}

// function to initialize UART
void uart_init (void)
{
    UBRR0H = (BAUDRATE>>8);                      // shift the register right by 8 bits
    UBRR0L = BAUDRATE;                           // set baud rate
    UCSR0B= (1<<TXEN0)|(1<<RXEN0);                // enable receiver and transmitter
    UCSR0C= (1<<UCSZ00)|(1<<UCSZ01);   // 8bit data format
}

// function to send data
void uart_transmit (unsigned char data)
{
    while (!( UCSR0A & (1<<UDRE0)));                // wait while register is free
    UDR0 = data;                                   // load data in the register
}

// function to receive data
unsigned char uart_recieve (void)
{
    while(!(UCSR0A) & (1<<RXC0));                   // wait while data is being received
    return UDR0;                                   // return 8-bit data
}

uint16_t adc_read(uint8_t channel)
{
    // select the corresponding channel 0~7
    // ANDing with ’7′ will always keep the value
    // of ‘channel’ between 0 and 7
    channel &= 0x07;  // AND operation with 7
    ADMUX = (ADMUX & 0xF8) | channel; // clears the bottom 3 bits before ORing

    // start single convertion
    // write ’1′ to ADSC
    ADCSRA |= (1<<ADSC);

    // wait for conversion to complete
    // ADSC becomes ’0′ again
    // till then, run loop continuously
    while(ADCSRA & (1<<ADSC));

    return (ADC);
}

int hex[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
int main(void)
{
    volatile int j;
  //  _delay_ms(100);

//  DDRD |= ((1 << TRIAC_GATE_N) | (1 << TRIAC_GATE_P));
    DDRB |= (1 << LED) | (1 << SOLDER);
  //  PORTB &= ~(1 << PB2);
//
//  init_IRQ0();
  initTimer0();
  //  hv5812_init();
  uart_init();
  adc_init();
  sei();

  //  PORTB |= (1 << PB1);
  // ---- Main Loop ----

  while (1)
  {
      if(armed) {

          uart_transmit(0x30);
          uart_transmit('x');
          uart_transmit(hex[(value >> 12) & 0xf]);
          uart_transmit(hex[(value >> 8) & 0xf]);
          uart_transmit(hex[(value >> 4) & 0xf]);
          uart_transmit(hex[(value >> 0) & 0xf]);
          uart_transmit('\r');
          uart_transmit('\n');

          uart_transmit(hex[(j++) & 0xf]);
          PORTB &= ~(1 << LED);
          armed = 0;
      }

  }

  return 0;
}

// ============================================================================
