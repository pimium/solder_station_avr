
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>

// ----------------------------------------------------------------------------

volatile uint16_t counter_interrupt = 0;
volatile uint8_t armed = 0;

// ----------------------------------------------------------------------------

static inline void initTimer0(void)
{
  // Timer 0 Configuration
  TCCR0B |= (0 << CS02) | (1 << CS01) | (0 << CS00)  // Prescaler = 1
            | (0 << WGM02); //Fast PWM
  TCCR0A |= ((1 << COM0B1) | (0 << COM0B0) // Clear PORT OC0B on OCR0B match
            | (1 << COM0A1) | (0 << COM0A0) // Clear PORT OC0A on OCR0A match
            | (1 << WGM00) | (1 << WGM01));

  OCR0A = 0x80; // Counter Top
//  OCR0B = 0x80;
}


int main(void)
{

  // ---- Initialization ----

  //  _delay_ms(100);

  DDRB |= (1 << PB2) | (1 << PB5);

  initTimer0();
  sei();

  // ---- Main Loop ----

  while (1)
  {
  }

  return 0;
}

// ============================================================================
