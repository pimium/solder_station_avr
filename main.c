
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
//#include "pid.h"


#define MAX_PWM 0xE0

ISR(TIMER0_OVF_vect)
{
  PORTB &= ~(1 << PB0);
}

static inline void initTimer0(void)
{
  // Timer 0 Configuration
  TCCR0B |= (0 << CS02) | (0 << CS01) | (1 << CS00)  // Prescaler = 1
           | (1 << WGM02); //Fast PWM
//  TCCR0A |= ((1 << COM0B1) | (0 << COM0B0) // Clear PORT OC0B on OCR0B match
//            | (1 << COM0A1) | (0 << COM0A0) // Clear PORT OC0A on OCR0A match
//            | (1 << WGM00) | (1 << WGM01));

  // Set to 'Fast PWM' mode
  TCCR0A |= (1 << WGM01) | (1 << WGM00);
  TIMSK0 |= (1 << TOIE0);

// Clear OC0B output on compare match, upwards counting.
  TCCR0A |= (1 << COM0B1);
  OCR0B = MAX_PWM/4+MAX_PWM/8;  // Filament setting
  OCR0A = MAX_PWM;
}

int main(void)
{

  // ---- Initialization ----

  //  _delay_ms(100);

  DDRB |= (1 << PB0) | (1 << PB1);

  initTimer0();

  sei();

  // ---- Main Loop ----

  while (1)
  {
      PORTB |= (1 << PB0);  // High voltage.
  }

  return 0;
}

// ============================================================================
