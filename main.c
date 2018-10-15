#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>

#include "hv5812.h"

// define some macros
#define BAUD 9600                              // define baud
#define BAUDRATE ((F_CPU) / (BAUD * 16UL) - 1) // set baud rate value for UBRR

#ifndef LED
#define LED PB2
#endif
#ifndef SOLDER
#define SOLDER PB1
#endif

volatile static uint8_t measuring = 0;
volatile static uint16_t value;

uint16_t adc_read(uint8_t channel);
void uart_transmit(unsigned char data);

// ----------------------------------------------------------------------------

ISR(TIMER2_OVF_vect)
{
    if (OCR2A)
    {
        PORTB |= (1 << SOLDER);
    }
}

ISR(TIMER2_COMPB_vect)
{
    PORTB &= ~(1 << SOLDER);
    PORTB |= (1 << LED);
    measuring = 1;
}

ISR(TIMER2_COMPA_vect)
{
    PORTB &= ~(1 << SOLDER);
}

static inline void initTimer0(void)
{
    // Timer 0 konfigurieren
    TCCR0B |= (1 << CS02) | (0 << CS01) | (0 << CS00); // Prescaler
    TCCR0A |= ((1 << COM0A1) | (0 << COM0A0) | (1 << WGM00));
            // Overflow Interrupt erlauben // Output Compare B Match Interrupt Enable
    TIMSK0 |= ((1 << TOIE0) | (1 << OCIE0B));
    OCR0A = 0x00;
}

static inline void initTimer2(void)
{
    // Timer 0 konfigurieren
    TCCR2B |= ((1 << CS22) | (0 << CS21) | (0 << CS20)); // Prescaler

    // Overflow Interrupt erlauben // Output Compare A and B Match Interrupt Enable
    TIMSK2 |= ((1 << TOIE2) | (1 << OCIE2A) | (1 << OCIE2B));
    OCR2B = 0x80;
    OCR2A = 0x00;
}

static inline void adc_init()
{
    // AREF = AVcc
    //    ADMUX = (1<<REFS0);

    // ADC Enable and prescaler of 128
    // 16000000/128 = 125000
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

// function to initialize UART
void uart_init(void)
{
    UBRR0H = (BAUDRATE >> 8);             // shift the register right by 8 bits
    UBRR0L = BAUDRATE;                    // set baud rate
    UCSR0B = (1 << TXEN0) | (1 << RXEN0); // enable receiver and transmitter
    UCSR0C = (1 << UCSZ00) | (1 << UCSZ01); // 8bit data format
}

// function to send data
void uart_transmit(unsigned char data)
{
    while (!(UCSR0A & (1 << UDRE0)))
        ;        // wait while register is free
    UDR0 = data; // load data in the register
}

// function to receive data
unsigned char uart_recieve(void)
{
    while (!(UCSR0A) & (1 << RXC0))
        ;        // wait while data is being received
    return UDR0; // return 8-bit data
}

uint16_t adc_read(uint8_t channel)
{
    // select the corresponding channel 0~7
    // ANDing with ’7′ will always keep the value
    // of ‘channel’ between 0 and 7
    channel &= 0x07;                  // AND operation with 7
    ADMUX = (ADMUX & 0xF8) | channel; // clears the bottom 3 bits before ORing

    // start single convertion
    // write ’1′ to ADSC
    ADCSRA |= (1 << ADSC);

    // wait for conversion to complete
    // ADSC becomes ’0′ again
    // till then, run loop continuously
    while (ADCSRA & (1 << ADSC))
        ;

    return (ADC);
}

uint16_t adc_readmean()
{
    uint16_t result = 0;
    for (int i = 0; i < 16; ++i)
    {
        result += adc_read(0);
    }
    return result >> 4;
}

int hex[] = {'0', '1', '2', '3', '4', '5', '6', '7',
             '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
int main(void)
{

    DDRB |= (1 << LED) | (1 << SOLDER);
    PORTB &= ~(1 << SOLDER);
    PORTB &= ~(1 << LED);

    //the noise in the system
    float Q = 0.022;
    float R = 0.917;
//    Kalman-Gain
    float K = 0.143359;

    float x_temp_est;
    float x_est;
    //initialize with a measurement
    float x_est_last = 0;
    uint16_t z_measured; //the 'noisy' value we measured

    uint8_t j = 0;

    //  init_IRQ0();
    initTimer2();
    //  hv5812_init();
    uart_init();
    adc_init();
    sei();

    uint16_t soll = 300;
    float y, ealt, e = 0;
    float esum = 0;
    uint8_t Kp = 15;
    float Ki = 0.0025, Kd = 0.02;
    float Ta = 1.0;

    // ---- Main Loop ----

    while (1)
    {
        if (measuring)
        {
            for (int i = 0; i < 1; ++i) {
                //do a prediction
                x_temp_est = x_est_last;
                //measure
                z_measured = adc_read(0);
                //correct
                x_est = x_temp_est + K * (z_measured - x_temp_est);
                //update our last's
                // P_last = P;
                x_est_last = x_est;
            }

            e = soll - x_est;
            esum = esum + e;
            y = Kp * e + Ki * Ta * esum + Kd * (e - ealt)/Ta;
            ealt = e;
            if (esum > 10000)
                esum = 10000;
            if(esum < -10000)
                esum = -10000;
            if (y > 0x70)
                y = 0x70;
            if (y < 0)
                y = 0;

            OCR2A = y;
            uart_transmit(hex[((uint16_t) x_est >> 8) & 0xf]);
            uart_transmit(hex[((uint16_t) x_est >> 4) & 0xf]);
            uart_transmit(hex[((uint16_t) x_est >> 0) & 0xf]);
            uart_transmit('\r');
            uart_transmit('\n');

            //          uart_transmit(hex[(j++) & 0xf]);
            //          PORTB &= ~(1 << LED);

            uart_transmit(hex[(j++) & 0xf]);
            measuring = 0;
            PORTB &= ~(1 << LED);
        }
    }

    return 0;
}

//abw = soll - ist;
//y = Kp * abw;
//
//esum = esum + abw;
//y = y + Ki * esum;
//
//y = y + Kd * (soll - stgAlt) / deltaT;
//stgAlt = soll;

// ============================================================================
