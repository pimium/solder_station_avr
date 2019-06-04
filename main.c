#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>

#include "adc.h"
#include "uart.h"
#include "vfd.h"
//#include "pid.h"

#define MAX_PWM 0xE0
#ifndef F_CPU
#error "F_CPU undefined, please define CPU frequency in Hz in Makefile"
#endif

/* Define UART buad rate here */
#define UART_BAUD_RATE 9600

volatile uint8_t rotate_counter = 0;
volatile uint8_t state = 0;
extern uint8_t timer_counter;
unsigned char hex[] = {'0', '1', '2', '3', '4', '5', '6', '7',
                       '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
#define SOLDER PC0

ISR(INT0_vect)
{
    if ((PIND & (1 << PD3)))
    {
        rotate_counter++;
    }
    else
    {
        rotate_counter--;
    }
    state = 1;
}

static inline void initTimer0(void)
{
    // Timer 0 configuration : Fast PWM
    TCCR0B |= (0 << CS02) | (0 << CS01) | (1 << CS00) // Prescaler = 1024
        //              | (0 << WGM02) //Fast PWM
        ;
    TCCR0A |= ((0 << COM0B1) | (0 << COM0B0) | (1 << WGM01));

    TIMSK0 |= (1 << TOIE0);
    //
    OCR0A = 0xFF; // Counter Top
    //    OCR0B = 0xFF - 0x20;
}

void initEncoder(void)
{
    DDRD &= ~((1 << PD2) | (1 << PD3));
    EIMSK = 1 << INT0;                   // Enable INT0
    EICRA = (1 << ISC01) | (1 << ISC00); // Trigger INT0 on rising edge
}

int main(void)
{

    // ---- Initialization ----

    unsigned int c;

    /*
     *  Initialize UART library, pass baudrate and AVR cpu clock
     *  with the macro
     *  UART_BAUD_SELECT() (normal speed mode )
     *  or
     *  UART_BAUD_SELECT_DOUBLE_SPEED() ( double speed mode)
     */
    uart_init(UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU));
    initEncoder();
    adc_init();
    vfd_init();

    initTimer0();

    DDRC |= (1 << SOLDER);
    PORTC |= (1 << SOLDER);

    sei();

    uart_puts("String stored in SRAM\n");

    /*
     * Transmit string from program memory to UART
     */
    //  uart_puts_P("String stored in FLASH\n");

    //    vfd_write_word(0, 0);
    //    vfd_write_special_character(2);

    uint8_t heat = 0;
    // ---- Main Loop ----

    while (1)
    {
        c = uart_getc();
        if (c & UART_NO_DATA)
        {
            /*
             * no data available from UART
             */
            if (state)
            {
                state = 0;
                uart_putc('\r');
                uart_putc(hex[(rotate_counter >> 4) & 0x0F]);
                uart_putc(hex[(rotate_counter >> 0) & 0x0F]);
                uart_putc('\n');
            }
            uint16_t adc_result = adc_read(6);
            if ((1 == heat) && (adc_result < rotate_counter))
            {
                PORTC |= (1 << SOLDER);
            }
            else
            {
                PORTC &= ~(1 << SOLDER);
            }

            if (timer_counter == 0)
            {
                uint16_t adc_hex = vfd_convert_bcd(rotate_counter);
                uart_puts("adc result: ");
                uart_putc(hex[(adc_result >> 8) & 0x0F]);
                uart_putc(hex[(adc_result >> 4) & 0x0F]);
                uart_putc(hex[(adc_result >> 0) & 0x0F]);
                //          uart_putc('\t');
                vfd_write_word(4, ((adc_hex >> 8) & 0x0F));
                vfd_write_word(5, ((adc_hex >> 4) & 0x0F));
                vfd_write_word(6, ((adc_hex >> 0) & 0x0F));
                uart_putc('\n');
                uart_putc('\r');
                timer_counter = 0xf0;

                vfd_write_special_character(7);
                vfd_write_special_character(4);
            }
        }
        else
        {
            /*
             * new data available from UART
             * check for Frame or Overrun error
             */
            if (c & UART_FRAME_ERROR)
            {
                /* Framing Error detected, i.e no stop bit detected */
                uart_puts_P("UART Frame Error: ");
            }
            if (c & UART_OVERRUN_ERROR)
            {
                /*
                 * Overrun, a character already present in the UART UDR register
                 * was
                 * not read by the interrupt handler before the next character
                 * arrived,
                 * one or more received characters have been dropped
                 */
                uart_puts_P("UART Overrun Error: ");
            }
            if (c & UART_BUFFER_OVERFLOW)
            {
                /*
                 * We are not reading the receive buffer fast enough,
                 * one or more received character have been dropped
                 */
                uart_puts_P("Buffer overflow error: ");
            }
            /*
             * send received character back
             */
            if (c == 'a')
            {
                heat = 1;
                //        PORTC |= (1 << SOLDER);
            }
            else
            {
                heat = 0;
                //        PORTC &= ~(1 << SOLDER);
            }
            uart_puts("\nI get: ");
            uart_putc((unsigned char)c);
            uart_putc('\n');

            vfd_write_word(0, 4);
            vfd_write_word(1, 1);
            vfd_write_word(2, 2);
            vfd_write_word(3, 3);
        }
    }

    return 0;
}

// ============================================================================
