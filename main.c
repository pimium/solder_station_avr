#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>

#include "adc.h"
#include "seven_seg.h"
#include "uart.h"
//#include "pid.h"
//#define SEVEN_SEG_DIO PC1
//#define SEVEN_SEG_MCLR PC2
#define MAX_PWM 0xE0
#ifndef F_CPU
#error "F_CPU undefined, please define CPU frequency in Hz in Makefile"
#endif

/* Define UART buad rate here */
#define UART_BAUD_RATE 9600

volatile uint16_t rotate_counter = 0;
volatile uint8_t state = 0;
volatile uint8_t timer_counter = 0;

unsigned char hex[] = {'0', '1', '2', '3', '4', '5', '6', '7',
                       '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
#define SOLDER PC0

ISR(INT1_vect)
{
    if ((PIND & (1 << PD4)))
    {
        rotate_counter++;
    }
    else
    {
        rotate_counter--;
    }
    rotate_counter = rotate_counter & 0x03FF;
    state = 1;
}

ISR(TIMER0_COMPA_vect) // Timer1 ISR
{
    DDRC |= (1 << PC3);
    PORTC ^= (1 << PC3);
}

ISR(TIMER0_COMPB_vect) // Timer1 ISR
{
    PORTC &= ~(1 << SOLDER);
}

ISR(TIMER0_OVF_vect) // Timer1 ISR
{
    PORTC |= (1 << SOLDER);
    timer_counter--;
    seven_seg_handle();
}

static inline void initTimer0(void)
{

    TCCR0B |= (0 << CS02) | (1 << CS01) | (0 << CS00) // Prescaler = 1024
              | (0 << WGM02)                          // Fast PWM
        ;
    TCCR0A |= ((0 << COM0B1) | (0 << COM0B0) | (1 << WGM01) | (1 << WGM00));

    TIMSK0 |= (1 << TOIE0) | (1 << OCIE0A) | (1 << OCIE0B);
    //
    OCR0A = 0xE0; // Counter Top
    OCR0B = 0x00; // Counter Top
    //    OCR0B = 0xFF - 0x20;
}
//
void initEncoder(void)
{
    DDRD &= ~((1 << PD4) | (1 << PD3));
    EIMSK = 1 << INT1;                   // Enable INT0
    EICRA = (1 << ISC11) | (1 << ISC10); // Trigger INT0 on rising edge
}

uint8_t get_timer_count(void) { return timer_counter; }

void set_timer_count(uint8_t value) { timer_counter = value; }

int main(void)
{

    // ---- Initialization ----

    // initial values for the kalman filter
    float x_est_last = 0;
    float P_last = 0;
    // the noise in the system
    float Q = 0.022;
    float R = 0.617;

    float K;
    float P;
    float P_temp;
    float x_temp_est;
    float x_est;
    float z_measured; // the 'noisy' value we measured
    //
    //
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
    seven_seg_init();

    initTimer0();

    DDRC |= (1 << SOLDER);
    PORTC &= ~(1 << SOLDER);

    sei();
    //
    uart_puts("String stored in SRAM\n");

    /*
     * Transmit string from program memory to UART
     */
    uart_puts_P("String stored in FLASH\n");
    -write_loop_byte(1, 0);

    uint8_t heat = 0;
    uint16_t adc_result = 0;
    //    // ---- Main Loop ----
    //
    //    x_est_last = adc_read(6);
    //
    while (1)
    {

#if 1
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
                uart_putc(hex[(rotate_counter >> 12) & 0x0F]);
                uart_putc(hex[(rotate_counter >> 8) & 0x0F]);
                uart_putc(hex[(rotate_counter >> 4) & 0x0F]);
                uart_putc(hex[(rotate_counter >> 0) & 0x0F]);
                uart_putc('\n');
            }

            // do a prediction
            x_temp_est = x_est_last;
            P_temp = P_last + Q;
            // calculate the Kalman gain
            K = P_temp * (1.0 / (P_temp + R));
            // measure
            z_measured = adc_read();
            // correct
            x_est = x_temp_est + K * (z_measured - x_temp_est);
            P = (1 - K) * P_temp;

            // update our last's
            P_last = P;
            x_est_last = x_est;

            adc_result = x_est;
            //            adc_result = adc_read();

            if ((1 == heat) && (adc_result < rotate_counter))
            {
                OCR0B = 0x80;
                write_loop_byte(1, 0xF6);
            }
            else
            {
                //                PORTC &= ~(1 << SOLDER);
                OCR0B = 0x00;
                write_loop_byte(1, 0x80);
            }

            if (get_timer_count() < 0x80)
            {
                seven_seg_reset();
                //                uint16_t adc_hex =
                //                vfd_convert_bcd(adc_result);
                //                uint16_t rotate_hex =
                //                vfd_convert_bcd(rotate_counter);
                uart_puts("adc result: ");
                uart_putc(hex[(adc_result >> 12) & 0x0F]);
                uart_putc(hex[(adc_result >> 8) & 0x0F]);
                uart_putc(hex[(adc_result >> 4) & 0x0F]);
                uart_putc(hex[(adc_result >> 0) & 0x0F]);
                uart_puts("\trotate_counter: ");
                uart_putc(hex[(rotate_counter >> 12) & 0x0F]);
                uart_putc(hex[(rotate_counter >> 8) & 0x0F]);
                uart_putc(hex[(rotate_counter >> 4) & 0x0F]);
                uart_putc(hex[(rotate_counter >> 0) & 0x0F]);

                //          uart_putc('\t');
                //                seven_seg_write_display(adc_read());
                uart_putc('\n');
                uart_putc('\r');
                set_timer_count(0x0f);
            }
            seven_seg_write_display(adc_result);
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
                write_loop_byte(1, 0x80);
            }
            else
            {
                heat = 0;
                write_loop_byte(1, 0);
            }
            uart_puts("\nI get: ");
            uart_putc((unsigned char)c);
            uart_putc('\r');
            //            vfd_write_word(0, 4);
            //            vfd_write_word(1, 1);
            //            vfd_write_word(2, 2);
            //            vfd_write_word(3, 3);
        }

#endif
    }

    return 0;
}

// ============================================================================
