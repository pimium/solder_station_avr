#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>

#include "adc.h"
#include "lcd.h"
#include "rotary_encoder.h"
#include "seven_seg.h"
#include "uart.h"

#define MAX_PWM 0xE0
#ifndef F_CPU
#error "F_CPU undefined, please define CPU frequency in Hz in Makefile"
#endif

/* Define UART buad rate here */
#define UART_BAUD_RATE 9600

volatile uint8_t state = 0;
volatile uint8_t timer_counter = 0;

uint8_t heat = 0;
uint8_t schwelle = 0;

#define SOLDER_L PB2
#define SOLDER_H PB1

// ISR(TIMER0_COMPA_vect) // Timer1 ISR
//{
//    DDRC |= (1 << PC3);
//    PORTC ^= (1 << PC3);
//}
//
// ISR(TIMER0_COMPB_vect) // Timer1 ISR
//{
//    PORTC &= ~(1 << PC4);
//    PORTB &= ~(1 << SOLDER_L);
//}

ISR(TIMER1_OVF_vect) // Timer1 ISR
{
    DDRC |= (1 << PC4);
    PORTC |= (1 << PC4);
    timer_counter--;
    seven_seg_handle_byte();
    seven_handle_word();
}

static inline void initTimer0(void)
{

    TCCR0B |= (0 << CS02) | (1 << CS01) | (0 << CS00) // Prescaler = 1024
              | (0 << WGM02)                          // Fast PWM
        ;
    TCCR0A |= ((0 << COM0B1) | (0 << COM0B0) | (1 << WGM01) | (1 << WGM00));

    TIMSK0 |= (1 << TOIE0) | (1 << OCIE0A) | (1 << OCIE0B);
    //
    OCR0A = 0xB0; // Counter Top
    OCR0B = 0x80; // Counter Top
    //    OCR0B = 0xFF - 0x20;
}

static inline void initTimer1(void)
{
    DDRB |= (1 << SOLDER_L) | (1 << SOLDER_H);
    PORTB &= ~((1 << SOLDER_L) | (1 << SOLDER_H));

    TCCR1B |= (0 << CS02) | (1 << CS01) | (0 << CS00) // Prescaler = 1024
              | (0 << WGM12)                          // Fast PWM
        ;
    TCCR1A |= ((1 << COM1A1) | (1 << COM1A0) | (1 << COM1B1) | (1 << COM1B0) |
               (0 << WGM11) | (1 << WGM10));

    TIMSK1 |= (1 << TOIE1) | (0 << OCIE1A) | (0 << OCIE1B);
    //
    OCR1A = 0xff; // Counter Top
    OCR1B = 0xff; // Counter Top
    //    OCR0B = 0xFF - 0x20;
}

uint8_t get_timer_count(void) { return timer_counter; }

void set_timer_count(uint8_t value) { timer_counter = value; }

int main(void)
{

    // ---- Initialization ----
    //
    //    // initial values for the kalman filter
    //    float x_est_last = 0;
    //    float P_last = 0;
    //    // the noise in the system
    //    float Q = 0.022;
    //    float R = 0.617;
    //
    //    float K;
    //    float P;
    //    float P_temp;
    //    float x_temp_est;
    //    float x_est;
    //    float z_measured; // the 'noisy' value we measured
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

    //    initTimer0();
    initTimer1();
    /* initialize display, cursor off */
    lcd_init(LCD_DISP_ON);

    sei();
    //
    uart_puts("String stored in SRAM\n");

    /* clear display and home cursor */
    lcd_clrscr();

    /* put string to display (line 1) with linefeed */
    lcd_puts("LCD Test Line 1\n");

    /*
     * Transmit string from program memory to UART
     */
    uart_puts_P("String stored in \nFLASH\n");
    write_byte_to_register(1, 0);

    uint16_t adc_result = 0;
    uint16_t rotateCounter = 0;

    lcd_clrscr(); /* clear display home cursor */

    //    // ---- Main Loop ----
    lcd_puts_P("Act1: 123 2: 345\n");
    lcd_puts_P("Set1: 567 2: 567");
    while (1)
    {

        //        lcd_clrscr();     /* clear display home cursor */
        lcd_home();

/* put string from program memory to display */

/* move BOTH lines one position to the left */
//        lcd_command(LCD_MOVE_DISP_LEFT);
#if 1
        c = uart_getc();
        if (c & UART_NO_DATA)
        {
            /*
             * no data available from UART
             */

            //            // do a prediction
            //            x_temp_est = x_est_last;
            //            P_temp = P_last + Q;
            //            // calculate the Kalman gain
            //            K = P_temp * (1.0 / (P_temp + R));
            //            // measure
            //            z_measured = adc_read();
            //            // correct
            //            x_est = x_temp_est + K * (z_measured - x_temp_est);
            //            P = (1 - K) * P_temp;
            //
            //            // update our last's
            //            P_last = P;
            //            x_est_last = x_est;
            //
            //            adc_result = x_est;
            adc_result = adc_read();
            rotateCounter = get_rotate_counter();

            if (adc_result < rotateCounter)
            {
                schwelle = rotateCounter - adc_result;
                if (schwelle > 0x80)
                    schwelle = 0x80;
            }
            else
                schwelle = 0;
            schwelle = 255 - schwelle;

            if ((1 == heat) && (schwelle > 0))
            {
                OCR1B = schwelle;
                write_byte_to_register(1, 0xF6);
            }
            else
            {
                OCR1B = 0xff;
                write_byte_to_register(1, 0x80);
            }

            if (get_timer_count() < 0x80)
            {
                //                seven_seg_reset();
                //                uint16_t adc_hex =
                //                vfd_convert_bcd(adc_result);
                //                uint16_t rotate_hex =
                //                vfd_convert_bcd(rotateCounter);

                uart_puts("adc result: ");
                uart_putc(hex((adc_result >> 12) & 0x0F));
                uart_putc(hex((adc_result >> 8) & 0x0F));
                uart_putc(hex((adc_result >> 4) & 0x0F));
                uart_putc(hex((adc_result >> 0) & 0x0F));
                uart_puts("\trotateCounter: ");
                uart_putc(hex((rotateCounter >> 12) & 0x0F));
                uart_putc(hex((rotateCounter >> 8) & 0x0F));
                uart_putc(hex((rotateCounter >> 4) & 0x0F));
                uart_putc(hex((rotateCounter >> 0) & 0x0F));

                //          uart_putc('\t');
                //                seven_seg_write_display(adc_read());
                uart_putc('\n');
                uart_putc('\r');
                set_timer_count(0x0f);
            }
            seven_seg_write_display(adc_result);
            lcd_gotoxy(5, 0);
            lcd_put_temp(adc_result);
            lcd_gotoxy(5, 1);
            lcd_put_temp(rotateCounter);
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
                write_byte_to_register(1, 0x80);
            }
            else
            {
                heat = 0;
                write_byte_to_register(1, 0);
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
