//
// Created by pimi on 15.03.18.
//

#ifndef HV5812_H
#define HV5812_H
#include <avr/io.h>

// ============================================================================

//// -----(+)--------------->	// Vcc,	Pin 1 on SSD1306 Board
//// -----(-)--------------->	// GND,	Pin 2 on SSD1306 Board
#ifndef HV5812_BLA
#define HV5812_BLA PB3
#endif
#ifndef HV5812_STR
#define HV5812_STR PB2
#endif
#ifndef HV5812_CLK
#define HV5812_CLK PB1
#endif
#ifndef HV5812_DAT
#define HV5812_DAT PB0
#endif

void hv5812_init(void);
void hv5812_send_byte(uint8_t byte);
void hv5812_blank(uint8_t clear);

#endif // HV5812_H
