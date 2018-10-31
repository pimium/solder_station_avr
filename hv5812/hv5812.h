//
// Created by pimi on 15.03.18.
//

#ifndef HV5812_H
#define HV5812_H
#include <avr/io.h>

// ============================================================================

/*/Pin Connection  7-LT-44Z
 a    28
 b    27
 c    25
 d    24
 e    22
 f    21
 g    19
 1G   30
 1G   26
 2G   23
 3G   20
 4G   16
 5G   13
 6G    8
 7G    5
 S10  32   °C
 //S11  31 °F
 S12  29   °C
 S8    4  Kolben
 S7    6  Start Stop
 S5    9  Ofen
 S4    4  Glocke
*/
#define FI_A 6
#define FI_B 5
#define FIC 0
#define FI_D 1
#define FI_E 10
#define FI_F 7
#define FI_G 2
#define POS1 4
#define POS2 7
#define POS3 3
#define POS4 1
#define POS5 0
#define POS6 13
#define POS7 11


#ifndef HV5812_DDR
#define HV5812_DDR DDRC
#endif
#ifndef HV5812_BLA
#define HV5812_BLA PC3
#endif
#ifndef HV5812_STR
#define HV5812_STR PC2
#endif
#ifndef HV5812_CLK
#define HV5812_CLK PC1
#endif
#ifndef HV5812_DAT
#define HV5812_DAT PC4
#endif

void hv5812_init(void);
void hv5812_send_byte(uint8_t byte);
void hv5812_write_char(uint8_t position, uint8_t display_value);
void hv5812_blank(uint8_t clear);

#endif // HV5812_H
