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
