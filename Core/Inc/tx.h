#include "main.h"

#define DMAMUX1_TIM3_UP_ID  27


/* =========================================================
 * BSRR BIT DEFINES
 * SET   = bit N      (lower 16 bits)
 * CLEAR = bit N+16   (upper 16 bits)
 * ========================================================= */

/* PORTA */
#define PA0_SET   (1UL << 0)
#define PA0_CLR   (1UL << 16)
#define PA3_SET   (1UL << 3)
#define PA3_CLR   (1UL << 19)
#define PA15_SET  (1UL << 15)
#define PA15_CLR  (1UL << 31)

/* PORTB */
#define PB3_SET   (1UL << 3)
#define PB3_CLR   (1UL << 19)
#define PB10_SET  (1UL << 10)
#define PB10_CLR  (1UL << 26)
#define PB11_SET  (1UL << 11)
#define PB11_CLR  (1UL << 27)

/* PORTD */
#define PD12_SET  (1UL << 12)
#define PD12_CLR  (1UL << 28)
#define PD13_SET  (1UL << 13)
#define PD13_CLR  (1UL << 29)
#define PD14_SET  (1UL << 14)
#define PD14_CLR  (1UL << 30)
#define PD15_SET  (1UL << 15)
#define PD15_CLR  (1UL << 31)

/* PORTF */
#define PF0_SET   (1UL << 0)
#define PF0_CLR   (1UL << 16)
#define PF1_SET   (1UL << 1)
#define PF1_CLR   (1UL << 17)
#define PF2_SET   (1UL << 2)
#define PF2_CLR   (1UL << 18)
#define PF6_SET   (1UL << 6)
#define PF6_CLR   (1UL << 22)
#define PF12_SET  (1UL << 12)
#define PF12_CLR  (1UL << 28)
#define PF13_SET  (1UL << 13)
#define PF13_CLR  (1UL << 29)

void transmit(uint32_t pulses, float *delays);