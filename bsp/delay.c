#include "delay.h"

/*
 * Simple blocking delay using empty loops.
 * Not precise. Intended for short hardware delays only.
 *
 * Adjust LOOP_PER_MS according to CPU frequency.
 */
#define LOOP_PER_MS 28000U   /* for ~168 MHz MCU */

void delay_ms(uint32_t ms)
{
    volatile uint32_t i;

    while (ms--)
    {
        for (i = 0; i < LOOP_PER_MS; i++)
        {
            __asm__("nop");
        }
    }
}
