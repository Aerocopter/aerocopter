#ifndef DELAY_H
#define DELAY_H

#include <stdint.h>

/*
 * Crude blocking delay.
 * Time is approximate and CPU-frequency dependent.
 * Suitable for sensor startup and calibration delays.
 */
void delay_ms(uint32_t ms);

#endif
