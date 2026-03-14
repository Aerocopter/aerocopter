#ifndef __BSP_IIC_H
#define __BSP_IIC_H

#include <stdint.h>

void bsp_i2c3_init(void);

void bsp_i2c3_write(uint8_t addr, uint8_t reg, uint8_t data);
uint8_t bsp_i2c3_read(uint8_t addr, uint8_t reg);
void bsp_i2c3_read_buffer(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);

#endif
