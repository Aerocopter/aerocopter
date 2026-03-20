#ifndef __BSP_SPI_H
#define __BSP_SPI_H

#include <stdint.h>

/* SPI Mode */
typedef enum
{
    SPI_MODE_0 = 0,  // CPOL=0 CPHA=0
    SPI_MODE_1,      // CPOL=0 CPHA=1
    SPI_MODE_2,      // CPOL=1 CPHA=0
    SPI_MODE_3       // CPOL=1 CPHA=1
} bsp_spi_mode_t;

/* ================= SPI1 ================= */
void bsp_spi1_cs_low(void);
void bsp_spi1_cs_high(void);
void bsp_spi1_init(bsp_spi_mode_t mode);
uint8_t bsp_spi1_transfer(uint8_t data);
void bsp_spi1_transfer_buffer(uint8_t *tx, uint8_t *rx, uint16_t len);

/* ================= SPI2 ================= */
void bsp_spi2_init(bsp_spi_mode_t mode);
uint8_t bsp_spi2_transfer(uint8_t data);
void bsp_spi2_transfer_buffer(uint8_t *tx, uint8_t *rx, uint16_t len);
void bsp_spi2_cs_low(void);
void bsp_spi2_cs_high(void);

/* ================= SPI3 ================= */
void bsp_spi3_init(bsp_spi_mode_t mode);
uint8_t bsp_spi3_transfer(uint8_t data);
void bsp_spi3_transfer_buffer(uint8_t *tx, uint8_t *rx, uint16_t len);
void bsp_spi3_cs_low(void);
void bsp_spi3_cs_high(void);

#endif
