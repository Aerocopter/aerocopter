#include "drv_icm42688.h"
#include "bsp_spi.h"   /* your SPI interface */
#include "delay.h"

/* Registers */
#define REG_WHO_AM_I            0x75
#define REG_BANK_SEL            0x76
#define REG_DEVICE_CONFIG       0x11
#define REG_PWR_MGMT0           0x4E
#define REG_GYRO_CONFIG0        0x4F
#define REG_ACCEL_CONFIG0       0x50
#define REG_GYRO_CONFIG1        0x51
#define REG_GYRO_ACCEL_CONFIG0  0x52
#define REG_ACCEL_DATA_X1       0x1F

#define REG_GYRO_DATA_X1        0x25


#define BANK0 0x00
#define BANK1 0x10
#define BANK2 0x20

/* SPI helpers */

static void icm_write_reg(uint8_t reg, uint8_t val)
{
    bsp_spi3_cs_low();
    bsp_spi3_transfer(reg & 0x7F);
    bsp_spi3_transfer(val);
    bsp_spi3_cs_high();
}

static uint8_t icm_read_reg(uint8_t reg)
{
    uint8_t val;
    bsp_spi3_cs_low();
    bsp_spi3_transfer(reg | 0x80);
    val = bsp_spi3_transfer(0xFF);
    bsp_spi3_cs_high();
    return val;
}

static void icm_read_buf(uint8_t reg, uint8_t *buf, uint16_t len)
{
    bsp_spi3_cs_low();
    bsp_spi3_transfer(reg | 0x80);
    for (uint16_t i = 0; i < len; i++)
        buf[i] = bsp_spi3_transfer(0xFF);
    bsp_spi3_cs_high();
}

static void icm_bank(uint8_t bank)
{
    icm_write_reg(REG_BANK_SEL, bank);
}

/* Public functions */

void drv_icm42688_init(void)
{
    delay_ms(10);

    icm_bank(BANK0);

    /* Soft reset */
    icm_write_reg(REG_DEVICE_CONFIG, 0x01);
    delay_ms(10);   /* extra delay for reset stability */

    /* Verify WHO_AM_I */
    if (icm_read_reg(REG_WHO_AM_I) != 0x47)
    {
        /* Device not found — return for debug instead of blocking */
        return;
    }

    icm_bank(BANK0);
    /* Enable accel + gyro */
    icm_write_reg(REG_PWR_MGMT0, 0x0F);

    /* Gyro: ±2000 dps, 1kHz */
    icm_write_reg(REG_GYRO_CONFIG0, 0x06);

    /* Accel: ±16g, 1kHz */
    icm_write_reg(REG_ACCEL_CONFIG0, 0x06);

    delay_ms(10);

    icm_bank(BANK0);
    /* Digital filters */
    icm_write_reg(0x51, 0x56);
    icm_write_reg(0x52, 0x11);
    icm_write_reg(0x53, 0x0D);
    icm_write_reg(0x63, 0x00);
    icm_write_reg(0x65, 0x08);
    icm_write_reg(0x66, 0x00);
    icm_write_reg(0x68, 0x00);
    icm_write_reg(0x69, 0x00);

    /* Anti-alias filters */
    icm_bank(BANK1);
    icm_write_reg(0x0B, 0xA0);
    icm_write_reg(0x0C, 0x0C);
    icm_write_reg(0x0D, 0x90);
    icm_write_reg(0x0E, 0x80);

    icm_bank(BANK2);
    icm_write_reg(0x03, 0x18);
    icm_write_reg(0x04, 0x90);
    icm_write_reg(0x05, 0x80);

    icm_bank(BANK0);
    icm_write_reg(0x51, 0x12);
    icm_write_reg(0x53, 0x05);
    icm_write_reg(0x52, 0x33);

    icm_bank(BANK0);
    icm_write_reg(0x4E, 0x0F);

    delay_ms(10);
}

void drv_icm42688_read_raw(drv_icm42688_raw_t *raw)
{
    uint8_t buf[6];

    icm_bank(BANK0);

    /* Burst read accel + temp + gyro (same sample frame) */
    icm_read_buf(REG_ACCEL_DATA_X1, buf, 6);

    raw->acc[0] = (int16_t)(((int16_t)buf[0] << 8) | buf[1]);
    raw->acc[1] = (int16_t)(((int16_t)buf[2] << 8) | buf[3]);
    raw->acc[2] = (int16_t)(((int16_t)buf[4] << 8) | buf[5]);


    icm_read_buf(REG_GYRO_DATA_X1, buf, 6);
    raw->gyro[0] = (int16_t)(((int16_t)buf[0] << 8) | buf[1]);
    raw->gyro[1] = (int16_t)(((int16_t)buf[2] << 8) | buf[3]);
    raw->gyro[2] = (int16_t)(((int16_t)buf[4] << 8) | buf[5]);



    //uart_send_string("gyro[0]:");
    //uart_send_uint32(raw->gyro[0]);
    //uart_send_string("\r\n");

}
