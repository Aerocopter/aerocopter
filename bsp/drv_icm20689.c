#include "drv_icm20689.h"
#include "bsp_spi.h"

/* =========================================================
   SPI2 interface
   ========================================================= */

#define ICM_CS_LOW()    bsp_spi2_cs_low()
#define ICM_CS_HIGH()   bsp_spi2_cs_high()
#define ICM_TXRX(x)     bsp_spi2_transfer(x)

/* =========================================================
   Register Map (ICM20689)
   ========================================================= */

#define REG_WHO_AM_I        0x75
#define REG_PWR_MGMT_1      0x6B
#define REG_PWR_MGMT_2      0x6C
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_CONFIG2   0x1D
#define REG_SMPLRT_DIV      0x19
#define REG_ACCEL_XOUT_H    0x3B

#define WHO_AM_I_VALUE      0x98

/* =========================================================
   Low Level
   ========================================================= */

static void icm_write_reg(uint8_t reg, uint8_t val)
{
    ICM_CS_LOW();
    ICM_TXRX(reg & 0x7F);
    ICM_TXRX(val);
    ICM_CS_HIGH();
}

static uint8_t icm_read_reg(uint8_t reg)
{
    uint8_t val;

    ICM_CS_LOW();
    ICM_TXRX(reg | 0x80);
    val = ICM_TXRX(0xFF);
    ICM_CS_HIGH();

    return val;
}

static void icm_read_buf(uint8_t reg, uint8_t *buf, uint8_t len)
{
    ICM_CS_LOW();
    ICM_TXRX(reg | 0x80);

    for(uint8_t i = 0; i < len; i++)
        buf[i] = ICM_TXRX(0xFF);

    ICM_CS_HIGH();
}

/* =========================================================
   Public API
   ========================================================= */

uint8_t drv_icm20689_who_am_i(void)
{
    return icm_read_reg(REG_WHO_AM_I);
}

void drv_icm20689_init(void)
{
    /* Reset device */
    icm_write_reg(REG_PWR_MGMT_1, 0x80);
    for(volatile uint32_t i = 0; i < 200000; i++);

    /* Auto select clock */
    icm_write_reg(REG_PWR_MGMT_1, 0x01);
    icm_write_reg(REG_PWR_MGMT_2, 0x00);

    /* Sample rate = 1kHz */
    icm_write_reg(REG_SMPLRT_DIV, 0x00);

    /* DLPF = 92Hz */
    icm_write_reg(REG_CONFIG, 0x02);
    icm_write_reg(REG_ACCEL_CONFIG2, 0x02);

    /* Gyro ±2000dps */
    icm_write_reg(REG_GYRO_CONFIG, 0x18);

    /* Accel ±16g */
    icm_write_reg(REG_ACCEL_CONFIG, 0x18);

    /* Check device ID */
    uint8_t id = drv_icm20689_who_am_i();
    if(id != WHO_AM_I_VALUE)
    {
        while(1);   // halt if not detected
    }
}

void drv_icm20689_read_raw(drv_icm20689_raw_t *raw)
{
    uint8_t buf[14];

    icm_read_buf(REG_ACCEL_XOUT_H, buf, 14);

    raw->acc[0]  = (int16_t)((buf[0]  << 8) | buf[1]);
    raw->acc[1]  = (int16_t)((buf[2]  << 8) | buf[3]);
    raw->acc[2]  = (int16_t)((buf[4]  << 8) | buf[5]);

    raw->gyro[0] = (int16_t)((buf[8]  << 8) | buf[9]);
    raw->gyro[1] = (int16_t)((buf[10] << 8) | buf[11]);
    raw->gyro[2] = (int16_t)((buf[12] << 8) | buf[13]);
}
