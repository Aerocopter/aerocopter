#include "drv_ist8310.h"
#include "bsp_iic.h"
#include "delay.h"

/* ================= 常量定义 ================= */

#define IST8310_ADDR        0x0C
#define IST8310_WHO_AM_I    0x00
#define IST8310_EXPECT_ID   0x10
#define IST8310_DATA_START  0x03

/* ================= 全局对象 ================= */

ist8310_t ist8310;

/* ================= 底层读写 ================= */

static void ist_write(uint8_t reg, uint8_t data)
{
    bsp_i2c3_write(IST8310_ADDR, reg, data);
}

static uint8_t ist_read(uint8_t reg)
{
    return bsp_i2c3_read(IST8310_ADDR, reg);
}

static void ist_read_buf(uint8_t reg, uint8_t *buf, uint8_t len)
{
    bsp_i2c3_read_buffer(IST8310_ADDR, reg, buf, len);
}

/* ================= 设备检测 ================= */

uint8_t drv_ist8310_detect(void)
{
    uint8_t id = ist_read(IST8310_WHO_AM_I);
    return (id == IST8310_EXPECT_ID);
}

/* ================= 初始化 ================= */

uint8_t drv_ist8310_init(void)
{
    if (!drv_ist8310_detect())
        return 0;

    /* 软复位 */
    ist_write(0x0B, 0x01);
    
    delay_ms(100);
    /* 设置单次测量模式 */
    ist_write(0x0A, 0x01);

    ist8310.healthy = 1;
    return 1;
}

/* ================= 读取原始数据 ================= */

void drv_ist8310_read_raw(int16_t *mag)
{
    uint8_t buf[6];

    /* 触发测量 */
    ist_write(0x0A, 0x01);

    /* 简单延时（必要） */
    for(volatile int i = 0; i < 2000; i++);

    ist_read_buf(IST8310_DATA_START, buf, 6);

    mag[0] = (int16_t)(buf[1] << 8 | buf[0]);
    mag[1] = (int16_t)(buf[3] << 8 | buf[2]);
    mag[2] = (int16_t)(buf[5] << 8 | buf[4]);
}

/* ================= 更新接口（对齐 IMU 驱动） ================= */

void drv_ist8310_update(void)
{
    if (!ist8310.healthy)
        return;

    drv_ist8310_read_raw(ist8310.mag_raw);

    /* 如需校准/坐标变换，可在这里处理 */
    ist8310.mag[0] = ist8310.mag_raw[0];
    ist8310.mag[1] = ist8310.mag_raw[1];
    ist8310.mag[2] = ist8310.mag_raw[2];
}
