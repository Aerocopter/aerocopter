#include "drv_ms5611.h"
#include <stdbool.h>
#include "stdio.h"

// 外部I2C函数（由bsp提供）
extern void bsp_i2c3_init(void);
extern void bsp_i2c3_write(uint8_t addr, uint8_t reg, uint8_t data);
extern uint8_t bsp_i2c3_read(uint8_t addr, uint8_t reg);
extern void bsp_i2c3_read_buffer(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);

// ==================== 常量定义 ====================
#define MS5611_ADDR             0x77   // 默认地址（CSB接地），若接高电平则为0x76

// 命令
#define MS5611_CMD_RESET        0x1E
#define MS5611_CMD_READ_PROM    0xA0   // PROM读取基地址（每个字占2字节）
#define MS5611_CMD_CONV_D1      0x40   // 压力转换基值
#define MS5611_CMD_CONV_D2      0x50   // 温度转换基值
#define MS5611_CMD_ADC_READ     0x00   // 读取ADC结果

// 过采样率（使用最高精度4096，可根据需要调整）
#define MS5611_OSR              0x08   // OSR=4096

// 转换时间（微秒）对应OSR_4096
#define CONV_TIME_US            9040

// 全局变量
ms5611_t ms5611 = {0};

// ==================== 内部函数 ====================

/**
 * @brief 简单毫秒延时（需根据实际平台替换）
 */
static void delay_ms(uint32_t ms)
{
    // 示例：空循环延时，实际使用时请替换为精确延时
    for (volatile uint32_t i = 0; i < ms * 8000; i++);
}

/**
 * @brief 向传感器写入命令（无数据）
 */
static void write_cmd(uint8_t cmd)
{
    bsp_i2c3_write(MS5611_ADDR, cmd, 0);
}

/**
 * @brief 读取PROM中的一个字（16位）
 */
static uint16_t prom_read(uint8_t index)
{
    uint8_t buf[2];
    uint8_t reg = MS5611_CMD_READ_PROM + (index * 2);
    bsp_i2c3_read_buffer(MS5611_ADDR, reg, buf, 2);
    return (uint16_t)((buf[0] << 8) | buf[1]);
}

/**
 * @brief 读取ADC结果（24位）
 */
static uint32_t adc_read(void)
{
    uint8_t buf[3];
    // 发送读命令后直接读取（有些I2C实现需要重新发起读操作，此处按标准流程）
    // 为了兼容提供的接口，这里使用read_buffer，注意reg参数为命令字节
    bsp_i2c3_read_buffer(MS5611_ADDR, MS5611_CMD_ADC_READ, buf, 3);
    return (uint32_t)((buf[0] << 16) | (buf[1] << 8) | buf[2]);
}

/**
 * @brief 等待转换完成
 */
static void wait_conv(void)
{
    // 转换为毫秒并增加余量
    delay_ms(CONV_TIME_US / 1000 + 10);
}

/**
 * @brief 启动温度转换
 */
static void start_temp_conv(void)
{
    write_cmd(MS5611_CMD_CONV_D2 | MS5611_OSR);
}

/**
 * @brief 启动压力转换
 */
static void start_press_conv(void)
{
    write_cmd(MS5611_CMD_CONV_D1 | MS5611_OSR);
}

/**
 * @brief 计算CRC并验证PROM数据
 * @param prom 包含8个字的数组
 * @return true=校验通过
 */
static bool crc_check(uint16_t prom[8])
{
    uint8_t crc_read = prom[7] & 0x0F;
    uint16_t crc = 0;
    prom[7] &= 0xFF00;   // 清除原有CRC位

    for (uint8_t i = 0; i < 16; i++) {
        if (i % 2 == 1) {
            crc ^= prom[i >> 1] & 0x00FF;
        } else {
            crc ^= prom[i >> 1] >> 8;
        }
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x3000;
            } else {
                crc <<= 1;
            }
        }
    }
    uint8_t crc_calc = (crc >> 12) & 0x0F;
    return (crc_read == crc_calc);
}

/**
 * @brief 补偿计算（根据数据手册算法）
 */
static void compensate(void)
{
    int32_t dT = ms5611.temp_raw - ((int32_t)ms5611.c5 << 8);
    int32_t TEMP = 2000 + ((int64_t)dT * (int64_t)ms5611.c6 >> 23);
    int64_t OFF = ((int64_t)ms5611.c2 << 16) + ((int64_t)ms5611.c4 * (int64_t)dT >> 7);
    int64_t SENS = ((int64_t)ms5611.c1 << 15) + ((int64_t)ms5611.c3 * (int64_t)dT >> 8);

    // 温度低于20℃时的二阶补偿
    if (TEMP < 2000) {
        int32_t T2 = ((int64_t)dT * (int64_t)dT) >> 31;
        int32_t tmp = (TEMP - 2000) * (TEMP - 2000);
        int64_t OFF2 = (5 * (int64_t)tmp) >> 1;
        int64_t SENS2 = OFF2 >> 1;

        if (TEMP < -1500) {
            tmp = (TEMP + 1500) * (TEMP + 1500);
            OFF2 += 7 * (int64_t)tmp;
            SENS2 += (11 * (int64_t)tmp) >> 1;
        }

        TEMP -= T2;
        OFF -= OFF2;
        SENS -= SENS2;
    }

    int64_t P = ((int64_t)ms5611.press_raw * SENS >> 21) - OFF;
    ms5611.pressure = (float)(P >> 15) / 100.0f;
    ms5611.temperature = (float)TEMP / 100.0f;
}

// ==================== 外部接口 ====================

uint8_t drv_ms5611_detect(void)
{
    uint16_t prom[8];

    // 尝试读取PROM的前几个字（例如索引0和1），判断I2C通信是否正常
    // 更严谨的做法是读取全部8个字并验证CRC
    for (int i = 0; i < 8; i++) {
        prom[i] = prom_read(i);
    }

    return crc_check(prom) ? 1 : 0;
}

uint8_t drv_ms5611_init(void)
{
    // 1. I2C初始化（如果外部未初始化可调用，但通常由bsp统一管理）
    // bsp_i2c3_init();

    // 2. 复位传感器
    write_cmd(MS5611_CMD_RESET);
    delay_ms(10);   // 复位等待

    // 3. 读取校准系数
    uint16_t prom[8];
    for (int i = 0; i < 8; i++) {
        prom[i] = prom_read(i);
    }

    // 4. CRC校验
    if (!crc_check(prom)) {
        ms5611.healthy = 0;
        return 0;   // 初始化失败
    }

    // 5. 保存系数
    ms5611.c1 = prom[1];
    ms5611.c2 = prom[2];
    ms5611.c3 = prom[3];
    ms5611.c4 = prom[4];
    ms5611.c5 = prom[5];
    ms5611.c6 = prom[6];

    ms5611.healthy = 1;
    return 1;
}

void drv_ms5611_read_raw(int32_t *press_raw, int32_t *temp_raw)
{
    // 启动温度转换
    start_temp_conv();
    wait_conv();
    ms5611.temp_raw = (int32_t)adc_read();

    // 启动压力转换
    start_press_conv();
    wait_conv();
    ms5611.press_raw = (int32_t)adc_read();
   
    //printf("raw temp = %ld, raw press = %ld\n", ms5611.temp_raw, ms5611.press_raw);

    if (press_raw) *press_raw = ms5611.press_raw;
    if (temp_raw)  *temp_raw = ms5611.temp_raw;
}

void drv_ms5611_update(void)
{
    if (!ms5611.healthy) return;

    // 读取原始数据
    drv_ms5611_read_raw(&ms5611.press_raw, &ms5611.temp_raw);

    // 补偿计算
    compensate();
}
