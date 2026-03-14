#ifndef __DRV_MS5611_H
#define __DRV_MS5611_H

#include <stdint.h>

// MS5611设备结构体
typedef struct
{
    // 原始ADC值
    int32_t press_raw;     // 压力原始值（24位无符号）
    int32_t temp_raw;      // 温度原始值（24位有符号）
    
    // 补偿后的物理量
    float pressure;        // 气压（mbar/hPa）
    float temperature;     // 温度（摄氏度）
    
    // 校准系数（从PROM读取）
    uint16_t c1, c2, c3, c4, c5, c6;
    
    uint8_t healthy;       // 健康状态：0=异常，1=正常
} ms5611_t;

// 外部全局变量
extern ms5611_t ms5611;

// 函数接口
uint8_t drv_ms5611_detect(void);   // 检测传感器是否正常（通过PROM校验）
uint8_t drv_ms5611_init(void);     // 初始化传感器（复位、读取校准系数）
void drv_ms5611_read_raw(int32_t *press_raw, int32_t *temp_raw);  // 读取原始ADC值（阻塞）
void drv_ms5611_update(void);      // 执行一次完整测量并更新补偿值

#endif
