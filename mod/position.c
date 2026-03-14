#include "position.h"
#include "drv_ms5611.h"
#include "delay.h"
#include <math.h>

// 外部传感器全局变量（由驱动提供）
extern ms5611_t ms5611;

// 静态校准参数
static float pressure_ref = 1013.25f;   // 参考气压（对应高度0），默认海平面

/**
 * @brief 气压转高度（国际标准大气压公式）
 * @param pressure 气压值（mbar）
 * @return 高度（米）
 */
static float pressure_to_altitude(float pressure)
{
    // h = 44330 * (1 - (p/p0)^(1/5.255))
    return 44330.0f * (1.0f - powf(pressure / pressure_ref, 0.190295f));
}

// ==================== 外部接口 ====================

void position_init(void)
{
    // 初始化传感器
    drv_ms5611_init();

    // 初始参考气压取默认海平面值，可在校准时更新
    pressure_ref = 1013.25f;
}

void position_update(position_t *data)
{
    if (!data) return;

    // 更新传感器数据
    drv_ms5611_update();      // 更新气压/温度

    // 获取气压和温度
    float pressure = ms5611.pressure;
    float temperature = ms5611.temperature;

    // 计算高度
    float altitude = pressure_to_altitude(pressure);

    // 填充数据
    data->altitude = altitude;
    data->pressure = pressure;
    data->temperature = temperature;
    data->healthy = ms5611.healthy ? 1 : 0;
}

/**
 * @brief 校准：将当前平均气压设为高度零点（只校准气压）
 */
void position_calibrate(void)
{
    const int samples = 100;
    float pressure_sum = 0;

    for (int i = 0; i < samples; i++) {
        drv_ms5611_update();
        pressure_sum += ms5611.pressure;
        delay_ms(10);
    }

    // 更新参考气压（当前平均气压对应高度0）
    pressure_ref = pressure_sum / samples;
}

void position_offset_get(position_t *offset)
{
    if (!offset) return;

    // 返回参考气压值（存储在 altitude 字段中）
    offset->altitude = pressure_ref;
    offset->pressure = 0;
    offset->temperature = 0;
    offset->healthy = 1;
}

// 新增：直接获取当前高度（米）
float position_get_altitude(void)
{
    position_t data;
    position_update(&data);
    return data.altitude;
}
