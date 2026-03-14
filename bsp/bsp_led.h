#ifndef __BSP_LED_H
#define __BSP_LED_H

#include "stm32f4xx.h"

// 引脚定义（根据你的原理图修改）
#define LED_RED_PIN     GPIO_Pin_13   // PC13
#define LED_GREEN_PIN   GPIO_Pin_0    // PC0
#define LED_BLUE_PIN    GPIO_Pin_1    // PC1
#define LED_PORT        GPIOC

// 初始化所有 LED（配置 GPIO 为推挽输出，初始状态灭）
void BSP_LED_Init(void);

// 单独控制每个 LED 的亮灭（1 = 亮，0 = 灭）
void BSP_LED_Red(uint8_t on);
void BSP_LED_Green(uint8_t on);
void BSP_LED_Blue(uint8_t on);

// 同时设置三个 LED 的状态
void BSP_LED_SetAll(uint8_t red_on, uint8_t green_on, uint8_t blue_on);

// 翻转单个 LED（可用于调试）
void BSP_LED_ToggleRed(void);
void BSP_LED_ToggleGreen(void);
void BSP_LED_ToggleBlue(void);

#endif
