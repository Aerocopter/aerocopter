#include "bsp_led.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

// 初始化所有 LED 引脚
void BSP_LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // 使能 GPIOC 时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    // 配置三个引脚为推挽输出
    GPIO_InitStructure.GPIO_Pin = LED_RED_PIN | LED_GREEN_PIN | LED_BLUE_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(LED_PORT, &GPIO_InitStructure);

    // 初始状态：全部熄灭（假设高电平点亮，则熄灭时输出低电平）
    // 如果实际电路是低电平点亮，请将下面两行的 SetBits/ResetBits 互换
    GPIO_ResetBits(LED_PORT, LED_RED_PIN | LED_GREEN_PIN | LED_BLUE_PIN);
}

// 红色 LED 控制
void BSP_LED_Red(uint8_t on)
{
    if (on)
        GPIO_SetBits(LED_PORT, LED_RED_PIN);   // 高电平点亮
    else
        GPIO_ResetBits(LED_PORT, LED_RED_PIN); // 低电平熄灭
}

// 绿色 LED 控制
void BSP_LED_Green(uint8_t on)
{
    if (on)
        GPIO_SetBits(LED_PORT, LED_GREEN_PIN);
    else
        GPIO_ResetBits(LED_PORT, LED_GREEN_PIN);
}

// 蓝色 LED 控制
void BSP_LED_Blue(uint8_t on)
{
    if (on)
        GPIO_SetBits(LED_PORT, LED_BLUE_PIN);
    else
        GPIO_ResetBits(LED_PORT, LED_BLUE_PIN);
}

// 同时设置三个 LED（更高效，一次操作）
void BSP_LED_SetAll(uint8_t red_on, uint8_t green_on, uint8_t blue_on)
{
    uint16_t pins_on = 0;
    uint16_t pins_off = 0;

    if (red_on)   pins_on  |= LED_RED_PIN;
    else          pins_off |= LED_RED_PIN;

    if (green_on) pins_on  |= LED_GREEN_PIN;
    else          pins_off |= LED_GREEN_PIN;

    if (blue_on)  pins_on  |= LED_BLUE_PIN;
    else          pins_off |= LED_BLUE_PIN;

    // 先熄灭需要灭的，再点亮需要亮的（顺序无严格要求）
    GPIO_ResetBits(LED_PORT, pins_off);
    GPIO_SetBits(LED_PORT, pins_on);
}

// 翻转红色 LED
void BSP_LED_ToggleRed(void)
{
    GPIO_ToggleBits(LED_PORT, LED_RED_PIN);
}

// 翻转绿色 LED
void BSP_LED_ToggleGreen(void)
{
    GPIO_ToggleBits(LED_PORT, LED_GREEN_PIN);
}

// 翻转蓝色 LED
void BSP_LED_ToggleBlue(void)
{
    GPIO_ToggleBits(LED_PORT, LED_BLUE_PIN);
}
