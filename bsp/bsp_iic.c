#include "stm32f4xx.h"
#include "bsp_iic.h"
#include "CoOS.h"

OS_MutexID i2c_mutex;

/* =========================================================
   I2C3
   SCL = PA8
   SDA = PC9
   ========================================================= */

void bsp_i2c3_init(void)
{
    /* Enable clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;

    /* -------- PA8 -> SCL -------- */
    GPIOA->MODER &= ~(3 << (8 * 2));
    GPIOA->MODER |=  (2 << (8 * 2));      // AF mode
    GPIOA->OTYPER |= (1 << 8);            // Open-drain
    GPIOA->OSPEEDR |= (3 << (8 * 2));     // High speed
    GPIOA->PUPDR |= (1 << (8 * 2));       // Pull-up
    GPIOA->AFR[1] &= ~(0xF << 0);
    GPIOA->AFR[1] |=  (4 << 0);           // AF4 I2C

    /* -------- PC9 -> SDA -------- */
    GPIOC->MODER &= ~(3 << (9 * 2));
    GPIOC->MODER |=  (2 << (9 * 2));
    GPIOC->OTYPER |= (1 << 9);
    GPIOC->OSPEEDR |= (3 << (9 * 2));
    GPIOC->PUPDR |= (1 << (9 * 2));
    GPIOC->AFR[1] &= ~(0xF << 4);
    GPIOC->AFR[1] |=  (4 << 4);

    /* Reset I2C */
    I2C3->CR1 = I2C_CR1_SWRST;
    I2C3->CR1 = 0;

    /* 100kHz (APB1 = 42MHz) */
    I2C3->CR2   = 42;
    I2C3->CCR   = 210;
    I2C3->TRISE = 43;

    i2c_mutex = CoCreateMutex();   // 创建互斥量
    /* Enable I2C */
    I2C3->CR1 |= I2C_CR1_PE;
}

/* ================= 内部函数 ================= */

static void i2c3_start(void)
{
    I2C3->CR1 |= I2C_CR1_START;
    while(!(I2C3->SR1 & I2C_SR1_SB));
}

static void i2c3_stop(void)
{
    I2C3->CR1 |= I2C_CR1_STOP;
}

static void i2c3_send_addr(uint8_t addr, uint8_t dir)
{
    I2C3->DR = (addr << 1) | dir;
    while(!(I2C3->SR1 & I2C_SR1_ADDR));
    (void)I2C3->SR2;
}

static void i2c3_write_byte(uint8_t data)
{
    while(!(I2C3->SR1 & I2C_SR1_TXE));
    I2C3->DR = data;
}

static uint8_t i2c3_read_byte(uint8_t ack)
{
    if(ack) I2C3->CR1 |= I2C_CR1_ACK;
    else    I2C3->CR1 &= ~I2C_CR1_ACK;

    while(!(I2C3->SR1 & I2C_SR1_RXNE));
    return I2C3->DR;
}

/* ================= Public API ================= */

void bsp_i2c3_write(uint8_t addr, uint8_t reg, uint8_t data)
{
    CoEnterMutexSection(i2c_mutex);   // 加锁
    i2c3_start();
    i2c3_send_addr(addr, 0);
    i2c3_write_byte(reg);
    i2c3_write_byte(data);
    i2c3_stop();
    CoLeaveMutexSection(i2c_mutex);   // 解锁
}

uint8_t bsp_i2c3_read(uint8_t addr, uint8_t reg)
{
    uint8_t data;
    CoEnterMutexSection(i2c_mutex);   // 加锁
    
    i2c3_start();
    i2c3_send_addr(addr, 0);
    i2c3_write_byte(reg);

    i2c3_start();
    i2c3_send_addr(addr, 1);
    data = i2c3_read_byte(0);

    i2c3_stop();
    CoLeaveMutexSection(i2c_mutex);   // 解锁
    return data;
}

void bsp_i2c3_read_buffer(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    CoEnterMutexSection(i2c_mutex);   // 加锁
    i2c3_start();
    i2c3_send_addr(addr, 0);
    i2c3_write_byte(reg);

    i2c3_start();
    i2c3_send_addr(addr, 1);

    for(uint16_t i = 0; i < len; i++)
    {
        buf[i] = i2c3_read_byte(i < (len - 1));
    }

    i2c3_stop();
    CoLeaveMutexSection(i2c_mutex);   // 解锁
}
