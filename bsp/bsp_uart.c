#include "stdio.h"
#include "unistd.h"
#include "bsp_uart.h"

/* ======================== Configuration ======================== */

#define UART_RX_BUFFER_SIZE   128

/* ======================== Private Variables ======================== */

/* Ring buffer for RX */
static volatile char     rx_buffer[UART_RX_BUFFER_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

/* ======================== Initialization ======================== */

void uart_init(uint32_t baudrate)
{
    uint32_t pclk1;

    /* 1. Enable GPIOA clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    /* 2. Enable USART2 clock (APB1) */
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* 3. Configure PA2 (TX) and PA3 (RX) as Alternate Function */
    GPIOA->MODER &= ~((3U << (2 * 2)) | (3U << (3 * 2)));
    GPIOA->MODER |=  (2U << (2 * 2)) | (2U << (3 * 2));  /* AF mode */

    /* 4. Select AF7 (USART2) */
    GPIOA->AFR[0] &= ~((0xFU << (2 * 4)) | (0xFU << (3 * 4)));
    GPIOA->AFR[0] |=  (7U << (2 * 4)) | (7U << (3 * 4));

    /* 5. Configure baud rate
    STM32F405 default APB1 clock = 42 MHz (if system = 168 MHz) */
    if ((RCC->CFGR & RCC_CFGR_PPRE1) == RCC_CFGR_PPRE1_DIV1)
        pclk1 = SystemCoreClock;
    else
        pclk1 = SystemCoreClock / 4;  // 当前配置

    USART2->BRR = (pclk1 + baudrate/2U) / baudrate;

    /* 6. Enable transmitter and receiver */
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;

    /* 7. Enable RX interrupt */
    USART2->CR1 |= USART_CR1_RXNEIE;

    /* 8. Enable USART */
    USART2->CR1 |= USART_CR1_UE;

    /* 9. Configure NVIC */
    NVIC_SetPriority(USART2_IRQn, 5);
    NVIC_EnableIRQ(USART2_IRQn);
}

/* ======================== TX Functions ======================== */

void uart_send_char(char c)
{
    /* Wait until transmit data register is empty */
    while (!(USART2->SR & USART_SR_TXE));

    USART2->DR = (uint8_t)c;
}

void uart_send_string(const char *str)
{
    while (*str)
    {
        uart_send_char(*str++);
    }
}

/* Convert uint32 value to decimal string and send via UART (no stdlib used) */
void uart_send_uint32(uint32_t value)
{
    char buffer[10];          /* Maximum 10 digits for uint32_t */
    uint32_t i = 0;

    /* Special case: value is zero */
    if (value == 0)
    {
        uart_send_char('0');
        return;
    }

    /* Extract digits in reverse order */
    while (value > 0)
    {
        buffer[i++] = (value % 10U) + '0';
        value /= 10U;
    }

    /* Transmit digits in correct order */
    while (i > 0)
    {
        uart_send_char(buffer[--i]);
    }
}

/* ======================== RX Functions ======================== */

int uart_available(void)
{
    return (rx_head != rx_tail);
}

int uart_read_char(void)
{
    if (rx_head == rx_tail)
    {
        return -1;  /* No data available */
    }

    char c = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) % UART_RX_BUFFER_SIZE;

    return (int)c;
}

/* ======================== Interrupt Handler ======================== */

void USART2_IRQHandler(void)
{
    if (USART2->SR & USART_SR_RXNE)
    {
        char c = (char)USART2->DR;

        uint16_t next = (rx_head + 1) % UART_RX_BUFFER_SIZE;

        /* Prevent buffer overflow */
        if (next != rx_tail)
        {
            rx_buffer[rx_head] = c;
            rx_head = next;
        }
    }
}
