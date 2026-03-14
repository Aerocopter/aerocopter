#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "stm32f4xx.h"
#include <stdint.h>

/*
 * Initialize USART2 on:
 *   TX -> PA2
 *   RX -> PA3
 *
 * @param baudrate: desired baud rate (e.g., 115200)
 */
void uart_init(uint32_t baudrate);

/*
 * Send a single character (blocking)
 */
void uart_send_char(char c);

/*
 * Send a null-terminated string (blocking)
 */
void uart_send_string(const char *str);
void uart_send_uint32(uint32_t value);

/*
 * Return 1 if RX buffer has data, 0 otherwise
 */
int uart_available(void);

/*
 * Read one character from RX buffer
 * Return:
 *   >=0 : valid character
 *   -1  : no data available
 */
int uart_read_char(void);

int _write(int file, char *ptr, int len);

#endif
