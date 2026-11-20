/*!
* @brief Component name:	UART
*
* Defines public interface to UART application level driver.
*
* @file UART.h
*/
#ifndef UART_H
#define UART_H

#include <stdbool.h>
#include <stdint.h>

bool boUART_Init	(void);
bool boUART_getc	(uint8_t *u8ch);
void uart_printf	(const char *fmt, ...);

#endif
