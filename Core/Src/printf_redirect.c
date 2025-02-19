/*
 * printf_redirect.c
 *
 *  Created on: 10 maj 2022
 *      Author: BOLO
 */

#include "uart.h"

int __io_putchar(int ch);
int _write(int file,char *ptr, int len);

int __io_putchar(int ch) {
 uart_UART1_transmit((uint8_t *)&ch, 1, 100);
 return ch;
}

int _write(int file,char *ptr, int len) {
 int DataIdx;
 for(DataIdx= 0; DataIdx< len; DataIdx++) {
  __io_putchar(*ptr++);
 }
 return len;
}
