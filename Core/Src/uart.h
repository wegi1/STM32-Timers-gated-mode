/*
 * uart.h
 *
 *  Created on: 10 maj 2022
 *      Author: BOLO
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "main.h"



/**
 *
 * @brief UART1 GPIO Configuration
 *
 * RXD  PA9     USART1_TX
 * TXT  PA10    USART1_RX
 *
 */
void uart_UART1_GPIO_config(void);

/**
 * @brief UART1 Peripherial Configuration
 */
void uart_UART1_config(void);

/**
 * @brief UART1 Transmit
 */
bool uart_UART1_transmit(uint8_t *data, uint8_t len, uint32_t timeout);


#endif /* INC_UART_H_ */
