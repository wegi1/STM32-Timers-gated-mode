/*
 * uart.c
 *
 *  Created on: 10 maj 2022
 *      Author: BOLO
 */

#include "uart.h"

extern uint32_t rcc_msGetTicks(void);

/**
 *
 * @brief UART1 GPIO Configuration
 *
 * RXD  PA9     USART1_TX
 * TXT  PA10    USART1_RX
 *
 */
void uart_UART1_GPIO_config(void)
{
    /*
    * PA9-Tx, Pa10-Rx
    * Enable PortA Clock
    * Mode to AF (UART1)
    * Output max 10MHZ
    * Map PA9, PA10 mapped to UART1
    */

    // PA9-Tx, Pa10-Rx
    // Enable PortA Clock
    RCC->APB2ENR |= (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN);

    // Mode to AF (UART1)
    // Output max 10MHZ
    GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_CNF10 | GPIO_CRH_MODE9 | GPIO_CRH_MODE10);
    GPIOA->CRH |= (GPIO_CRH_CNF9_1 | GPIO_CRH_CNF10_0 | GPIO_CRH_MODE9_0);

    // Map PA9, PA10 mapped to UART1
    AFIO->MAPR &= ~(AFIO_MAPR_USART1_REMAP);



}

/**
 * @brief UART1 Peripherial Configuration
 */
void uart_UART1_config(void)
{
    /*
     * Enable UART1 Clock
     * Enable Transmit
     * Parity to Even
     * Parity control enabled
     * Word length to 8 bits
     * Stop bits to 1
     * Disable HW Flow Control
     * Set Baud rate to 115200
     * Clear some flags and enable
     */
//============================================================

    /* Enable UART1 Clock */
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    /* Enable Transmit/Recive */
    //USART1->CR1 |= (USART_CR1_TE |  USART_CR1_RE); // NOT USED RECIVE BYTES HERE
    USART1->CR1 |= (USART_CR1_TE);

    /* Parity to Even */
    /* Parity control enabled */
    /* Word length to 8 bits */
    USART1->CR1 &= ~(USART_CR1_PS | USART_CR1_PCE | USART_CR1_M);

    /* Stop bits to 1 */
    /* Clear some flags and enable */
    //Clear LINEN and CLKEN in CR2
    USART1->CR2 &= ~(USART_CR2_STOP | USART_CR2_LINEN | USART_CR2_CLKEN);

    /* Disable HW Flow Control */
    //Clear SCEN, HDSEL and IREN in CR3
    USART1->CR3 &= ~(USART_CR3_CTSE | USART_CR3_RTSE | USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN);

    /* Set Baud rate to 115200 */
    //Set Baud rate to 115200 (72MHz = 39.0625 -> 39 / 1)
    //BRR = 72MHz/115200/16 = 39.0625
    //Mantissa = 39
    //Fraction = .0625*16 = 1
    USART1->BRR = (39UL << 4) | 1;

    /* Enable UART1 */
    USART1->CR1 |= USART_CR1_UE;
}

/**
 * @brief UART1 Transmit
 */
bool uart_UART1_transmit(uint8_t *data, uint8_t len, uint32_t timeout)
{
    //Wait on TXE to start transmit
    //Write to DR as TXE flag is HIGH (Tx buffer Empty)
    uint8_t dataIdx = 0;
    uint32_t startTick = rcc_msGetTicks();
    while(dataIdx<len)
    {
      if(USART1->SR & USART_SR_TXE) //Tx buffer empty
      {
        USART1->DR = data[dataIdx];
        dataIdx++;
      }
      else //Manage timeout
      {
        if((rcc_msGetTicks() - startTick)>= timeout) return false;
      }
    }
    //Wait for busy flag
    while(USART1->SR & USART_SR_TC)
    {
      if((rcc_msGetTicks() - startTick)>= timeout) return false;
    }
    return true;
}



