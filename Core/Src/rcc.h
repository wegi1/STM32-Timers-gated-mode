/*
 * rcc.h
 *
 *  Created on: May 10, 2022
 *      Author: BOLO
 */

#ifndef INC_RCC_H_
#define INC_RCC_H_

#include "main.h"

/**
 * @brief HSE Configuration
 */
void rcc_HSE_config(void);

/**
 * @brief SysTick configuration
 */
void rcc_SysTick_config(uint32_t arr);



/**
 * @brief Get ms Ticks
 */
uint32_t rcc_msGetTicks(void);

/**
 * @brief ms Delay
 */
void rcc_msDelay(uint32_t ms);





#endif /* INC_RCC_H_ */
