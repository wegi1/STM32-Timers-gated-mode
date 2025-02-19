/*
 * rcc.c
 *
 *  Created on: May 10, 2022
 *
 */

#include "rcc.h"

//ms Tick
__IO uint32_t msTicks = 0;




/**
 * @brief HSE Configuration
 */
void rcc_HSE_config(void)
{
    /*
     * Configuration parameters --> STMF103 Clock Tree
     *
     * HSE = 8MHz (High Speed External Clock)
     * PLL_M = 9  (9*8 = 72[MHz])
     * USB prescaler = 1.5 (for 48[MHz])
     * AHB prescaler = 1
     * Cortex prescaler = 1
     * --> 72[MHz] System Clock
     *
     * APB1 prescaler = 2 --> (36[MHz] from 72[MHz])
     * APB2 prescaler = 1 -->  72[MHz]
     * ADC  prescaler = 6 -->  12[MHz] for ADC conversion
     *
     */

    /* PLL Configuration */
    // PLL = 9
    //RCC->CFGR &= ~ //Clear bitfields [21:18] 11111111111000011111111111111111
    /* USB prescaler = 1.5 (for 48[MHz]) */
    RCC->CFGR &= ~(RCC_CFGR_PLLMULL_Msk | RCC_CFGR_USBPRE);
    RCC->CFGR |= (7UL << 18);
    /* HSE Oscillator */
    // Enable HSE Oscillator
    RCC->CR |= RCC_CR_HSEON;
    // Wait for it to stabilize
    while((RCC->CR & RCC_CR_HSERDY) == 0);


    //Select HSE as PLL source
    RCC->CFGR |= RCC_CFGR_PLLSRC;
    //Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    //Wait for PLL ready
    while((RCC->CR & RCC_CR_PLLRDY) == 0);


    //Flash pre-fetch enable and wait-state=2
    //0WS: 0-24MHz
    //1WS: 24-48MHz
    //2WS: 48-72MHz
    FLASH->ACR = 0;
    FLASH->ACR |= (FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_1);
    //Select PLL as main System Clock source
    RCC->CFGR &= (RCC_CFGR_SW);
    RCC->CFGR |= RCC_CFGR_SW_1;
    //Wait for active clock source
    while((RCC->CFGR & RCC_CFGR_SWS_1) == 0);

    /* Peripherials */
    //AHD Prescaler = 1
    //APB1 prescaler
    //APB2 prescaler
    /* ADC Prescaler to 12[MHz] */
    RCC->CFGR &= ~(RCC_CFGR_HPRE |  RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2 | RCC_CFGR_ADCPRE);
    RCC->CFGR |= (RCC_CFGR_PPRE1_2 | RCC_CFGR_ADCPRE_1);
}

/**
 * @brief SysTick configuration
 */
void rcc_SysTick_config(uint32_t arr)
{
    /*
     * Reset Control register
     * Set the reload value
     * Enable SysTick interrupt (NVIC)
     * Reset SysTick value to 0
     * Enable SysTick from Cntrl register
     */

     /* Reset Control register */
    SysTick->CTRL = 0;

     /* Set the reload value */
    SysTick->LOAD = arr - 1;

     /* Enable SysTick interrupt (NVIC) */
    NVIC_SetPriority(SysTick_IRQn, 0);

     /* Reset SysTick value to 0 */
    SysTick->VAL = 0;

     /* Enable SysTick from Cntrl register */
    SysTick->CTRL |= (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);
}



/**
 * @brief Get ms Ticks
 */
uint32_t rcc_msGetTicks(void)
{
    return msTicks;
}

/**
 * @brief ms Delay
 */
void rcc_msDelay(uint32_t ms)
{
    uint32_t startTicks = rcc_msGetTicks();
    while((rcc_msGetTicks() - startTicks ) < ms);
}



