/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
enum { SRAM_BB_REGION_START = 0x20000000 };
enum { SRAM_BB_REGION_END = 0x200fffff };
enum { SRAM_BB_ALIAS = 0x22000000 };
enum { PERIPH_BB_REGION_START = 0x40000000 };
enum { PERIPH_BB_REGION_END = 0x400fffff };
enum { PERIPH_BB_ALIAS = 0x42000000 };
#define SRAM_ADR_COND(adres) ( (uint32_t)&adres >= SRAM_BB_REGION_START && (uint32_t)&adres <= SRAM_BB_REGION_END )
#define PERIPH_ADR_COND(adres) ( (uint32_t)&adres >= PERIPH_BB_REGION_START && (uint32_t)&adres <= PERIPH_BB_REGION_END )
#define BB_SRAM2(adres, bit)( SRAM_BB_ALIAS + ((uint32_t)&adres - SRAM_BB_REGION_START)*32u + (uint32_t)(bit*4u) )
#define BB_PERIPH(adres, bit)( PERIPH_BB_ALIAS + ((uint32_t)&adres - PERIPH_BB_REGION_START)*32u + (uint32_t)(__builtin_ctz(bit))*4u)
/* bit - bit mask, not bit position! */
#define BB(adres, bit) *(__IO uint32_t *)( SRAM_ADR_COND(adres) ? BB_SRAM2(adres, bit) : \
( PERIPH_ADR_COND(adres) ? BB_PERIPH(adres, bit) : 0 ))
#define BB_SRAM(adres, bit) *(__IO uint32_t *)BB_SRAM2(adres, bit)
//*********************************************************************
typedef enum {
    /* Push-Pull */
    gpio_mode_output_PP_2MHz = 2,
    gpio_mode_output_PP_10MHz = 1,
    gpio_mode_output_PP_50MHz = 3,
    /* Open-Drain */
    gpio_mode_output_OD_2MHz = 6,
    gpio_mode_output_OD_10MHz = 5,
    gpio_mode_output_OD_50MHz = 7,
    /* Push-Pull */
    gpio_mode_alternate_PP_2MHz = 10,
    gpio_mode_alternate_PP_10MHz = 9,
    gpio_mode_alternate_PP_50MHz = 11,
    /* Open-Drain */
    gpio_mode_alternate_OD_2MHz = 14,
    gpio_mode_alternate_OD_10MHz = 13,
    gpio_mode_alternate_OD_50MHz = 15,
    /* Analog input (ADC) */
    gpio_mode_input_analog = 0,
    /* Floating digital input. */
    gpio_mode_input_floating = 4,
    /* Digital input with pull-up/down (depending on the ODR reg.). */
    gpio_mode_input_pull = 8
} GpioMode_t;

typedef enum {
    PA0 = 0x00000001,
    PA1 = 0x00000002,
    PA2 = 0x00000004,
    PA3 = 0x00000008,
    PA4 = 0x00000010,
    PA5 = 0x00000020,
    PA6 = 0x00000040,
    PA7 = 0x00000080,
    PA8 = 0x00000100,
    PA9 = 0x00000200,
    PA10 = 0x00000400,
    PA11 = 0x00000800,
    PA12 = 0x00001000,
    PA13 = 0x00002000,
    PA14 = 0x00004000,
    PA15 = 0x00008000,

    PB0 = 0x00000001,
    PB1 = 0x00000002,
    PB2 = 0x00000004,
    PB3 = 0x00000008,
    PB4 = 0x00000010,
    PB5 = 0x00000020,
    PB6 = 0x00000040,
    PB7 = 0x00000080,
    PB8 = 0x00000100,
    PB9 = 0x00000200,
    PB10 = 0x00000400,
    PB11 = 0x00000800,
    PB12 = 0x00001000,
    PB13 = 0x00002000,
    PB14 = 0x00004000,
    PB15 = 0x00008000,

    PC0 = 0x00000001,
    PC1 = 0x00000002,
    PC2 = 0x00000004,
    PC3 = 0x00000008,
    PC4 = 0x00000010,
    PC5 = 0x00000020,
    PC6 = 0x00000040,
    PC7 = 0x00000080,
    PC8 = 0x00000100,
    PC9 = 0x00000200,
    PC10 = 0x00000400,
    PC11 = 0x00000800,
    PC12 = 0x00001000,
    PC13 = 0x00002000,
    PC14 = 0x00004000,
    PC15 = 0x00008000,

    PD0 = 0x00000001,
    PD1 = 0x00000002,
    PD2 = 0x00000004,
    PD3 = 0x00000008,
    PD4 = 0x00000010,
    PD5 = 0x00000020,
    PD6 = 0x00000040,
    PD7 = 0x00000080,
    PD8 = 0x00000100,
    PD9 = 0x00000200,
    PD10 = 0x00000400,
    PD11 = 0x00000800,
    PD12 = 0x00001000,
    PD13 = 0x00002000,
    PD14 = 0x00004000,
    PD15 = 0x00008000,

    PE0 = 0x00000001,
    PE1 = 0x00000002,
    PE2 = 0x00000004,
    PE3 = 0x00000008,
    PE4 = 0x00000010,
    PE5 = 0x00000020,
    PE6 = 0x00000040,
    PE7 = 0x00000080,
    PE8 = 0x00000100,
    PE9 = 0x00000200,
    PE10 = 0x00000400,
    PE11 = 0x00000800,
    PE12 = 0x00001000,
    PE13 = 0x00002000,
    PE14 = 0x00004000,
    PE15 = 0x00008000,

    PF0 = 0x00000001,
    PF1 = 0x00000002,
    PF2 = 0x00000004,
    PF3 = 0x00000008,
    PF4 = 0x00000010,
    PF5 = 0x00000020,
    PF6 = 0x00000040,
    PF7 = 0x00000080,
    PF8 = 0x00000100,
    PF9 = 0x00000200,
    PF10 = 0x00000400,
    PF11 = 0x00000800,
    PF12 = 0x00001000,
    PF13 = 0x00002000,
    PF14 = 0x00004000,
    PF15 = 0x00008000,

    PG0 = 0x00000001,
    PG1 = 0x00000002,
    PG2 = 0x00000004,
    PG3 = 0x00000008,
    PG4 = 0x00000010,
    PG5 = 0x00000020,
    PG6 = 0x00000040,
    PG7 = 0x00000080,
    PG8 = 0x00000100,
    PG9 = 0x00000200,
    PG10 = 0x00000400,
    PG11 = 0x00000800,
    PG12 = 0x00001000,
    PG13 = 0x00002000,
    PG14 = 0x00004000,
    PG15 = 0x00008000

}GpioPin_t;
//*********************************************************************


void gpio_pin_cfg(GPIO_TypeDef * const port, GpioPin_t pin, GpioMode_t mode);

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
