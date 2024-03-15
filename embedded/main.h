/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fonts.h"
#include "z_displ_ILI9163.h"
#include "z_touch_XPT2046.h"
#include "eeprom_emul.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* EEPROM ADDRESS MACROS */
#define LOWER_THRESH 1
#define UPPER_THRESH 3
#define GRAD_UPPER 5 // The most significant 16 bits of the 32 bit float
#define GRAD_LOWER 7 // The least significant 16 bits of the 32 bit float
#define TARE_ADC 9
#define TARE_MASS 13

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void read_line_wifi(char *line);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DISPL_LED_Pin GPIO_PIN_0
#define DISPL_LED_GPIO_Port GPIOA
#define DISPL_SCK_Pin GPIO_PIN_1
#define DISPL_SCK_GPIO_Port GPIOA
#define DISPL_CS_Pin GPIO_PIN_4
#define DISPL_CS_GPIO_Port GPIOA
#define TOUCH_MISO_Pin GPIO_PIN_6
#define TOUCH_MISO_GPIO_Port GPIOA
#define DISPL_MOSI_Pin GPIO_PIN_7
#define DISPL_MOSI_GPIO_Port GPIOA
#define TOUCH_CS_Pin GPIO_PIN_0
#define TOUCH_CS_GPIO_Port GPIOB
#define PWR_LED_Pin GPIO_PIN_14
#define PWR_LED_GPIO_Port GPIOB
#define TOUCH_INT_Pin GPIO_PIN_8
#define TOUCH_INT_GPIO_Port GPIOA
#define TOUCH_INT_EXTI_IRQn EXTI9_5_IRQn
#define DISPL_DC_Pin GPIO_PIN_11
#define DISPL_DC_GPIO_Port GPIOA
#define DISPL_RST_Pin GPIO_PIN_12
#define DISPL_RST_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
