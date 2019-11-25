/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define BTN_MODE_Pin GPIO_PIN_0
#define BTN_MODE_GPIO_Port GPIOA
#define BTN_SET_Pin GPIO_PIN_1
#define BTN_SET_GPIO_Port GPIOA
#define TFT_RESET_PIN_Pin GPIO_PIN_2
#define TFT_RESET_PIN_GPIO_Port GPIOA
#define TFT_DC_PIN_Pin GPIO_PIN_3
#define TFT_DC_PIN_GPIO_Port GPIOA
#define LCD_CS_Pin GPIO_PIN_4
#define LCD_CS_GPIO_Port GPIOA
#define OIL_TEMP_Pin GPIO_PIN_0
#define OIL_TEMP_GPIO_Port GPIOB
#define NEUTRAL_Pin GPIO_PIN_2
#define NEUTRAL_GPIO_Port GPIOB
#define BLINK_Pin GPIO_PIN_10
#define BLINK_GPIO_Port GPIOB
#define HIBEAM_Pin GPIO_PIN_11
#define HIBEAM_GPIO_Port GPIOB
#define EEPROM_CS_Pin GPIO_PIN_12
#define EEPROM_CS_GPIO_Port GPIOB
#define RPMPin_Pin GPIO_PIN_8
#define RPMPin_GPIO_Port GPIOA
#define VPIN_Pin GPIO_PIN_9
#define VPIN_GPIO_Port GPIOA
#define VPIN_EXTI_IRQn EXTI9_5_IRQn
#define STEP1_Pin GPIO_PIN_3
#define STEP1_GPIO_Port GPIOB
#define STEP2_Pin GPIO_PIN_4
#define STEP2_GPIO_Port GPIOB
#define STEP3_Pin GPIO_PIN_5
#define STEP3_GPIO_Port GPIOB
#define STEP4_Pin GPIO_PIN_6
#define STEP4_GPIO_Port GPIOB
#define TFTBCKL_Pin GPIO_PIN_8
#define TFTBCKL_GPIO_Port GPIOB
#define LEDBCKL_Pin GPIO_PIN_9
#define LEDBCKL_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
