/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

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
#define I_SINK_Pin GPIO_PIN_0
#define I_SINK_GPIO_Port GPIOC
#define STATUS_1_Pin GPIO_PIN_1
#define STATUS_1_GPIO_Port GPIOC
#define STATUS_2_Pin GPIO_PIN_2
#define STATUS_2_GPIO_Port GPIOC
#define STATUS_3_Pin GPIO_PIN_3
#define STATUS_3_GPIO_Port GPIOC
#define ANLG_MUX_0_Pin GPIO_PIN_0
#define ANLG_MUX_0_GPIO_Port GPIOA
#define ANLG_MUX_1_Pin GPIO_PIN_1
#define ANLG_MUX_1_GPIO_Port GPIOA
#define ANLG_MUX_2_Pin GPIO_PIN_2
#define ANLG_MUX_2_GPIO_Port GPIOA
#define FAN_EN_Pin GPIO_PIN_3
#define FAN_EN_GPIO_Port GPIOA
#define ENC_A_Pin GPIO_PIN_0
#define ENC_A_GPIO_Port GPIOB
#define ENC_A_EXTI_IRQn EXTI0_IRQn
#define ENC_B_Pin GPIO_PIN_1
#define ENC_B_GPIO_Port GPIOB
#define ENC_B_EXTI_IRQn EXTI1_IRQn
#define ADC_CS_Pin GPIO_PIN_2
#define ADC_CS_GPIO_Port GPIOB
#define MUX_G2A_Pin GPIO_PIN_10
#define MUX_G2A_GPIO_Port GPIOB
#define MUX_G2B_Pin GPIO_PIN_11
#define MUX_G2B_GPIO_Port GPIOB
#define MUX_G1_Pin GPIO_PIN_12
#define MUX_G1_GPIO_Port GPIOB
#define MUX_A_Pin GPIO_PIN_13
#define MUX_A_GPIO_Port GPIOB
#define MUX_B_Pin GPIO_PIN_14
#define MUX_B_GPIO_Port GPIOB
#define MUX_C_Pin GPIO_PIN_15
#define MUX_C_GPIO_Port GPIOB
#define LCD_D4_Pin GPIO_PIN_6
#define LCD_D4_GPIO_Port GPIOC
#define LCD_D5_Pin GPIO_PIN_7
#define LCD_D5_GPIO_Port GPIOC
#define LCD_D6_Pin GPIO_PIN_8
#define LCD_D6_GPIO_Port GPIOC
#define LCD_D7_Pin GPIO_PIN_9
#define LCD_D7_GPIO_Port GPIOC
#define LCD_RS_Pin GPIO_PIN_10
#define LCD_RS_GPIO_Port GPIOC
#define LCD_RW_Pin GPIO_PIN_11
#define LCD_RW_GPIO_Port GPIOC
#define LCD_EN_Pin GPIO_PIN_12
#define LCD_EN_GPIO_Port GPIOC
#define BTN_A_Pin GPIO_PIN_4
#define BTN_A_GPIO_Port GPIOB
#define BTN_A_EXTI_IRQn EXTI4_IRQn
#define BTN_B_Pin GPIO_PIN_5
#define BTN_B_GPIO_Port GPIOB
#define BTN_B_EXTI_IRQn EXTI9_5_IRQn
#define BTN_C_Pin GPIO_PIN_6
#define BTN_C_GPIO_Port GPIOB
#define BTN_C_EXTI_IRQn EXTI9_5_IRQn
#define TEMP_CS_Pin GPIO_PIN_8
#define TEMP_CS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
