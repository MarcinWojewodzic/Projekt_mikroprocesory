/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define dc_oled_Pin GPIO_PIN_1
#define dc_oled_GPIO_Port GPIOA
#define cs_oled_Pin GPIO_PIN_4
#define cs_oled_GPIO_Port GPIOA
#define RC522_CS_Pin GPIO_PIN_4
#define RC522_CS_GPIO_Port GPIOC
#define rfid_rst_Pin GPIO_PIN_5
#define rfid_rst_GPIO_Port GPIOC
#define rst_oled_Pin GPIO_PIN_7
#define rst_oled_GPIO_Port GPIOE
#define analizator_Pin GPIO_PIN_8
#define analizator_GPIO_Port GPIOE
#define out0_Pin GPIO_PIN_0
#define out0_GPIO_Port GPIOD
#define out1_Pin GPIO_PIN_1
#define out1_GPIO_Port GPIOD
#define out2_Pin GPIO_PIN_2
#define out2_GPIO_Port GPIOD
#define out3_Pin GPIO_PIN_3
#define out3_GPIO_Port GPIOD
#define in0_Pin GPIO_PIN_4
#define in0_GPIO_Port GPIOD
#define in2_Pin GPIO_PIN_6
#define in2_GPIO_Port GPIOD
#define in3_Pin GPIO_PIN_7
#define in3_GPIO_Port GPIOD
#define in1_Pin GPIO_PIN_4
#define in1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
