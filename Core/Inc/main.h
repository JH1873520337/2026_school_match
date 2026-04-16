/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define TB6612_1_PWMA_Pin GPIO_PIN_5
#define TB6612_1_PWMA_GPIO_Port GPIOE
#define TB6612_1_PWMB_Pin GPIO_PIN_6
#define TB6612_1_PWMB_GPIO_Port GPIOE
#define Motor4_A_Encoder_Pin GPIO_PIN_0
#define Motor4_A_Encoder_GPIO_Port GPIOA
#define Motor4_B_Encoder_Pin GPIO_PIN_1
#define Motor4_B_Encoder_GPIO_Port GPIOA
#define ICM42688P_CS_Pin GPIO_PIN_4
#define ICM42688P_CS_GPIO_Port GPIOA
#define ICM42688P_SCK_Pin GPIO_PIN_5
#define ICM42688P_SCK_GPIO_Port GPIOA
#define ICM42688P_MISO_Pin GPIO_PIN_6
#define ICM42688P_MISO_GPIO_Port GPIOA
#define ICM42688P_MOSI_Pin GPIO_PIN_7
#define ICM42688P_MOSI_GPIO_Port GPIOA
#define ICM42688P_INT1_Pin GPIO_PIN_4
#define ICM42688P_INT1_GPIO_Port GPIOC
#define ICM42688P_INT1_EXTI_IRQn EXTI4_IRQn
#define BUZZ_control_Pin GPIO_PIN_2
#define BUZZ_control_GPIO_Port GPIOB
#define Motor1_A_Encoder_Pin GPIO_PIN_9
#define Motor1_A_Encoder_GPIO_Port GPIOE
#define Motor1_B_Encoder_Pin GPIO_PIN_11
#define Motor1_B_Encoder_GPIO_Port GPIOE
#define TB6612_2_PWMB_Pin GPIO_PIN_14
#define TB6612_2_PWMB_GPIO_Port GPIOB
#define TB6612_2_PWMA_Pin GPIO_PIN_15
#define TB6612_2_PWMA_GPIO_Port GPIOB
#define Motor3_B_Encoder_Pin GPIO_PIN_12
#define Motor3_B_Encoder_GPIO_Port GPIOD
#define Motor3_A_Encoder_Pin GPIO_PIN_13
#define Motor3_A_Encoder_GPIO_Port GPIOD
#define TB6612_2_BIN2_Pin GPIO_PIN_6
#define TB6612_2_BIN2_GPIO_Port GPIOC
#define TB6612_2_BIN1_Pin GPIO_PIN_7
#define TB6612_2_BIN1_GPIO_Port GPIOC
#define TB6612_2_AIN2_Pin GPIO_PIN_8
#define TB6612_2_AIN2_GPIO_Port GPIOC
#define TB6612_2_AIN1_Pin GPIO_PIN_9
#define TB6612_2_AIN1_GPIO_Port GPIOC
#define TB6612_1_BIN2_Pin GPIO_PIN_8
#define TB6612_1_BIN2_GPIO_Port GPIOA
#define TB6612_1_BIN1_Pin GPIO_PIN_9
#define TB6612_1_BIN1_GPIO_Port GPIOA
#define TB6612_1_AIN2_Pin GPIO_PIN_10
#define TB6612_1_AIN2_GPIO_Port GPIOA
#define TB6612_1_AIN1_Pin GPIO_PIN_11
#define TB6612_1_AIN1_GPIO_Port GPIOA
#define Motor2_A_Encoder_Pin GPIO_PIN_15
#define Motor2_A_Encoder_GPIO_Port GPIOA
#define camera_tx_Pin GPIO_PIN_10
#define camera_tx_GPIO_Port GPIOC
#define camera_rx_Pin GPIO_PIN_11
#define camera_rx_GPIO_Port GPIOC
#define screen_tx_Pin GPIO_PIN_12
#define screen_tx_GPIO_Port GPIOC
#define screen_rx_Pin GPIO_PIN_2
#define screen_rx_GPIO_Port GPIOD
#define Motor2_B_Encoder_Pin GPIO_PIN_3
#define Motor2_B_Encoder_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
