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
#include "stm32g4xx_hal.h"

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
#define E_stop_Pin GPIO_PIN_13
#define E_stop_GPIO_Port GPIOC
#define E_stop_EXTI_IRQn EXTI15_10_IRQn
#define RCC_OSC32_IN_Pin GPIO_PIN_14
#define RCC_OSC32_IN_GPIO_Port GPIOC
#define RCC_OSC32_OUT_Pin GPIO_PIN_15
#define RCC_OSC32_OUT_GPIO_Port GPIOC
#define RCC_OSC_IN_Pin GPIO_PIN_0
#define RCC_OSC_IN_GPIO_Port GPIOF
#define RCC_OSC_OUT_Pin GPIO_PIN_1
#define RCC_OSC_OUT_GPIO_Port GPIOF
#define Motor_Dir_Pin GPIO_PIN_0
#define Motor_Dir_GPIO_Port GPIOC
#define Encoder_Pin GPIO_PIN_0
#define Encoder_GPIO_Port GPIOA
#define EncoderA1_Pin GPIO_PIN_1
#define EncoderA1_GPIO_Port GPIOA
#define Homing_signal_Pin GPIO_PIN_4
#define Homing_signal_GPIO_Port GPIOA
#define gripper_u_d_Pin GPIO_PIN_4
#define gripper_u_d_GPIO_Port GPIOC
#define Manual_mode_Pin GPIO_PIN_0
#define Manual_mode_GPIO_Port GPIOB
#define Auto_Mode_Pin GPIO_PIN_1
#define Auto_Mode_GPIO_Port GPIOB
#define reed_up_Pin GPIO_PIN_7
#define reed_up_GPIO_Port GPIOC
#define PWM_Pin GPIO_PIN_8
#define PWM_GPIO_Port GPIOA
#define reed_down_Pin GPIO_PIN_9
#define reed_down_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define gripper_o_c_Pin GPIO_PIN_10
#define gripper_o_c_GPIO_Port GPIOC
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define reed_open_Pin GPIO_PIN_4
#define reed_open_GPIO_Port GPIOB
#define reed_close_Pin GPIO_PIN_9
#define reed_close_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* ── Joystick Shield (Funduino) ───────────────────────────────────────────── */
#define JOY_BTN_A_Pin        GPIO_PIN_5
#define JOY_BTN_A_GPIO_Port  GPIOA   /* PA5 */
#define JOY_BTN_B_Pin        GPIO_PIN_6
#define JOY_BTN_B_GPIO_Port  GPIOA   /* PA6 */
#define JOY_BTN_C_Pin        GPIO_PIN_7
#define JOY_BTN_C_GPIO_Port  GPIOA   /* PA7 */
#define JOY_BTN_D_Pin        GPIO_PIN_11
#define JOY_BTN_D_GPIO_Port  GPIOB   /* PB11 */
#define JOY_BTN_K_Pin        GPIO_PIN_10
#define JOY_BTN_K_GPIO_Port  GPIOB   /* PB10 */
#define JOY_ADC_Pin          GPIO_PIN_2
#define JOY_ADC_GPIO_Port    GPIOC   /* PC2 → ADC2_IN8 */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
