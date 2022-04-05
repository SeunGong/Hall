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
#include "stm32f7xx_hal.h"

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
#define ADC3_IN14_IR_L_Pin GPIO_PIN_4
#define ADC3_IN14_IR_L_GPIO_Port GPIOF
#define ADC3_IN15_IR_R_Pin GPIO_PIN_5
#define ADC3_IN15_IR_R_GPIO_Port GPIOF
#define ADC3_IN4_Joystick_X_Pin GPIO_PIN_6
#define ADC3_IN4_Joystick_X_GPIO_Port GPIOF
#define ADC3_IN5_Joystick_Y_Pin GPIO_PIN_7
#define ADC3_IN5_Joystick_Y_GPIO_Port GPIOF
#define GPIO_Output_ToF_GPIO0_Pin GPIO_PIN_10
#define GPIO_Output_ToF_GPIO0_GPIO_Port GPIOF
#define DAC_OUT1_ThrottleR_Pin GPIO_PIN_4
#define DAC_OUT1_ThrottleR_GPIO_Port GPIOA
#define DAC_OUT2_ThrottleL_Pin GPIO_PIN_5
#define DAC_OUT2_ThrottleL_GPIO_Port GPIOA
#define GPIO_Output_BreakR_Pin GPIO_PIN_6
#define GPIO_Output_BreakR_GPIO_Port GPIOA
#define GPIO_Output_BreakL_Pin GPIO_PIN_7
#define GPIO_Output_BreakL_GPIO_Port GPIOA
#define GPIO_Output_ToF_GPIO1_Pin GPIO_PIN_11
#define GPIO_Output_ToF_GPIO1_GPIO_Port GPIOF
#define GPIO_Output_ToF_GPIO3_Pin GPIO_PIN_13
#define GPIO_Output_ToF_GPIO3_GPIO_Port GPIOF
#define GPIO_Output_ToF_GPIO4_Pin GPIO_PIN_14
#define GPIO_Output_ToF_GPIO4_GPIO_Port GPIOF
#define GPIO_Output_ToF_GPIO5_Pin GPIO_PIN_15
#define GPIO_Output_ToF_GPIO5_GPIO_Port GPIOF
#define GPIO_Output_ToF_XSHUT0_Pin GPIO_PIN_0
#define GPIO_Output_ToF_XSHUT0_GPIO_Port GPIOG
#define GPIO_Output_ToF_XSHUT1_Pin GPIO_PIN_1
#define GPIO_Output_ToF_XSHUT1_GPIO_Port GPIOG
#define GPIO_In_HallL_Pin GPIO_PIN_10
#define GPIO_In_HallL_GPIO_Port GPIOE
#define GPIO_In_HallLE11_Pin GPIO_PIN_11
#define GPIO_In_HallLE11_GPIO_Port GPIOE
#define GPIO_In_HallLE12_Pin GPIO_PIN_12
#define GPIO_In_HallLE12_GPIO_Port GPIOE
#define GPIO_In_HallR_Pin GPIO_PIN_13
#define GPIO_In_HallR_GPIO_Port GPIOE
#define GPIO_In_HallRE14_Pin GPIO_PIN_14
#define GPIO_In_HallRE14_GPIO_Port GPIOE
#define GPIO_In_HallRE15_Pin GPIO_PIN_15
#define GPIO_In_HallRE15_GPIO_Port GPIOE
#define GPIO_In_BLE_GPI_Pin GPIO_PIN_10
#define GPIO_In_BLE_GPI_GPIO_Port GPIOD
#define GPIO_Output_BLE_GPO_Pin GPIO_PIN_11
#define GPIO_Output_BLE_GPO_GPIO_Port GPIOD
#define TIM4_CH3_PWV_Buzzer_Pin GPIO_PIN_14
#define TIM4_CH3_PWV_Buzzer_GPIO_Port GPIOD
#define TIM4_CH4_PWM_LED_PLS_Pin GPIO_PIN_15
#define TIM4_CH4_PWM_LED_PLS_GPIO_Port GPIOD
#define GPIO_Output_ToF_XSHUT2_Pin GPIO_PIN_2
#define GPIO_Output_ToF_XSHUT2_GPIO_Port GPIOG
#define GPIO_Output_ToF_XSHUT3_Pin GPIO_PIN_3
#define GPIO_Output_ToF_XSHUT3_GPIO_Port GPIOG
#define GPIO_Output_ToF_XSHUT4_Pin GPIO_PIN_4
#define GPIO_Output_ToF_XSHUT4_GPIO_Port GPIOG
#define GPIO_Output_ToF_XSHUT5_Pin GPIO_PIN_5
#define GPIO_Output_ToF_XSHUT5_GPIO_Port GPIOG
#define GPIO_EXTI6_JoystickSW_Pin GPIO_PIN_6
#define GPIO_EXTI6_JoystickSW_GPIO_Port GPIOG
#define GPIO_EXTI6_JoystickSW_EXTI_IRQn EXTI9_5_IRQn
#define GPIO_Output_BoardingSW_Pin GPIO_PIN_7
#define GPIO_Output_BoardingSW_GPIO_Port GPIOG
#define GPIO_Output_ReverseL_Pin GPIO_PIN_4
#define GPIO_Output_ReverseL_GPIO_Port GPIOD
#define GPIO_Output_ReverseR_Pin GPIO_PIN_5
#define GPIO_Output_ReverseR_GPIO_Port GPIOD
#define GPIO_Output_LED_Blue_Pin GPIO_PIN_7
#define GPIO_Output_LED_Blue_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
