/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define INIT_DUTY_SPT_L 1200
#define INIT_DUTY_SPT_R 1200
#define DUTY_SPT_RANGE 600

#define INIT_DUTY_SP2_L 3000
#define INIT_DUTY_SP2_R 3000
#define DUTY_SP2_RANGE 700

#define GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS 16.4


#define __PID_SPEED_T(cfg, error, correction,dir, newDutyL, newDutyR) ({ \
	correction = (cfg).Kp * error + (cfg).Ki * (cfg).ekSum + (cfg).Kd * ((cfg).ek1 - error);\
	(cfg).ek1 = error; \
	(cfg).ekSum += error; \
	correction = correction > DUTY_SPT_RANGE ? DUTY_SPT_RANGE : (correction < -DUTY_SPT_RANGE ? -DUTY_SPT_RANGE : correction); \
	newDutyL = INIT_DUTY_SPT_L + correction; \
	newDutyR = INIT_DUTY_SPT_R - correction; \
})

#define __PID_SPEED_2(cfg, error, correction, dir, newDutyL, newDutyR) ({ \
	correction = (cfg).Kp * error + (cfg).Ki * (cfg).ekSum + (cfg).Kd * ((cfg).ek1 - error);\
	(cfg).ek1 = error; \
	(cfg).ekSum += error; \
	correction = correction > DUTY_SP2_RANGE ? DUTY_SP2_RANGE : (correction < -DUTY_SP2_RANGE ? -DUTY_SP2_RANGE : correction); \
	newDutyL = INIT_DUTY_SP2_L + correction*dir; \
	newDutyR = INIT_DUTY_SP2_R - correction*dir; \
})
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OLED_SCL_Pin GPIO_PIN_5
#define OLED_SCL_GPIO_Port GPIOE
#define OLED_SDA_Pin GPIO_PIN_6
#define OLED_SDA_GPIO_Port GPIOE
#define AIN2_Pin GPIO_PIN_2
#define AIN2_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_3
#define AIN1_GPIO_Port GPIOA
#define BIN1_Pin GPIO_PIN_4
#define BIN1_GPIO_Port GPIOA
#define BIN2_Pin GPIO_PIN_5
#define BIN2_GPIO_Port GPIOA
#define Right_Encoder2_Pin GPIO_PIN_6
#define Right_Encoder2_GPIO_Port GPIOA
#define Right_Encoder1_Pin GPIO_PIN_7
#define Right_Encoder1_GPIO_Port GPIOA
#define OLED_RST_Pin GPIO_PIN_7
#define OLED_RST_GPIO_Port GPIOE
#define OLED_DE_Pin GPIO_PIN_8
#define OLED_DE_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_10
#define LED3_GPIO_Port GPIOE
#define SERVO_Pin GPIO_PIN_14
#define SERVO_GPIO_Port GPIOE
#define ECHO_Pin GPIO_PIN_12
#define ECHO_GPIO_Port GPIOD
#define TRIG_Pin GPIO_PIN_13
#define TRIG_GPIO_Port GPIOD
#define PWMA_Pin GPIO_PIN_6
#define PWMA_GPIO_Port GPIOC
#define PWMB_Pin GPIO_PIN_7
#define PWMB_GPIO_Port GPIOC
#define Left_Encoder2_Pin GPIO_PIN_15
#define Left_Encoder2_GPIO_Port GPIOA
#define Left_Encoder1_Pin GPIO_PIN_3
#define Left_Encoder1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
