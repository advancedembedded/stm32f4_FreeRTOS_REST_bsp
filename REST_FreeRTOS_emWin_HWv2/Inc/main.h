/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define TP1_Pin GPIO_PIN_2
#define TP1_GPIO_Port GPIOE
#define TP2_Pin GPIO_PIN_3
#define TP2_GPIO_Port GPIOE
#define TS_INT_Pin GPIO_PIN_4
#define TS_INT_GPIO_Port GPIOE
#define I2CBB_SDA_Pin GPIO_PIN_5
#define I2CBB_SDA_GPIO_Port GPIOE
#define I2CBB_SCL_Pin GPIO_PIN_6
#define I2CBB_SCL_GPIO_Port GPIOE
#define HUMID_INT_Pin GPIO_PIN_13
#define HUMID_INT_GPIO_Port GPIOC
#define HVAC_CH3_Pin GPIO_PIN_6
#define HVAC_CH3_GPIO_Port GPIOF
#define HVAC_CH2_Pin GPIO_PIN_7
#define HVAC_CH2_GPIO_Port GPIOF
#define HVAC_CH1_Pin GPIO_PIN_8
#define HVAC_CH1_GPIO_Port GPIOF
#define HVAC_CH0_Pin GPIO_PIN_9
#define HVAC_CH0_GPIO_Port GPIOF
#define WIFI_GPIO2_Pin GPIO_PIN_1
#define WIFI_GPIO2_GPIO_Port GPIOC
#define LCD_ENABLE_Pin GPIO_PIN_2
#define LCD_ENABLE_GPIO_Port GPIOA
#define WIFI_GPIO0_Pin GPIO_PIN_5
#define WIFI_GPIO0_GPIO_Port GPIOA
#define LCD_BRT_TIM3_CH2_Pin GPIO_PIN_7
#define LCD_BRT_TIM3_CH2_GPIO_Port GPIOA
#define WIFI_GPIO16_Pin GPIO_PIN_4
#define WIFI_GPIO16_GPIO_Port GPIOC
#define WIFI_RESET_Pin GPIO_PIN_5
#define WIFI_RESET_GPIO_Port GPIOC
#define PBOOT1_TP5_Pin GPIO_PIN_2
#define PBOOT1_TP5_GPIO_Port GPIOB
#define SPI2_RESET_Pin GPIO_PIN_11
#define SPI2_RESET_GPIO_Port GPIOD
#define HVAC_CH7_PWM_Pin GPIO_PIN_12
#define HVAC_CH7_PWM_GPIO_Port GPIOD
#define SPI2_WP_Pin GPIO_PIN_13
#define SPI2_WP_GPIO_Port GPIOD
#define WIFI_ENABLE_Pin GPIO_PIN_3
#define WIFI_ENABLE_GPIO_Port GPIOG
#define SD_CARD_DETECT_Pin GPIO_PIN_8
#define SD_CARD_DETECT_GPIO_Port GPIOA
#define HVAC_CH4_Pin GPIO_PIN_7
#define HVAC_CH4_GPIO_Port GPIOD
#define HVAC_CH5_Pin GPIO_PIN_9
#define HVAC_CH5_GPIO_Port GPIOG
#define HVAC_CH6_Pin GPIO_PIN_13
#define HVAC_CH6_GPIO_Port GPIOG
#define TP4_Pin GPIO_PIN_14
#define TP4_GPIO_Port GPIOG
#define TP3_Pin GPIO_PIN_5
#define TP3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
