/**
 ******************************************************************************
 * @file    HTS221_Driver_HL.h
 * @author  MEMS Application Team
 * @version V3.0.0
 * @date    12-August-2016
 * @brief   This file contains definitions for the HTS221_Driver_HL.c firmware driver
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HTS221_DRIVER_HL_H
#define __HTS221_DRIVER_HL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
/* Include sensor component drivers. */
#include "HTS221_Driver.h"
	
#ifndef NULL
#define NULL ( void * )0
#endif

typedef struct
{
  int16_t AXIS_X;
  int16_t AXIS_Y;
  int16_t AXIS_Z;
} SensorAxesRaw_t;

/**
 * @brief  Sensor axes data structure definition
 */
typedef struct
{
  int32_t AXIS_X;
  int32_t AXIS_Y;
  int32_t AXIS_Z;
} SensorAxes_t;

/**
 * @brief  Sensor output data rate enumerator definition
 */
typedef enum
{
  ODR_LOW,
  ODR_MID_LOW,
  ODR_MID,
  ODR_MID_HIGH,
  ODR_HIGH
} SensorOdr_t;

/**
 * @brief  Sensor full scale enumerator definition
 */
typedef enum
{
  FS_LOW,
  FS_MID_LOW,
  FS_MID,
  FS_MID_HIGH,
  FS_HIGH
} SensorFs_t;

/**
 * @brief  Sensor interrupt pin enumerator definition
 */
typedef enum
{
  INT1_PIN,
  INT2_PIN
} SensorIntPin_t;

typedef struct
{

  /* Identity */
  uint8_t who_am_i;

  /* Configuration */
  uint8_t address;       /* Sensor I2C address (NOTE: Not a unique sensor ID). */
  uint8_t instance;      /* Sensor instance (NOTE: Sensor ID unique only within its class). */
  uint8_t isInitialized; /* Sensor setup done. */
  uint8_t isEnabled;     /* Sensor ON. */
  uint8_t isCombo;       /* Combo sensor (component consists of more sensors). */

  /* Pointer to the Data */
  void *pData;

  /* Pointer to the Virtual Table */
  void *pVTable;
  /* Pointer to the Extended Virtual Table */
  void *pExtVTable;
} DrvContextTypeDef;



/**
 * @brief  Component's Status enumerator definition.
 */
typedef enum
{
  COMPONENT_OK = 0,
  COMPONENT_ERROR,
  COMPONENT_TIMEOUT,
  COMPONENT_NOT_IMPLEMENTED
} DrvStatusTypeDef;

typedef struct
{
  DrvStatusTypeDef ( *Init            ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *DeInit          ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Sensor_Enable   ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Sensor_Disable  ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Get_WhoAmI      ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *Check_WhoAmI    ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Get_Hum         ) ( DrvContextTypeDef*, float* );
  DrvStatusTypeDef ( *Get_ODR         ) ( DrvContextTypeDef*, float* );
  DrvStatusTypeDef ( *Set_ODR         ) ( DrvContextTypeDef*, SensorOdr_t );
  DrvStatusTypeDef ( *Set_ODR_Value   ) ( DrvContextTypeDef*, float );
  DrvStatusTypeDef ( *Read_Reg        ) ( DrvContextTypeDef*, uint8_t, uint8_t* );
  DrvStatusTypeDef ( *Write_Reg       ) ( DrvContextTypeDef*, uint8_t, uint8_t );
  DrvStatusTypeDef ( *Get_DRDY_Status ) ( DrvContextTypeDef*, uint8_t* );
} HUMIDITY_Drv_t;

/**
 * @brief  HUMIDITY data structure definition
 */
typedef struct
{
  void *pComponentData; /* Component specific data. */
  void *pExtData;       /* Other data. */
} HUMIDITY_Data_t;

typedef struct
{
  DrvStatusTypeDef ( *Init            ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *DeInit          ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Sensor_Enable   ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Sensor_Disable  ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Get_WhoAmI      ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *Check_WhoAmI    ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Get_Temp        ) ( DrvContextTypeDef*, float* );
  DrvStatusTypeDef ( *Get_ODR         ) ( DrvContextTypeDef*, float* );
  DrvStatusTypeDef ( *Set_ODR         ) ( DrvContextTypeDef*, SensorOdr_t );
  DrvStatusTypeDef ( *Set_ODR_Value   ) ( DrvContextTypeDef*, float );
  DrvStatusTypeDef ( *Read_Reg        ) ( DrvContextTypeDef*, uint8_t, uint8_t* );
  DrvStatusTypeDef ( *Write_Reg       ) ( DrvContextTypeDef*, uint8_t, uint8_t );
  DrvStatusTypeDef ( *Get_DRDY_Status ) ( DrvContextTypeDef*, uint8_t* );
} TEMPERATURE_Drv_t;

/**
 * @brief  TEMPERATURE data structure definition
 */
typedef struct
{
  void *pComponentData; /* Component specific data. */
  void *pExtData;       /* Other data. */
} TEMPERATURE_Data_t;


#define HTS221_SENSORS_MAX_NUM  1     /**< HTS221 max number of instances */
#define HTS221_ADDRESS_DEFAULT  0xBE  /**< HTS221 I2C Address */

typedef struct
{
  uint8_t isHumInitialized;
  uint8_t isTempInitialized;
  uint8_t isHumEnabled;
  uint8_t isTempEnabled;
} HTS221_Combo_Data_t;

/**
 * @brief HTS221 humidity specific data internal structure definition
 */

typedef struct
{
  HTS221_Combo_Data_t *comboData;       /* Combo data to manage in software enable/disable of the combo sensors */
} HTS221_H_Data_t;


/**
 * @brief HTS221 temperature specific data internal structure definition
 */

typedef struct
{
  HTS221_Combo_Data_t *comboData;       /* Combo data to manage in software enable/disable of the combo sensors */
} HTS221_T_Data_t;

/**
 * @}
 */

/** @addtogroup HTS221_Public_Variables Public variables
 * @{
 */

extern HUMIDITY_Drv_t HTS221_H_Drv;
extern TEMPERATURE_Drv_t HTS221_T_Drv;
extern HTS221_Combo_Data_t HTS221_Combo_Data[HTS221_SENSORS_MAX_NUM];

#ifdef __cplusplus
}
#endif

#endif /* __HTS221_DRIVER_HL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
