/**
  ******************************************************************************
  * @file    stm324x9i_REST_HWv2_rtc.c
  * @author  Martin
  * @version V1
  * @date    
  * @brief   This file provides the kernel rtc functions 
  ******************************************************************************
  * @attention
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


#include "stm32f4xx_hal.h"

extern void Error_Handler(void);

RTC_HandleTypeDef hrtc;

/**
  * @brief  RTC init
  * @param  None
  * @retval None
  */
	
void BSP_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initialize RTC and set the Time and Date 
    */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief  RTC Get time. 
  * @param  Time: Pointer to Time structure
  * @retval None
  */
void BSP_RTC_GetTime(  RTC_TimeTypeDef *Time)
{
   HAL_RTC_GetTime(&hrtc, Time, RTC_FORMAT_BIN);
}

/**
  * @brief  RTC Set time. 
  * @param  Time: Pointer to Time structure
  * @retval None
  */
void BSP_RTC_SetTime(  RTC_TimeTypeDef *Time)
{
   Time->StoreOperation = 0;
   Time->SubSeconds = 0;
   Time->DayLightSaving = 0;
   HAL_RTC_SetTime(&hrtc, Time, RTC_FORMAT_BIN);
}

/**
  * @brief  RTC Get date
  * @param  Date: Pointer to Date structure
  * @retval None
  */
void BSP_RTC_GetDate(  RTC_DateTypeDef *Date)
{
   HAL_RTC_GetDate(&hrtc, Date, RTC_FORMAT_BIN);
   
   if((Date->Date == 0) || (Date->Month == 0))
   {
     Date->Date = Date->Month = 1;
   }    
}

/**
  * @brief  RTC Set date
  * @param  Date: Pointer to Date structure
  * @retval None
  */
void BSP_RTC_SetDate(  RTC_DateTypeDef *Date)
{
   HAL_RTC_SetDate(&hrtc, Date, RTC_FORMAT_BIN);
}

void BSP_BACKUP_SaveParameter(uint32_t address, uint32_t data)
{
  HAL_RTCEx_BKUPWrite(&hrtc,address,data);  
}


uint32_t BSP_BACKUP_RestoreParameter(uint32_t address)
{
   return HAL_RTCEx_BKUPRead(&hrtc,address);  
}
