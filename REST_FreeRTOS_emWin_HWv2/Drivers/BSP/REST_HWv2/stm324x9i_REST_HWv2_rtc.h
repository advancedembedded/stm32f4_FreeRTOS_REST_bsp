#include "stm32f4xx_hal.h"

void BSP_RTC_Init(void);
void BSP_RTC_GetTime(  RTC_TimeTypeDef *Time);
void BSP_RTC_SetTime(  RTC_TimeTypeDef *Time);
void BSP_RTC_GetDate(  RTC_DateTypeDef *Date);
void BSP_RTC_SetDate(  RTC_DateTypeDef *Date);
void BSP_BACKUP_SaveParameter(uint32_t address, uint32_t data);
uint32_t BSP_BACKUP_RestoreParameter(uint32_t address);
