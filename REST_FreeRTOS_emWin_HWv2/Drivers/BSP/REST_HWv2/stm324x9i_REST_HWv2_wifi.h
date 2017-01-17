#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* Thread prototypes */
void ESP_Update_Thread(void const* params);
void ESP_Main_Thread(void const* params);

void BSP_WIFI_Init(void);

