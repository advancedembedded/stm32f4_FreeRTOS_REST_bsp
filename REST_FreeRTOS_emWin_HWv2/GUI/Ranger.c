#include "dialog.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "calibration.h"

static void GUIThread(void const * argument);
osThreadDef(GUI_Thread, GUIThread, osPriorityNormal, 0, 2048);
osThreadId GUI_Thread_ThreadId;

uint32_t GUI_FreeMem = 0;
uint8_t state;
uint8_t temperature;

void setSkin()
{
    BUTTON_SKINFLEX_PROPS Props;/* Temp variable to store customized settings of the skin */
    BUTTON_GetSkinFlexProps(&Props, BUTTON_SKINFLEX_PI_ENABLED); /* Get current skin properties of enabled button */
    Props.aColorFrame[0] = GUI_LIGHTGRAY; /* Change colors of several parts of the button */
    Props.aColorFrame[1] = GUI_LIGHTGRAY;
    Props.aColorFrame[2] = 0x000DA088;
    Props.aColorUpper[0] = //0x0064B49C;
        Props.aColorUpper[1] = 0x0059A89C;//0x0064B49C;
    Props.aColorLower[0] = 0x0059A88D; //0x0x0064B49C;0072856A;
    Props.aColorLower[1] = 0x0048856A;//   0x0072856A;//0x004B9175;//0x0072856A;
    BUTTON_SetSkinFlexProps(&Props, BUTTON_SKINFLEX_PI_ENABLED); /* Store new skin properties to enabled button */
    BUTTON_SetSkinFlexProps(&Props, BUTTON_SKINFLEX_PI_FOCUSSED); /* Store new skin properties to enabled button */
    Props.aColorUpper[0] = 0x000DCC88;
    Props.aColorUpper[1] = 0x000DCC88;
    Props.aColorLower[0] = 0x000DCC88;
    Props.aColorLower[1] = 0x000DCC88;
    BUTTON_SetSkinFlexProps(&Props, BUTTON_SKINFLEX_PI_PRESSED); /* Store new skin properties to enabled button */

    CHECKBOX_SKINFLEX_PROPS Props1;/* Temp variable to store customized settings of the skin */
    CHECKBOX_GetSkinFlexProps(&Props1, CHECKBOX_SKINFLEX_PI_ENABLED); /* Get current skin properties of enabled button */
    Props1.aColorFrame[0] = GUI_GREEN; /* Change colors of several parts of the button */
    Props1.aColorFrame[1] = GUI_BLUE;
    Props1.aColorFrame[2] = GUI_RED;
    Props1.aColorInner[0] = 0x0059A89C;
    Props1.aColorInner[1] = 0x0059A88D;
    //Props1.aColorInner[2] = 0x0048856A;
    CHECKBOX_SetSkinFlexProps(&Props1, CHECKBOX_SKINFLEX_PI_ENABLED); /* Store new skin properties to enabled button */
    CHECKBOX_SetSkinFlexProps(&Props1, CHECKBOX_SKINFLEX_PI_DISABLED); /* Store new skin properties to enabled button */
}

/**
  * @brief  Start task
  * @param  argument: pointer that is passed to the thread function as start argument.
  * @retval None
  */
static void GUIThread(void const * argument)
{   
	
	/* Initialize GUI */
	GUI_Init();   
  WM_MULTIBUF_Enable(1);
	
	/* Check for calibration */
  if(CalibrationIsDone() == 0)
  {
    CalibrationInit();
  }
	
	temperature=60;
  state=1;
  BUTTON_Handle          hButton;
  uint8_t								 buffer[10];

  setSkin();
  uint16_t xPos0 = 10;
  uint16_t yPos0 = 10;
  uint16_t xPos1 = 100;
  uint16_t yPos1  = 100;
  uint16_t r = 50;
  
  /* Show the main menu */
  /* Gui background Task */
  while(1) {
		
		switch (state)
        {
        case 0:
            break;
        case 1:
            CreateHomeWin();
            state=0;
            break;
        case 2:
//            CreateNumericKeyboard();
            state=0;
            break;
        case 3:
            CreateMode();
            GUI_Delay(100);
            state=0;
            break;
        case 4:
            CreateSettings();
            state=0;
            break;
        case 5:
            CreateFAN();
            state=0;
            break;
        case 6:
            state=0;
            break;
        case 7:
            CreateSchedule();
            state=0;
            break;
        case 8:
//            CreateAlphaKeyboard();
            state=0;
            break;
        case 10:
            //CreateColors();
            state=0;
            break;
        case 11:
            //CreateTimeDate();
            state=0;
            break;
        case 12:
            CreateKeyboardLockout();
            state=0;
            break;
        case 13:
            CreateSettingsSchedule();
            state=0;
            break;
        case 14:
            CreateLanguages();
            state=0;
            break;
        case 15:
            //CreateProfile();
            state=0;
            break;
        case 16:
            //CreatePreferences();
            state=0;
            break;
        case 17:
            CreateSystemSetup();
            state=0;
            break;
        case 20:
            CreateThermostatLocations();
            state=0;
            break;
        case 21:
            CreateSystemType();
            state=0;
            break;
        case 22:
            CreateThermostatControls();
            state=0;
            break;
        case 23:
            CreateFanControl();
            state=0;
            break;
        case 24:
            CreateWifiSetup();
            state=0;
            break;
        case 25:
            CreateBackupHeat();
            state=0;
            break;
        case 26:
            CreateCoolingStages();
            state=0;
            break;
        case 27:
            CreateHeatingStages();
            state=0;
            break;
        case 30:
            CreateSettingsSchedule();
            state=0;
            break;
        case 31:
            //CreateWeekend();
            state=0;
            break;
        case 32:
            CreateEachDay();
            state=0;
            break;
        case 33:
            //CreateVacation();
            state=0;
            break;
        case 34:
            //CreateSettingsSchedule();
            state=0;
            break;
        }
    GUI_Exec(); /* Do the background work ... Update windows etc.) */
    osDelay(20); /* Nothing left to do for the moment ... Idle processing */ /* osDelay */
		GUI_FreeMem = GUI_ALLOC_GetNumFreeBytes();
  }
}

void BSP_GUI_Init(void) {

GUI_Thread_ThreadId = osThreadCreate(osThread(GUI_Thread), NULL);
	/* TODO: add error check */	
}