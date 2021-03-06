/*********************************************************************
*                                                                    *
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
*                                                                    *
**********************************************************************
*                                                                    *
* C-file generated by:                                               *
*                                                                    *
*        GUI_Builder for emWin version 5.28                          *
*        Compiled Jan 30 2015, 16:41:06                              *
*        (c) 2015 Segger Microcontroller GmbH & Co. KG               *
*                                                                    *
**********************************************************************
*                                                                    *
*        Internet: www.segger.com  Support: support@segger.com       *
*                                                                    *
**********************************************************************
*/

#include "DIALOG.h"
#include "stm32f4xx_hal.h"

extern uint8_t state;

#define ID_WINDOW_0  (GUI_ID_USER + 0x00)
#define ID_BUTTON_OFF  (GUI_ID_USER + 0x01)
#define ID_BUTTON_HEAT  (GUI_ID_USER + 0x02)
#define ID_BUTTON_COOL  (GUI_ID_USER + 0x03)
#define ID_BUTTON_AUTO  (GUI_ID_USER + 0x04)
#define ID_BUTTON_EHEAT  (GUI_ID_USER + 0x05)
#define ID_BUTTON_CANCEL  (GUI_ID_USER + 0x06)
#define ID_BUTTON_DONE  (GUI_ID_USER + 0x07)
#define ID_HEADER_0  (GUI_ID_USER + 0x08)
#define ID_TEXT_HEADER  (GUI_ID_USER + 0x09)

/*********************************************************************
*
*       _aDialogCreate
*/
static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] =
{
    { WINDOW_CreateIndirect, "Mode", ID_WINDOW_0, 0, 0, 480, 272, 0, 0x0, 0 },
    { BUTTON_CreateIndirect, "OFF", ID_BUTTON_OFF, 18, 130, 80, 30, 0, 0x0, 0 },
    { BUTTON_CreateIndirect, "HEAT", ID_BUTTON_HEAT, 108, 130, 80, 30, 0, 0x0, 0 },
    { BUTTON_CreateIndirect, "COOL", ID_BUTTON_COOL, 198, 130, 80, 30, 0, 0x0, 0 },
    { BUTTON_CreateIndirect, "AUTO", ID_BUTTON_AUTO, 289, 130, 80, 30, 0, 0x0, 0 },
    { BUTTON_CreateIndirect, "E-HEAT", ID_BUTTON_EHEAT, 380, 130, 80, 30, 0, 0x0, 0 },
    { BUTTON_CreateIndirect, "CANCEL", ID_BUTTON_CANCEL, 18, 213, 80, 30, 0, 0x0, 0 },
    { BUTTON_CreateIndirect, "DONE", ID_BUTTON_DONE, 380, 213, 80, 30, 0, 0x0, 0 },
    { HEADER_CreateIndirect, "Header", ID_HEADER_0, -1, 0, 482, 50, 0, 0x0, 0 },
    { TEXT_CreateIndirect, "SELECT MODE", ID_TEXT_HEADER, 163, 8, 480, 50, 0, 0x64, 0 },
};

/*********************************************************************
*
*       _cbDialog
*/
static void _cbDialog(WM_MESSAGE * pMsg)
{
    WM_HWIN hItem;
    int     NCode;
    int     Id;

    switch (pMsg->MsgId)
    {
    case WM_INIT_DIALOG:
        //
        // Initialization of 'Button'
        //
        hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_OFF);
        BUTTON_SetFont(hItem, GUI_FONT_16B_1);
        BUTTON_SetTextColor(hItem, BUTTON_CI_UNPRESSED, GUI_WHITE);
        BUTTON_SetTextColor(hItem, BUTTON_CI_PRESSED, GUI_WHITE);
        //
        // Initialization of 'Button'
        //
        hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_HEAT);
        BUTTON_SetFont(hItem, GUI_FONT_16B_1);
        BUTTON_SetTextColor(hItem, BUTTON_CI_UNPRESSED, GUI_RED);
        BUTTON_SetTextColor(hItem, BUTTON_CI_PRESSED, GUI_RED);
        //
        // Initialization of 'Button'
        //
        hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_COOL);
        BUTTON_SetFont(hItem, GUI_FONT_16B_1);
        BUTTON_SetTextColor(hItem, BUTTON_CI_UNPRESSED, GUI_BLUE);
        BUTTON_SetTextColor(hItem, BUTTON_CI_PRESSED, GUI_BLUE);

        //
        // Initialization of 'Button'
        //
        hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_AUTO);
        BUTTON_SetFont(hItem, GUI_FONT_16B_1);
        BUTTON_SetTextColor(hItem, BUTTON_CI_UNPRESSED, GUI_WHITE);
        BUTTON_SetTextColor(hItem, BUTTON_CI_PRESSED, GUI_WHITE);
        //
        // Initialization of 'Button'
        //
        hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_EHEAT);
        BUTTON_SetFont(hItem, GUI_FONT_16B_1);
        BUTTON_SetTextColor(hItem, BUTTON_CI_UNPRESSED, GUI_WHITE);
        BUTTON_SetTextColor(hItem, BUTTON_CI_PRESSED, GUI_WHITE);
        //
        // Initialization of 'Button'
        //
        hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_CANCEL);
        BUTTON_SetFont(hItem, GUI_FONT_16B_1);
        BUTTON_SetTextColor(hItem, BUTTON_CI_UNPRESSED, GUI_WHITE);
        BUTTON_SetTextColor(hItem, BUTTON_CI_PRESSED, GUI_WHITE);
        //
        // Initialization of 'Button'
        //
        hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_DONE);
        BUTTON_SetFont(hItem, GUI_FONT_16B_1);
        BUTTON_SetTextColor(hItem, BUTTON_CI_UNPRESSED, GUI_WHITE);
        BUTTON_SetTextColor(hItem, BUTTON_CI_PRESSED, GUI_WHITE);
        //
        // Initialization of 'Text'
        //
        hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_HEADER);
        TEXT_SetTextColor(hItem, 0x00FFFFFF);
        TEXT_SetFont(hItem, GUI_FONT_32B_1);
        break;
    case WM_NOTIFY_PARENT:
        Id    = WM_GetId(pMsg->hWinSrc);
        NCode = pMsg->Data.v;
        switch(Id)
        {
        case ID_BUTTON_OFF: // Notifications sent by 'Button'
            switch(NCode)
            {
            case WM_NOTIFICATION_CLICKED:
                break;
            }
            break;
        case ID_BUTTON_HEAT: // Notifications sent by 'Button'
            switch(NCode)
            {
            case WM_NOTIFICATION_CLICKED:
                break;
            }
            break;
        case ID_BUTTON_COOL: // Notifications sent by 'Button'
            switch(NCode)
            {
            case WM_NOTIFICATION_CLICKED:
                break;
            }
            break;
        case ID_BUTTON_AUTO: // Notifications sent by 'Button'
            switch(NCode)
            {
            case WM_NOTIFICATION_CLICKED:
                break;
             }
            break;
        case ID_BUTTON_EHEAT: // Notifications sent by 'Button'
            switch(NCode)
            {
            case WM_NOTIFICATION_CLICKED:
                break;
            }
            break;
        case ID_BUTTON_CANCEL: // Notifications sent by 'Button'
            switch(NCode)
            {
            case WM_NOTIFICATION_CLICKED:
                GUI_Delay(100);
                state=1;
                break;
             }
            break;
        case ID_BUTTON_DONE: // Notifications sent by 'Button'
            switch(NCode)
            {
            case WM_NOTIFICATION_CLICKED:
                GUI_Delay(100);
                state=1;
                break;
            }
            break;
        }
        break;
    default:
        WM_DefaultProc(pMsg);
        break;
    }
}
/*********************************************************************
*
*       CreateMode
*/
WM_HWIN CreateMode(void);
WM_HWIN CreateMode(void)
{
    WM_HWIN hWin;

    hWin = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), _cbDialog, WM_HBKWIN, 0, 0);
    return hWin;
}

/*************************** End of file ****************************/
