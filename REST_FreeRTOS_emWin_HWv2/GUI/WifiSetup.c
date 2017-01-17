/*********************************************************************
*                                                                    *
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
*                                                                    *
**********************************************************************
*                                                                    *
* C-file generated by:                                               *
*                                                                    *
*        GUI_Builder for emWin version 5.32                          *
*        Compiled Oct  8 2015, 11:59:02                              *
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

#define ID_WINDOW_0 (GUI_ID_USER + 0x02)
#define ID_HEADER_0 (GUI_ID_USER + 0x07)
#define ID_TEXT_HEADER (GUI_ID_USER + 0x08)
#define ID_BUTTON_CANCEL (GUI_ID_USER + 0x11)
#define ID_BUTTON_DISCONNECT (GUI_ID_USER + 0x14)
#define ID_BUTTON_DONE (GUI_ID_USER + 0x15)
#define ID_TEXT_1 (GUI_ID_USER + 0x17)
#define ID_TEXT_2 (GUI_ID_USER + 0x18)
#define ID_TEXT_3 (GUI_ID_USER + 0x19)
#define ID_TEXT_4 (GUI_ID_USER + 0x1A)
#define ID_TEXT_5 (GUI_ID_USER + 0x1B)
#define ID_TEXT_6 (GUI_ID_USER + 0x1C)
#define ID_TEXT_7 (GUI_ID_USER + 0x1D)
#define ID_TEXT_8 (GUI_ID_USER + 0x1E)
#define ID_TEXT_9 (GUI_ID_USER + 0x1F)
#define ID_TEXT_10 (GUI_ID_USER + 0x20)
#define ID_TEXT_11 (GUI_ID_USER + 0x21)
#define ID_TEXT_12 (GUI_ID_USER + 0x22)
#define ID_TEXT_13 (GUI_ID_USER + 0x23)
#define ID_TEXT_14 (GUI_ID_USER + 0x24)

extern uint8_t state;
extern uint8_t temperature;

/*********************************************************************
*
*       _aDialogCreate
*/
static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] =
{
    { WINDOW_CreateIndirect, "Window", ID_WINDOW_0, 8, 1, 480, 272, 0, 0x0, 0 },
    { HEADER_CreateIndirect, "Header", ID_HEADER_0, 0, 0, 480, 50, 0, 0x0, 0 },
    { TEXT_CreateIndirect, "WIFI SETUP", ID_TEXT_HEADER, 0, 0, 480, 50, 0, 0x64, 0 },
    { BUTTON_CreateIndirect, "CANCEL", ID_BUTTON_CANCEL, 20, 220, 80, 28, 0, 0x0, 0 },
    { BUTTON_CreateIndirect, "Disconnect from Network", ID_BUTTON_DISCONNECT, 128, 220, 222, 28, 0, 0x0, 0 },
    { BUTTON_CreateIndirect, "DONE", ID_BUTTON_DONE, 378, 220, 76, 28, 0, 0x0, 0 },
    { TEXT_CreateIndirect, "Text", ID_TEXT_1, 27, 60, 154, 20, 0, 0x64, 0 },
    { TEXT_CreateIndirect, "Text", ID_TEXT_2, 195, 60, 181, 20, 0, 0x64, 0 },
    { TEXT_CreateIndirect, "Text", ID_TEXT_3, 78, 82, 103, 20, 0, 0x64, 0 },
    { TEXT_CreateIndirect, "Text", ID_TEXT_4, 195, 82, 182, 20, 0, 0x64, 0 },
    { TEXT_CreateIndirect, "Text", ID_TEXT_5, 23, 104, 157, 20, 0, 0x64, 0 },
    { TEXT_CreateIndirect, "Text", ID_TEXT_6, 195, 104, 157, 20, 0, 0x64, 0 },
    { TEXT_CreateIndirect, "Text", ID_TEXT_7, 29, 126, 151, 20, 0, 0x64, 0 },
    { TEXT_CreateIndirect, "Text", ID_TEXT_8, 195, 126, 184, 20, 0, 0x64, 0 },
    { TEXT_CreateIndirect, "Text", ID_TEXT_9, 17, 148, 161, 20, 0, 0x64, 0 },
    { TEXT_CreateIndirect, "Text", ID_TEXT_10, 195, 148, 203, 20, 0, 0x64, 0 },
    { TEXT_CreateIndirect, "Text", ID_TEXT_11, 68, 172, 74, 20, 0, 0x64, 0 },
    { TEXT_CreateIndirect, "Text", ID_TEXT_12, 145, 172, 175, 20, 0, 0x64, 0 },
    { TEXT_CreateIndirect, "Text", ID_TEXT_13, 321, 172, 87, 20, 0, 0x64, 0 },
    { TEXT_CreateIndirect, "Text", ID_TEXT_14, 106, 188, 232, 20, 0, 0x64, 0 },
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
        // Initialization of 'Text'
        //
        hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_HEADER);
        TEXT_SetFont(hItem, GUI_FONT_32B_1);
        TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
        TEXT_SetTextColor(hItem, GUI_MAKE_COLOR(0x00FFFFFF));
        //
        // Initialization of 'Button'
        //
        hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_CANCEL);
        BUTTON_SetFont(hItem, GUI_FONT_16B_1);
        BUTTON_SetTextColor(hItem,0,GUI_MAKE_COLOR(0x00FFFFFF));
        //
        // Initialization of 'Button'
        //
        hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_DISCONNECT);
        BUTTON_SetFont(hItem, GUI_FONT_16B_1);
        BUTTON_SetTextColor(hItem,0,GUI_MAKE_COLOR(0x00FFFFFF));
        // Initialization of 'Button'
        //
        hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_DONE);
        BUTTON_SetFont(hItem, GUI_FONT_16B_1);
        BUTTON_SetTextColor(hItem,0,GUI_MAKE_COLOR(0x00FFFFFF));
        //
        // Initialization of 'Text'
        //
        hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_1);
        TEXT_SetFont(hItem, GUI_FONT_20_ASCII);
        TEXT_SetText(hItem, "Wifi Network:");
        TEXT_SetTextAlign(hItem, GUI_TA_RIGHT | GUI_TA_VCENTER);
        TEXT_SetTextColor(hItem, GUI_MAKE_COLOR(0x00787878));
        //
        // Initialization of 'Text'
        //
        hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_2);
        TEXT_SetText(hItem, "My Wifi Network");
        TEXT_SetFont(hItem, GUI_FONT_20_ASCII);
        TEXT_SetTextAlign(hItem, GUI_TA_LEFT | GUI_TA_VCENTER);
        //
        // Initialization of 'Text'
        //
        hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_3);
        TEXT_SetText(hItem, "Status:");
        TEXT_SetTextAlign(hItem, GUI_TA_RIGHT | GUI_TA_VCENTER);
        TEXT_SetFont(hItem, GUI_FONT_20_ASCII);
        TEXT_SetTextColor(hItem, GUI_MAKE_COLOR(0x00787878));
        //
        // Initialization of 'Text'
        //
        hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_4);
        TEXT_SetFont(hItem, GUI_FONT_20_ASCII);
        TEXT_SetText(hItem, "Working");
        TEXT_SetTextAlign(hItem, GUI_TA_LEFT | GUI_TA_VCENTER);
        //
        // Initialization of 'Text'
        //
        hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_5);
        TEXT_SetFont(hItem, GUI_FONT_20_ASCII);
        TEXT_SetTextAlign(hItem, GUI_TA_RIGHT | GUI_TA_VCENTER);
        TEXT_SetText(hItem, "IPAddress:");
        TEXT_SetTextColor(hItem, GUI_MAKE_COLOR(0x00787878));
        //
        // Initialization of 'Text'
        //
        hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_6);
        TEXT_SetTextAlign(hItem, GUI_TA_LEFT | GUI_TA_VCENTER);
        TEXT_SetText(hItem, "192.168.1.30");
        TEXT_SetFont(hItem, GUI_FONT_20_ASCII);
        //
        // Initialization of 'Text'
        //
        hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_7);
        TEXT_SetFont(hItem, GUI_FONT_20_ASCII);
        TEXT_SetTextAlign(hItem, GUI_TA_RIGHT | GUI_TA_VCENTER);
        TEXT_SetText(hItem, "Thermostat MAC:");
        TEXT_SetTextColor(hItem, GUI_MAKE_COLOR(0x00787878));
        //
        // Initialization of 'Text'
        //
        hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_8);
        TEXT_SetFont(hItem, GUI_FONT_20_ASCII);
        TEXT_SetText(hItem, "00:50:56:8A:E3:C5");
        //
        // Initialization of 'Text'
        //
        hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_9);
        TEXT_SetTextAlign(hItem, GUI_TA_RIGHT | GUI_TA_VCENTER);
        TEXT_SetFont(hItem, GUI_FONT_20_ASCII);
        TEXT_SetText(hItem, "Thermostat CRC:");
        TEXT_SetTextColor(hItem, GUI_MAKE_COLOR(0x00787878));
        //
        // Initialization of 'Text'
        //
        hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_10);
        TEXT_SetTextAlign(hItem, GUI_TA_LEFT | GUI_TA_VCENTER);
        TEXT_SetText(hItem, "3453");
        TEXT_SetFont(hItem, GUI_FONT_20_ASCII);
        //
        // Initialization of 'Text'
        //
        hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_11);
        TEXT_SetText(hItem, "Please visit");
        TEXT_SetTextAlign(hItem, GUI_TA_RIGHT | GUI_TA_VCENTER);
        TEXT_SetFont(hItem, GUI_FONT_16_1);
        //
        // Initialization of 'Text'
        //
        hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_12);
        TEXT_SetTextAlign(hItem, GUI_TA_LEFT | GUI_TA_VCENTER);
        TEXT_SetText(hItem, "http://www.restranger.com");
        TEXT_SetFont(hItem, GUI_FONT_16B_1);
        //
        // Initialization of 'Text'
        //
        hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_13);
        TEXT_SetTextAlign(hItem, GUI_TA_LEFT | GUI_TA_VCENTER);
        TEXT_SetText(hItem, "to setup your");
        TEXT_SetFont(hItem, GUI_FONT_16_1);
        //
        // Initialization of 'Text'
        //
        hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_14);
        TEXT_SetText(hItem, "thermostat for remote access");
        TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
        TEXT_SetFont(hItem, GUI_FONT_16_1);
        break;
    case WM_NOTIFY_PARENT:
        Id    = WM_GetId(pMsg->hWinSrc);
        NCode = pMsg->Data.v;
        switch(Id)
        {
        case ID_BUTTON_CANCEL: // Notifications sent by 'Button'
            switch(NCode)
            {
            case WM_NOTIFICATION_CLICKED:
                GUI_Delay(100);
                state=17;
                break;
            }
            break;
        case ID_BUTTON_DISCONNECT: // Notifications sent by 'Button'
            switch(NCode)
            {
            case WM_NOTIFICATION_CLICKED:
                GUI_Delay(100);
                state=17;
                break;
            }
            break;
        case ID_BUTTON_DONE: // Notifications sent by 'Button'
            switch(NCode)
            {
            case WM_NOTIFICATION_CLICKED:
                GUI_Delay(100);
                state=17;
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
*       CreateWindow
*/
WM_HWIN CreateWifiSetup(void);
WM_HWIN CreateWifiSetup(void)
{
    WM_HWIN hWin;

    hWin = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), _cbDialog, WM_HBKWIN, 0, 0);
    return hWin;
}

/*************************** End of file ****************************/