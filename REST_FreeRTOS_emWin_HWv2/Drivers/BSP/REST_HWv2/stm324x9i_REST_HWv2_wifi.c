#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "esp8266.h"

/* Wifi network settings */
#define WIFINAME            "ssid"
#define WIFIPASS            "password"

/* ESP working structure and result enumeration */
volatile ESP_t ESP;
ESP_Result_t espRes;

/* Client connection pointer */
ESP_CONN_t* conn;

/* Connection manupulation */
uint32_t bw;

uint8_t responseData[] = 
"HTTP/1.1 200 OK\r\n"
"Content-Type: text/html\r\n"
"Connection: close\r\n"
"\r\n"
"<html>\n"
"   <head>\n"
"       <meta http-equiv=\"Refresh\" content=\"4\" />\n"
"   </head>\n"
"   <body>\n"
"       <h1>Welcome to web server produced by REST HWv2 module!</h1>\n"
"       This website will constantly update itself every 1 second!\n"
"   </body>\n"
"</html>\n";

/* Thread prototypes */
void ESP_Update_Thread(void const* params);
void ESP_Main_Thread(void const* params);

/* Thread definitions */
osThreadDef(ESP_Update, ESP_Update_Thread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
osThreadDef(ESP_Main, ESP_Main_Thread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);

osThreadId ESP_Update_ThreadId, ESP_Main_ThreadId;

/* ESP callback declaration */
int ESP_Callback(ESP_Event_t evt, ESP_EventParams_t* params);


void BSP_WIFI_Init(void) {

/* Initialize WiFi threads */
	
  ESP_Update_ThreadId = osThreadCreate(osThread(ESP_Update), NULL);
  ESP_Main_ThreadId = osThreadCreate(osThread(ESP_Main), NULL);
	
	if(ESP_Update_ThreadId == NULL | ESP_Main_ThreadId == NULL) {
		/* TODO: need to add error condition */
	}
}


/***********************************************/
/**       WiFi Thread implementations         **/
/***********************************************/

/**
 * \brief  Update ESP received data thread
 */
void ESP_Update_Thread(void const* params) {
    
	while (1) {
        /* Process ESP update */       
				ESP_Update(&ESP);
				osDelay(50);				
    }
}

/**
 * \brief  Application thread to work with ESP module only
 */
void ESP_Main_Thread(void const* params) {
    
	
		/* Init ESP library with 115200 bauds */
    if ((espRes = ESP_Init(&ESP, 115200, ESP_Callback)) == espOK) {
        printf("ESP module init successfully!\r\n");
    } else {
        printf("ESP Init error. Status: %d\r\n", espRes);
    }
    
    /* Try to connect to wifi network in blocking mode */
    if ((espRes = ESP_STA_Connect(&ESP, WIFINAME, WIFIPASS, NULL, 0, 1)) == espOK) {
        printf("Connected to network\r\n");
    } else {
        printf("Problems trying to connect to network: %d\r\n", espRes);
    }
    
    /* Enable server mode on port 80 (HTTP) */
    if ((espRes = ESP_SERVER_Enable(&ESP, 80, 1)) == espOK) {
        printf("Server mode is enabled. Try to connect to %d.%d.%d.%d to see the magic\r\n", ESP.STAIP[0], ESP.STAIP[1], ESP.STAIP[2], ESP.STAIP[3]);
    } else {
        printf("Problems trying to enable server mode: %d\r\n", espRes);
    }
    
    while (1) {
        ESP_ProcessCallbacks(&ESP);         /* Process all callbacks */
    }
}

/***********************************************/
/**               Library callback            **/
/***********************************************/
int ESP_Callback(ESP_Event_t evt, ESP_EventParams_t* params) {
    ESP_CONN_t* conn;
    uint8_t* data;
    
    switch (evt) {                              /* Check events */
        case espEventIdle:
            //printf("Stack is IDLE!\r\n");
            break;
        case espEventConnActive: {
            conn = (ESP_CONN_t *)params->CP1;   /* Get connection for event */
            //printf("Connection %d just became active!\r\n", conn->Number);
            break;
        }
        case espEventConnClosed: {
            conn = (ESP_CONN_t *)params->CP1;   /* Get connection for event */
            //printf("Connection %d was just closed!\r\n", conn->Number);
            break;
        }
        case espEventDataReceived: {
            conn = (ESP_CONN_t *)params->CP1;   /* Get connection for event */
            data = (uint8_t *)params->CP2;      /* Get data */
        
            /* Notify user about informations */
            //printf("Data received: %d bytes\r\n", params->UI);
        
            if (ESP_IsReady(&ESP) == espOK) {   /* Send data back when we have received all the data from device */
                if (strstr((char *)data, "/favicon")) { /* When browser requests favicon image, ignore it! */
                    ESP_CONN_Close(&ESP, conn, 0);      /* Close connection directly on favicon request */
                } else {
                    espRes = ESP_CONN_Send(&ESP, conn, responseData, sizeof(responseData), &bw, 0); /* Send data on other requests */
                }
            }
            break;
        }
        case espEventDataSent:
            conn = (ESP_CONN_t *)params->CP1;   /* Get connection for event */
            printf("Data sent conn: %d\r\n", conn->Number);
            printf("Close conn resp: %d\r\n", ESP_CONN_Close(&ESP, conn, 0));
            osDelay(100);
            break;
        case espEventDataSentError:
            conn = (ESP_CONN_t *)params->CP1;   /* Get connection for event */
            ESP_CONN_Close(&ESP, conn, 0);
            break;
        default:
            break;
    }
    
    return 0;
}

/* 1ms handler function, called from SysTick interrupt */
void TM_DELAY_1msHandler() {
    ESP_UpdateTime(&ESP, 1);                /* Update ESP library time for 1 ms */
}
