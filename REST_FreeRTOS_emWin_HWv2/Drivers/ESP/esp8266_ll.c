/** 
   ----------------------------------------------------------------------
    Copyright (c) 2016 Tilen Majerle

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without restriction,
    including without limitation the rights to use, copy, modify, merge,
    publish, distribute, sublicense, and/or sell copies of the Software, 
    and to permit persons to whom the Software is furnished to do so, 
    subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
    AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
   ----------------------------------------------------------------------
 */
#include "esp8266_ll.h"

/* Include platform dependant libraries */
#include "main.h"
#include "stm32f4xx_hal.h"

UART_HandleTypeDef huart2;
uint8_t RxBuffer;

/* Buffer's length must be select according to real messages frequency */
#define RXBUF_LEN            128 // must be power of 2
#define TXBUF_LEN            128 // must be power of 2
#define RXBUF_MSK            (RXBUF_LEN-1)
#define TXBUF_MSK            (TXBUF_LEN-1)

static uint32_t TX_complete = 1;


uint8_t ESP_LL_Init(ESP_LL_t* LL) {
  
		/* Init USART */
		huart2.Instance = USART2;
		huart2.Init.BaudRate = LL->Baudrate;
		huart2.Init.WordLength = UART_WORDLENGTH_8B;
		huart2.Init.StopBits = UART_STOPBITS_1;
		huart2.Init.Parity = UART_PARITY_NONE;
		huart2.Init.Mode = UART_MODE_TX_RX;
		huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE; //UART_HWCONTROL_RTS_CTS;
		huart2.Init.OverSampling = UART_OVERSAMPLING_16;
		if (HAL_UART_Init(&huart2) != HAL_OK)
		{
			Error_Handler();
		}
         
		/* Put UART peripheral in reception process */  
		if(HAL_UART_Receive_IT(&huart2, &RxBuffer, 1) != HAL_OK)
		{
			Error_Handler();
		}
		
		/* We were successful */
		return 0;
}

uint8_t ESP_LL_SendData(ESP_LL_t* LL, const uint8_t* data, uint16_t count) {
    
		
	  /* Disable the UART Parity Error Interrupt and RXNE interrupt*/
    CLEAR_BIT(huart2.Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
		/* Send data */
		HAL_UART_Transmit(&huart2, (uint8_t *)data, count,2000);
    /* We were successful */
    SET_BIT(huart2.Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE);
		return 0;
}

uint8_t ESP_LL_SetReset(ESP_LL_t* LL, uint8_t state) {
    /* Set pin according to status */
    if (state == ESP_RESET_SET) {
        HAL_GPIO_WritePin(GPIOC, WIFI_RESET_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOC, WIFI_RESET_Pin, GPIO_PIN_SET);
    }
    
    /* We are OK */
    return 0;
}

uint8_t ESP_LL_SetRTS(ESP_LL_t* LL, uint8_t state) {
    
		if (state == ESP_RTS_SET) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    }
		/* We are OK */
    return 0;
}

/* USART receive interrupt handler */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
		
		ESP_DataReceived(&RxBuffer, 1);
		/* Put UART peripheral in reception process */  
		if(HAL_UART_Receive_IT(huart, &RxBuffer, 1) != HAL_OK)
		{
			//Error_Handler();
		}
}
