/**
  ******************************************************************************
  * @file    uart.c
  * @author  WI6LABS
  * @version V1.0.0
  * @date    16-September-2016
  * @brief   provide the UART interface
  *
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
/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm8sxx_system
  * @{
  */

/** @addtogroup stm8sxx_System_Private_Includes
  * @{
  */
#include "uart.h"

#ifdef __cplusplus
 extern "C" {
#endif

/**
  * @}
  */

/** @addtogroup stm8sxx_System_Private_Defines
  * @{
  */

/// @brief number of received characters

/**
  * @}
  */

/** @addtogroup stm8sxx_System_Private_TypesDefinitions
  * @{
  */

/// @brief defines the global attributes of the UART
typedef struct {
  uint8_t begin;
  uint8_t end;
  volatile uint16_t data_available;
  uart_id_e uart_id;
  uint8_t rxpData[UART_RCV_SIZE];
}uart_conf_t;

/**
  * @}
  */

/** @addtogroup stm8sxx_System_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup stm8sxx_System_Private_Variables
  * @{
  */
/// @brief uart caracteristics
static uart_conf_t g_uart_config[NB_UART_MANAGED] = {
  //UART2 PD5/PD6
  {
    .uart_id = UART2_E,
    .data_available = 0,
    .begin = 0,
    .end = 0
  }
};

/**
  * @}
  */

/** @addtogroup stm8sxx_System_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/** @addtogroup stm8sxx_System_Private_Functions
  * @{
  */

/**
  * @brief  Function called to initialize the uart interface
  * @param  serial_id : one of the defined serial interface
  * @param  baudRate : baudrate to apply to the uart
  * @retval None
  */
void uart_init(uart_id_e uart_id, uint32_t baudRate)
{
  if(uart_id == UART2_E) {
    UART2_DeInit();
    UART2_Init(baudRate, UART2_WORDLENGTH_8D, UART2_STOPBITS_1, UART2_PARITY_NO,
                UART2_SYNCMODE_CLOCK_DISABLE, UART2_MODE_TXRX_ENABLE);
    UART2_ITConfig(UART2_IT_RXNE_OR, ENABLE);

    enableInterrupts();

    uart_flush(uart_id);

    //Clear status register
    (void)UART2->SR;
    (void)UART2->DR;
  }
}

/**
  * @brief  Function called to deinitialize the uart interface
  * @param  serial_id : one of the defined serial interface
  * @retval None
  */
void uart_deinit(uart_id_e uart_id)
{
  if(uart_id == UART2_E) {
    UART2_DeInit();
  }
}

/**
  * @brief  Function returns the amount of data available
  * @param  serial_id : one of the defined serial interface
  * @retval The number of serial data available - int
  */
int uart_available(uart_id_e uart_id)
{
  if(uart_id < NB_UART_MANAGED) {
    return g_uart_config[uart_id].data_available;
  } else {
    return 0;
  }
}

/**
  * @brief  Return the first element of the rx buffer
  * @param  serial_id : one of the defined serial interface
  * @retval The first byte of incoming serial data available (or -1 if no data is available) - int
  */
int8_t uart_read(uart_id_e uart_id)
{
  int8_t data = -1;

  if(uart_id < NB_UART_MANAGED) {
    if(g_uart_config[uart_id].data_available > 0) {
      data = g_uart_config[uart_id].rxpData[g_uart_config[uart_id].begin++];
      if(g_uart_config[uart_id].begin >= UART_RCV_SIZE) {
        g_uart_config[uart_id].begin = 0;
      }
      g_uart_config[uart_id].data_available--;
    }
  }

  return data;
}

/**
  * @brief  Return the first element of the rx buffer without removing it from
  *         the buffer
  * @param  serial_id : one of the defined serial interface
  * @retval The first byte of incoming serial data available (or -1 if no data is available) - int
  */
int8_t uart_peek(uart_id_e uart_id)
{
  int8_t data = -1;

  if(uart_id < NB_UART_MANAGED) {
    if(g_uart_config[uart_id].data_available > 0) {
      data = g_uart_config[uart_id].rxpData[g_uart_config[uart_id].begin];
    }
  }

  return data;
}

/**
  * @brief  Flush the content of the RX buffer
  * @param  serial_id : one of the defined serial interface
  * @retval None
  */
void uart_flush(uart_id_e uart_id)
{
  if(uart_id < NB_UART_MANAGED) {
    g_uart_config[uart_id].data_available = 0;
    g_uart_config[uart_id].end = 0;
    g_uart_config[uart_id].begin = 0;
  }
}

/**
  * @brief  write the data on the uart
  * @param  serial_id : one of the defined serial interface
  * @param  data : byte to write
  * @retval The number of bytes written
  */
uint8_t uart_write(uart_id_e uart_id, uint8_t data)
{
  if(uart_id == UART2_E){
    UART2_SendData8(data);
    while(UART2_GetFlagStatus(UART2_FLAG_TXE) == RESET);
    return 1;
  }

  return 0;
}

/**
  * @brief  Read receive byte from uart
  * @param  UartHandle pointer on the uart reference
  * @param  byte byte to read
  * @retval None
  */
void uart_getc(uart_id_e uart_id, uint8_t byte)
{
  if(uart_id < NB_UART_MANAGED) {
    if(g_uart_config[uart_id].data_available >= UART_RCV_SIZE) {
      return;
    }
    g_uart_config[uart_id].rxpData[g_uart_config[uart_id].end++] = byte;
    if(g_uart_config[uart_id].end >= UART_RCV_SIZE) {
      g_uart_config[uart_id].end = 0;
    }
    g_uart_config[uart_id].data_available++;
  }
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
