/**
  ******************************************************************************
  * @file    uart.c
  * @author  WI6LABS
  * @version V1.0.0
  * @date    08-10-2018
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
#include "uart.h"
#include "Arduino.h"
#include "PinAF_STM8Lxx.h"

#ifdef __cplusplus
extern "C"
{
#endif
#if !defined(NO_HWSERIAL)

// @brief uart caracteristics
#if defined(STM8Lxx)
#define UART_NUM (3)
#elif defined(STM8Sxx)
#define UART_NUM (4)
#else
#error "Unknown Family - unknown UART_NUM"
#endif

static USART_TypeDef *uart_handlers[UART_NUM] = {NULL};
static void (*rx_callback[UART_NUM])(serial_t*);
static serial_t *rx_callback_obj[UART_NUM];

/**
* @brief  Function called to initialize the uart interface
* @param  obj : pointer to serial_t structure
* @retval None
*/

void uart_init(serial_t *obj)
{
  uint32_t databits;
  uint32_t stopbits;
  uint32_t parity;

  if (obj == NULL)
  {
    return;
  }

  // Determine the UART to use (UART_1, UART_2, ...)
  USART_TypeDef *uart_tx = pinmap_peripheral(obj->pin_tx, PinMap_UART_TX);
  USART_TypeDef *uart_rx = pinmap_peripheral(obj->pin_rx, PinMap_UART_RX);

  //Pins Rx/Tx must not be NP
  if(uart_rx == NP || uart_tx == NP) {
    //printf("ERROR: at least one UART pin has no peripheral\n");
    return;
  }

  // Get the peripheral name (UART_1, UART_2, ...) from the pin and assign it to the object
  obj->uart = pinmap_merge_peripheral(uart_tx, uart_rx);

  if(obj->uart == NP) {
    //printf("ERROR: UART pins mismatch\n");
    return;
  }

  enableInterrupts();

#if defined(STM8Sxx)
#ifdef UART1
  if (obj->uart == UART1)
  {
    UART1_DeInit();
    UART1_Init(obj->baudrate, obj->databits, obj->stopbits, obj->parity,
               UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TXRX_ENABLE);
    UART1_ITConfig(UART1_IT_RXNE_OR, ENABLE);

    //Clear status register
    (void)UART1->SR;
    (void)UART1->DR;

    obj->index = 0;
  }
#endif
#ifdef UART2
  if (obj->uart == UART2)
  {
    UART2_DeInit();
    UART2_Init(obj->baudrate, obj->databits, obj->stopbits, obj->parity,
               UART2_SYNCMODE_CLOCK_DISABLE, UART2_MODE_TXRX_ENABLE);
    UART2_ITConfig(UART2_IT_RXNE_OR, ENABLE);

    //Clear status register
    (void)UART2->SR;
    (void)UART2->DR;

    obj->index = 1;
  }
#endif
#ifdef UART3
  if (obj->uart == UART3)
  {
    UART3_DeInit();
    UART3_Init(obj->baudrate, obj->databits, obj->stopbits, obj->parity,
               UART3_MODE_TXRX_ENABLE);
    UART3_ITConfig(UART3_IT_RXNE_OR, ENABLE);

    //Clear status register
    (void)UART3->SR;
    (void)UART3->DR;

    obj->index = 2;
  }
#endif
#ifdef UART4
  if (obj->uart == UART4)
  {
    UART4_DeInit();
    UART4_Init(obj->baudrate, obj->databits, obj->stopbits, obj->parity,
               UART4_MODE_TXRX_ENABLE);
    UART4_ITConfig(UART4_IT_RXNE_OR, ENABLE);

    //Clear status register
    (void)UART4->SR;
    (void)UART4->DR;

    obj->index = 3;
  }
#endif
#elif defined(STM8Lxx)
  if (obj->uart == USART1) {
    CLK_PeripheralClockConfig(CLK_Peripheral_USART1, ENABLE);
    obj->index = 0;
  } else if (obj->uart == USART2) {
    CLK_PeripheralClockConfig(CLK_Peripheral_USART2, ENABLE);
    obj->index = 1;
  } else if (obj->uart == USART3) {
    CLK_PeripheralClockConfig(CLK_Peripheral_USART3, ENABLE);
    obj->index = 2;
  }

  USART_DeInit(obj->uart);

  /* Configure alternate function */
  pin_Set8LAFPin(STM_PIN_AFNUM(pinmap_function(obj->pin_rx, PinMap_UART_RX)));
  pin_Set8LAFPin(STM_PIN_AFNUM(pinmap_function(obj->pin_tx, PinMap_UART_TX)));

  /* Configure USART Tx as alternate function push-pull  (software pull up)*/
  GPIO_ExternalPullUpConfig(get_GPIO_Port(STM_PORT(obj->pin_tx)), STM_GPIO_PIN(obj->pin_tx), ENABLE);

  /* Configure USART Rx as alternate function push-pull  (software pull up)*/
  GPIO_ExternalPullUpConfig(get_GPIO_Port(STM_PORT(obj->pin_rx)), STM_GPIO_PIN(obj->pin_rx), ENABLE);

  USART_Init(obj->uart, obj->baudrate, obj->databits, obj->stopbits, obj->parity,
            (USART_Mode_TypeDef)(USART_Mode_Tx | USART_Mode_Rx));
  USART_ITConfig(obj->uart, USART_IT_RXNE, ENABLE);
  USART_Cmd(obj->uart, ENABLE);
#endif // STM8Lxx

  uart_handlers[obj->index] = obj->uart;
}

/**
* @brief  Function called to deinitialize the uart interface
* @param  serial_id : one of the defined serial interface
* @retval None
*/
void uart_deinit(serial_t *obj)
{
  if(obj == NULL)
    return;

#if defined(STM8Sxx)
#ifdef UART1
  if (obj->uart == UART1)
  {
    UART1_DeInit();
  }
#endif
#ifdef UART2
  if (obj->uart == UART2)
  {
    UART2_DeInit();
  }
#endif // UART2
#ifdef UART3
  if (obj->uart == UART3)
  {
    UART3_DeInit();
  }
#endif
#ifdef UART4
  if (obj->uart == UART4)
  {
    UART4_DeInit();
  }
#endif
#elif defined(STM8Lxx)
  USART_DeInit(obj->uart);
  if (obj->uart == USART1) {
    CLK_PeripheralClockConfig(CLK_Peripheral_USART1, DISABLE);
  } else if (obj->uart == USART2) {
    CLK_PeripheralClockConfig(CLK_Peripheral_USART2, DISABLE);
  } else if (obj->uart == USART3) {
    CLK_PeripheralClockConfig(CLK_Peripheral_USART3, DISABLE);
  }
#endif
}

/**
* @brief  write the data on the uart
* @param  obj : pointer to serial_t structure
* @param  data : byte to write
* @retval The number of bytes written
*/
uint8_t uart_write(serial_t *obj, uint8_t data)
{
  if(obj == NULL) {
    return 0;
  }

#if defined(STM8Sxx)
#ifdef UART1
  if (obj->uart == UART1)
  {
    if(obj->databits == UART_WORDLENGTH_9B) {
      UART1_SendData9(data);
    } else {
      UART1_SendData8(data);
    }
    while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET)
      ;
    return 1;
  }
#endif
#ifdef UART2
  if (obj->uart == UART2)
  {
    if(obj->databits == UART_WORDLENGTH_9B) {
      UART2_SendData9(data);
    } else {
      UART2_SendData8(data);
    }
    while (UART2_GetFlagStatus(UART2_FLAG_TXE) == RESET)
      ;
    return 1;
  }
#endif
#ifdef UART3
  if (obj->uart == UART3)
  {
    if(obj->databits == UART_WORDLENGTH_9B) {
      UART3_SendData9(data);
    } else {
      UART3_SendData8(data);
    }
    while (UART3_GetFlagStatus(UART3_FLAG_TXE) == RESET)
      ;
    return 1;
  }
#endif
#ifdef UART4
  if (obj->uart == UART4)
  {
    if(obj->databits == UART_WORDLENGTH_9B) {
      UART4_SendData9(data);
    } else {
      UART4_SendData8(data);
    }
    while (UART4_GetFlagStatus(UART4_FLAG_TXE) == RESET)
      ;
    return 1;
  }
#endif
#elif defined(STM8Lxx)
  if(obj->databits == UART_WORDLENGTH_9B) {
    USART_SendData9(obj->uart, data);
  } else {
    USART_SendData8(obj->uart, data);
  }
  while (USART_GetFlagStatus(obj->uart, USART_FLAG_TXE) == RESET)
    ;
  return 1;
#endif

  return 0;
}

/**
* @brief  Read receive byte from uart
* @param  obj : pointer to serial_t structure
* @retval last character received
*/
int uart_getc(serial_t *obj, unsigned char* c)
{
  if(obj == NULL) {
    return -1;
  }

#if defined(STM8Sxx)
#ifdef UART1
  if(obj->uart == UART1) {
    if(obj->databits == UART_WORDLENGTH_9B) {
      obj->recv = UART1_ReceiveData9();
    } else {
      obj->recv = UART1_ReceiveData8();
    }
  }
#endif
#ifdef UART2
  if(obj->uart == UART2) {
    if(obj->databits == UART_WORDLENGTH_9B) {
      obj->recv = UART2_ReceiveData9();
    } else {
      obj->recv = UART2_ReceiveData8();
    }
  }
#endif
#ifdef UART3
  if(obj->uart == UART3) {
    if(obj->databits == UART_WORDLENGTH_9B) {
      obj->recv = UART3_ReceiveData9();
    } else {
      obj->recv = UART3_ReceiveData8();
    }
  }
#endif
#ifdef UART4
  if(obj->uart == UART4) {
    if(obj->databits == UART_WORDLENGTH_9B) {
      obj->recv = UART4_ReceiveData9();
    } else {
      obj->recv = UART4_ReceiveData8();
    }
  }
#endif
#elif defined(STM8Lxx)
  if(obj->databits == UART_WORDLENGTH_9B) {
    obj->recv = USART_ReceiveData9(obj->uart);
  } else {
    obj->recv = USART_ReceiveData8(obj->uart);
  }
#else
  return -1;
#endif

  /* In the case that only 7 bits are used, remove the msb (when serial config is SERIAL_7Ex or 7Ox )*/
  if((obj->databits == UART_WORDLENGTH_8B) && (obj->parity != UART_PARITY_NONE))
  {
    obj->recv = (obj->recv & 0x7F);
  }

  *c = (unsigned char)(obj->recv);

  return 0;
}

/**
 * Begin asynchronous RX transfer (enable interrupt for data collecting)
 *
 * @param obj : pointer to serial_t structure
 * @param callback : function call at the end of reception
 * @retval none
 */
void uart_attach_rx_callback(serial_t *obj, void (*callback)(serial_t*))
{
  if(obj == NULL) {
    return;
  }

  rx_callback[obj->index] = callback;
  rx_callback_obj[obj->index] = obj;
}

/**
  * @brief  Return index of the serial handler
  * @param  UartHandle pointer on the uart reference
  * @retval index
  */
uint8_t uart_index(USART_TypeDef *huart)
{
  uint8_t i = 0;
  if(huart == NULL) {
    return UART_NUM;
  }

  for(i = 0; i < UART_NUM; i++) {
    if(huart == uart_handlers[i]) {
      break;
    }
  }

  return i;
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle pointer on the uart reference
  * @retval None
  */
void HAL_UART_RxCpltCallback(USART_TypeDef *huart)
{
  uint8_t index = uart_index(huart);

  if(index < UART_NUM) {
    rx_callback[index](rx_callback_obj[index]);
  }
}
#endif /* NO_HWSERIAL */

#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
