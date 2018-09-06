/**
  ******************************************************************************
  * @file    uart.h
  * @author  WI6LABS
  * @version V1.0.0
  * @date    08-10-2018
  * @brief   Header for uart module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_H
#define __UART_H

/* Includes ------------------------------------------------------------------*/
#include "stm8_def.h"
#include "variant.h"

#ifdef __cplusplus
extern "C"
{
#endif

#if defined(NO_HWSERIAL)
#define serial_t void*
#else
#if !defined(HWSERIAL_NONE) && defined(SERIAL_UART_INSTANCE)
#if SERIAL_UART_INSTANCE == 1
#define ENABLE_HWSERIAL1
#if !defined(Serial)
#define Serial Serial1
#define serialEvent serialEvent1
#endif
#elif SERIAL_UART_INSTANCE == 2
#define ENABLE_HWSERIAL2
#if !defined(Serial)
#define Serial Serial2
#define serialEvent serialEvent2
#endif
#elif SERIAL_UART_INSTANCE == 3
#define ENABLE_HWSERIAL3
#if !defined(Serial)
#define Serial Serial3
#define serialEvent serialEvent3
#endif
#elif SERIAL_UART_INSTANCE == 4
#define ENABLE_HWSERIAL4
#if !defined(Serial)
#define Serial Serial4
#define serialEvent serialEvent4
#endif
#else
#if !defined(Serial)
#warning "No generic 'Serial' defined!"
#endif
#endif /* SERIAL_UART_INSTANCE == x */
#endif /* !HWSERIAL_NONE && SERIAL_UART_INSTANCE */

#if defined(ENABLE_HWSERIAL1)
#if defined(UART1) || defined(USART1_BASE)
#define HAVE_HWSERIAL1
#endif
#endif
#if defined(ENABLE_HWSERIAL2)
#if defined(UART2) || defined(USART2_BASE)
#define HAVE_HWSERIAL2
#endif
#endif
#if defined(ENABLE_HWSERIAL3)
#if defined(UART3) || defined(USART3_BASE)
#define HAVE_HWSERIAL3
#endif
#endif
#if defined(ENABLE_HWSERIAL4)
#if defined(UART4)
#define HAVE_HWSERIAL4
#endif
#endif

/* Exported constants --------------------------------------------------------*/
#if defined(STM8Sxx)
#define USART1 UART1
#if defined(UART2)
#define USART2 UART2
#endif
#if defined(UART3)
#define USART3 UART3
#endif
#if defined(UART4)
#define USART4 UART4
#endif
#endif

#if defined(STM8Sxx)
#define USART_TypeDef void
#endif

#if defined(STM8Lxx)
#define UART_PARITY_ODD USART_Parity_Odd
#define UART_PARITY_EVEN USART_Parity_Even
#define UART_PARITY_NONE USART_Parity_No

#define UART_STOPBITS_2 USART_StopBits_2
#define UART_STOPBITS_1 USART_StopBits_1

#define UART_WORDLENGTH_8B USART_WordLength_8b
#define UART_WORDLENGTH_9B USART_WordLength_9b
#else //STM8Sxx
#if defined(UART1)
#define UART_PARITY_ODD UART1_PARITY_ODD
#define UART_PARITY_EVEN UART1_PARITY_EVEN
#define UART_PARITY_NONE UART1_PARITY_NO

#define UART_STOPBITS_2 UART1_STOPBITS_2
#define UART_STOPBITS_1 UART1_STOPBITS_1

#define UART_WORDLENGTH_8B UART1_WORDLENGTH_8D
#define UART_WORDLENGTH_9B UART1_WORDLENGTH_9D
#elif defined(UART2)
#define UART_PARITY_ODD UART2_PARITY_ODD
#define UART_PARITY_EVEN UART2_PARITY_EVEN
#define UART_PARITY_NONE UART2_PARITY_NO

#define UART_STOPBITS_2 UART2_STOPBITS_2
#define UART_STOPBITS_1 UART2_STOPBITS_1

#define UART_WORDLENGTH_8B UART2_WORDLENGTH_8D
#define UART_WORDLENGTH_9B UART2_WORDLENGTH_9D
#elif defined(UART3)
#define UART_PARITY_ODD UART3_PARITY_ODD
#define UART_PARITY_EVEN UART3_PARITY_EVEN
#define UART_PARITY_NONE UART3_PARITY_NO

#define UART_STOPBITS_2 UART3_STOPBITS_2
#define UART_STOPBITS_1 UART3_STOPBITS_1

#define UART_WORDLENGTH_8B UART3_WORDLENGTH_8D
#define UART_WORDLENGTH_9B UART3_WORDLENGTH_9D
#elif defined(UART4)
#define UART_PARITY_ODD UART4_PARITY_ODD
#define UART_PARITY_EVEN UART4_PARITY_EVEN
#define UART_PARITY_NONE UART4_PARITY_NO

#define UART_STOPBITS_2 UART4_STOPBITS_2
#define UART_STOPBITS_1 UART4_STOPBITS_1

#define UART_WORDLENGTH_8B UART4_WORDLENGTH_8D
#define UART_WORDLENGTH_9B UART4_WORDLENGTH_9D
#endif
#endif

/* Exported types ------------------------------------------------------------*/
typedef struct serial_s serial_t;

struct serial_s
{
  USART_TypeDef *uart;
  uint8_t index;
  uint8_t recv;
  uint32_t baudrate;
  uint32_t databits;
  uint32_t stopbits;
  uint32_t parity;
  PinName pin_tx;
  PinName pin_rx;
  uint8_t *rx_buff;
  volatile uint16_t rx_head;
  uint16_t rx_tail;
};

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void uart_init(serial_t *obj);
void uart_deinit(serial_t *obj);
uint8_t uart_write(serial_t *obj, uint8_t data);
int uart_getc(serial_t *obj, unsigned char* c);
void uart_attach_rx_callback(serial_t *obj, void (*callback)(serial_t*));

void HAL_UART_RxCpltCallback(USART_TypeDef *huart);
#endif /* NO_HWSERIAL */
#ifdef __cplusplus
}
#endif

#endif /* __UART_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
