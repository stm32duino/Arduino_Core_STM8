/*
 *******************************************************************************
 * Copyright (c) 2018, STMicroelectronics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
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
 *******************************************************************************
 */

#ifndef _PINAF_STM8LXX_H
#define _PINAF_STM8LXX_H
#ifdef STM8Lxx

#ifdef __cplusplus
extern "C" {
#endif

enum {
  AFIO_NONE,
  AFIO_SPI1_FULL_ENABLE,
  AFIO_SPI1_FULL_DISABLE,
  AFIO_SPI1_PORTF_ENABLE,
  AFIO_SPI1_PORTF_DISABLE,
  AFIO_SPI2_ENABLE,
  AFIO_SPI2_DISABLE,
  AFIO_USART1_PORTA_ENABLE,
  AFIO_USART1_PORTA_DISABLE,
  AFIO_USART1_PORTC_ENABLE,
  AFIO_USART1_PORTC_DISABLE,
  AFIO_USART1_CLK_ENABLE,
  AFIO_USART1_CLK_DISABLE,
  AFIO_USART3_PORTF_ENABLE,
  AFIO_USART3_PORTF_DISABLE,
  AFIO_USART3_Clk_ENABLE,
  AFIO_USART3_Clk_DISABLE,
  AFIO_TIM2_CH1_ENABLE,
  AFIO_TIM2_CH1_DISABLE,
  AFIO_TIM2_CH2_ENABLE,
  AFIO_TIM2_CH2_DISABLE,
  AFIO_TIM2_TRIGPortA_ENABLE,
  AFIO_TIM2_TRIGPortA_DISABLE,
  AFIO_TIM2_TRIGLSE_ENABLE,
  AFIO_TIM2_TRIGLSE_DISABLE,
  AFIO_TIM2_TIM3_BKIN_ENABLE,
  AFIO_TIM2_TIM3_BKIN_DISABLE,
  AFIO_TIM3_CH1_ENABLE,
  AFIO_TIM3_CH1_DISABLE,
  AFIO_TIM3_CH2_ENABLE,
  AFIO_TIM3_CH2_DISABLE,
  AFIO_TIM3_TRIGPortA_ENABLE,
  AFIO_TIM3_TRIGPortA_DISABLE,
  AFIO_TIM3_TRIGPortG_ENABLE,
  AFIO_TIM3_TRIGPortG_DISABLE,
  AFIO_TIM3_TRIGLSE_ENABLE,
  AFIO_TIM3_TRIGLSE_DISABLE,
  AFIO_ADC1_EXTRIG_ENABLE,
  AFIO_ADC1_EXTRIG_DISABLE,
  AFIO_CCO_ENABLE,
  AFIO_CCO_DISABLE
};

static inline void pin_Set8LAFPin(uint16_t afnum)
{
  switch (afnum)
  {
  case AFIO_SPI1_FULL_ENABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_SPI1Full, ENABLE);
      break;
  case AFIO_SPI1_FULL_DISABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_SPI1Full, DISABLE);
      break;
  case AFIO_SPI1_PORTF_ENABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_SPI1PortF, ENABLE);
      break;
  case AFIO_SPI1_PORTF_DISABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_SPI1PortF, DISABLE);
      break;
  case AFIO_SPI2_ENABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_SPI2Full, ENABLE);
      break;
  case AFIO_SPI2_DISABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_SPI2Full, DISABLE);
      break;
  case AFIO_USART1_PORTA_ENABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_USART1TxRxPortA, ENABLE);
      break;
  case AFIO_USART1_PORTA_DISABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_USART1TxRxPortA, DISABLE);
      break;
  case AFIO_USART1_PORTC_ENABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_USART1TxRxPortC, ENABLE);
      break;
  case AFIO_USART1_PORTC_DISABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_USART1TxRxPortC, DISABLE);
      break;
  case AFIO_USART1_CLK_ENABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_USART1Clk, ENABLE);
      break;
  case AFIO_USART1_CLK_DISABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_USART1Clk, DISABLE);
      break;
  case AFIO_USART3_PORTF_ENABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_USART3TxRxPortF, ENABLE);
      break;
  case AFIO_USART3_PORTF_DISABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_USART3TxRxPortF, DISABLE);
      break;
  case AFIO_USART3_Clk_ENABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_USART3Clk, ENABLE);
      break;
  case AFIO_USART3_Clk_DISABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_USART3Clk, DISABLE);
      break;
  case AFIO_TIM2_CH1_ENABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_TIM2Channel1, ENABLE);
      break;
  case AFIO_TIM2_CH1_DISABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_TIM2Channel1, DISABLE);
      break;
  case AFIO_TIM2_CH2_ENABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_TIM2Channel2, ENABLE);
      break;
  case AFIO_TIM2_CH2_DISABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_TIM2Channel2, DISABLE);
      break;
  case AFIO_TIM2_TRIGPortA_ENABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_TIM2TRIGPortA, ENABLE);
      break;
  case AFIO_TIM2_TRIGPortA_DISABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_TIM2TRIGPortA, DISABLE);
      break;
  case AFIO_TIM2_TRIGLSE_ENABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_TIM2TRIGLSE, ENABLE);
      break;
  case AFIO_TIM2_TRIGLSE_DISABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_TIM2TRIGLSE, DISABLE);
      break;
  case AFIO_TIM2_TIM3_BKIN_ENABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_TIM23BKIN, ENABLE);
      break;
  case AFIO_TIM2_TIM3_BKIN_DISABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_TIM23BKIN, DISABLE);
      break;
  case AFIO_TIM3_CH1_ENABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_TIM3Channel1, ENABLE);
      break;
  case AFIO_TIM3_CH1_DISABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_TIM3Channel1, DISABLE);
      break;
  case AFIO_TIM3_CH2_ENABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_TIM3Channel2, ENABLE);
      break;
  case AFIO_TIM3_CH2_DISABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_TIM3Channel2, DISABLE);
      break;
  case AFIO_TIM3_TRIGPortA_ENABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_TIM3TRIGPortA, ENABLE);
      break;
  case AFIO_TIM3_TRIGPortA_DISABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_TIM3TRIGPortA, DISABLE);
      break;
  case AFIO_TIM3_TRIGPortG_ENABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_TIM3TRIGPortG, ENABLE);
      break;
  case AFIO_TIM3_TRIGPortG_DISABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_TIM3TRIGPortG, DISABLE);
      break;
  case AFIO_TIM3_TRIGLSE_ENABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_TIM3TRIGLSE, ENABLE);
      break;
  case AFIO_TIM3_TRIGLSE_DISABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_TIM3TRIGLSE, DISABLE);
      break;
  case AFIO_ADC1_EXTRIG_ENABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_ADC1ExtTRIG1, ENABLE);
      break;
  case AFIO_ADC1_EXTRIG_DISABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_ADC1ExtTRIG1, DISABLE);
      break;
  case AFIO_CCO_ENABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_CCO, ENABLE);
      break;
  case AFIO_CCO_DISABLE:
      SYSCFG_REMAPPinConfig(REMAP_Pin_CCO, DISABLE);
      break;
  default:
  case AFIO_NONE:
     break;
  }
}

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus
#endif //STM8Lxx
#endif //_PINAF_STM8LXX_H
