/**
  ******************************************************************************
  * @file    clock.c
  * @author  WI6LABS
  * @version V1.0.0
  * @date
  * @brief   provide clock services for time purpose
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

#include "stm8_def.h"
#include "clock.h"
#include "timer.h"

#ifdef __cplusplus
extern "C"
{
#endif

  static volatile uint32_t g_current_ms = 1;
  static volatile uint8_t g_count_ms = 0;

  static void TimerDelay_PeriodElapsedCallback(timer_id_e timer_id);

  /**
  * @brief  Function called wto read the current micro second
  * @param  None
  * @retval None
  */
  uint32_t GetCurrentMicro(void)
  {
    uint32_t us;
    uint8_t cc;

    cc = (uint8_t) _asm("push cc\n sim\n pop a");
    us = (uint32_t)TIM4_GetCounter() + (g_count_ms * 249);
    us += g_current_ms * 1000;
    _asm("push a\n pop cc", cc);
    return us;
  }

  /**
  * @brief  Function called wto read the current millisecond
  * @param  None
  * @retval None
  */
  uint32_t GetCurrentMilli(void)
  {
    uint32_t ms;
    uint8_t cc;

    cc = (uint8_t) _asm("push cc\n sim\n pop a");
    ms = g_current_ms;
    _asm("push a\n pop cc", cc);
    return ms;
  }

  /**
  * @brief  This function configures the source of the time base.
  * @param  None
  * @retval None
  */
  void InitDelayTimer(void)
  {
    /*configure TIMER to get the microsecond precision time.
  1MhZ is applied to the timer. Timer will count until 1ms ends*/
    /* Currently the timer count until 250 us because TIM4 is a 8 bits timer*/
    attachIntHandle(TIM4_E, TimerDelay_PeriodElapsedCallback);
#if defined(STM8Sxx)
    TimerHandleInit(TIM4_E, 250, TIM4_PRESCALER_16);
#elif defined(STM8Lxx)
    TimerHandleInit(TIM4_E, 250, TIM4_Prescaler_16);
#endif
  }

  /**
  * @brief  TIM5 period elapsed callback in non blocking mode
  * @param  None
  * @retval None
  */
  static void TimerDelay_PeriodElapsedCallback(timer_id_e timer_id)
  {
    g_count_ms++;
    if (g_count_ms >= 4)
    {
      g_current_ms++;
      g_count_ms = 0;
    }
  }

#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
