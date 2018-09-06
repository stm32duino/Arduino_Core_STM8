/**
  ******************************************************************************
  * @file    timer.h
  * @author  WI6LABS
  * @version V1.0.0
  * @date    16-September-2016
  * @brief   Header for timer module
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
#ifndef __TIMER_H
#define __TIMER_H

/* Includes ------------------------------------------------------------------*/
#include "stm8_def.h"
#include "PeripheralPins.h"

#ifdef __cplusplus
extern "C"
{
#endif

  /* Exported types ------------------------------------------------------------*/
  typedef enum
  {
    TIM1_E = 0,
    TIM2_E,
    TIM3_E,
    TIM4_E,
    NB_TIMER_MANAGED
  } timer_id_e;

  typedef enum
  {
    bits_8 = 0xFF,
    bits_16 = 0xFFFF
  } timer_prescaler_limit;

  typedef enum
  {
    TIMER_PWM = 0x00000000,
    TIMER_RESERVED = 0x00000001,
    TIMER_OTHER = 0x00000002
  } timer_mode_e;

  typedef struct
  {
    uint8_t pin;
    int32_t count;
    uint8_t state;
  } timer_toggle_pin_config_str;

  /// @brief defines the global attributes of the TIMER
  typedef struct
  {

    timer_id_e timer_id;
    void (*irqHandle)(timer_id_e);
    void (*irqHandleOC)(timer_id_e, uint8_t);
    timer_mode_e timer_mode;
    timer_prescaler_limit prescalerLimit;
    timer_toggle_pin_config_str toggle_pin;
    uint8_t configured;
  } timer_conf_t;

/* Exported constants --------------------------------------------------------*/
#define MAX_FREQ 65535

  /* Exported macro ------------------------------------------------------------*/
  /* Exported functions ------------------------------------------------------- */
  void TimerHandleInit(timer_id_e timer_id, uint16_t period, uint16_t prescaler);
  void TimerHandleDeinit(timer_id_e timer_id);
  void attachIntHandle(timer_id_e timer_id, void (*irqHandle)(timer_id_e));
  void TIM_PeriodElapsedCallback(timer_id_e timer_id);

  void TimerPinInit(uint8_t _pin, uint32_t frequency, uint32_t duration);
  void TimerPinDeinit(uint8_t _pin);

  void TimerPulseInit(timer_id_e timer_id, uint16_t period, uint16_t pulseWidth, void (*irqHandle)(timer_id_e, uint8_t));
  void TimerPulseDeinit(timer_id_e timer_id);
  void TIM_OC_DelayElapsedCallback(timer_id_e timer_id);

  uint16_t getTimerCounter(timer_id_e timer_id);
  void setTimerCounter(timer_id_e timer_id, uint16_t value);
  void setCCRRegister(timer_id_e timer_id, uint8_t channel, uint16_t value);
#ifdef __cplusplus
}
#endif

#endif /* __TIMER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
