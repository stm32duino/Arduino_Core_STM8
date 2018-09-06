/**
  ******************************************************************************
  * @file    timer.c
  * @author  WI6LABS
  * @version V1.0.0
  * @date
  * @brief   provide timer services
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
#include "timer.h"
#include "digital_io.h"
#include "pins_arduino.h"
#ifdef __cplusplus
extern "C"
{
#endif

  static void HAL_TIMx_PeriodElapsedCallback(timer_id_e timer_id);
  static timer_id_e getInactiveTimer(void);
  static timer_id_e isPinAssociateToTimer(PinName pin);

  /// @brief timer caracteristics

  static timer_conf_t g_timer_config[NB_TIMER_MANAGED] = {
      {.timer_id = TIM1_E,
       .irqHandle = 0,
       .irqHandleOC = 0,
       .timer_mode = TIMER_OTHER,
       .prescalerLimit = bits_16,
       .toggle_pin = {.pin = NC},
       .configured = 0},
      {.timer_id = TIM2_E,
       .irqHandle = 0,
       .irqHandleOC = 0,
       .timer_mode = TIMER_OTHER,
       .prescalerLimit = bits_16,
       .toggle_pin = {.pin = NC},
       .configured = 0},
      {.timer_id = TIM3_E,
       .irqHandle = 0,
       .irqHandleOC = 0,
       .timer_mode = TIMER_OTHER,
       .prescalerLimit = bits_16,
       .toggle_pin = {.pin = NC},
       .configured = 0},
      {.timer_id = TIM4_E,
       .irqHandle = 0,
       .timer_mode = TIMER_RESERVED,
       .prescalerLimit = bits_8,
       .toggle_pin = {.pin = NC},
       .configured = 0}};

  /**
  * @brief  Find the first timer not used
  * @param  none
  * @retval The id of the first timer not used if not the number of id.
  */
  static timer_id_e getInactiveTimer(void)
  {
    uint8_t timer_id = NB_TIMER_MANAGED;
    uint8_t i = 0;
    for (i = 0; i < NB_TIMER_MANAGED; i++)
    {
      if ((g_timer_config[i].configured == 0) &&
          (g_timer_config[i].timer_mode == TIMER_OTHER))
      {
        timer_id = i;
        break;
      }
    }

    return (timer_id_e)timer_id;
  }

  /**
  * @brief  Search the timer associate to a pin
  * @param  port : port pointer
  * @param  pin : pin number
  * @retval The timer id
  */
  static timer_id_e isPinAssociateToTimer(PinName pin)
  {
    void *peripheral = pinmap_peripheral(pin, PinMap_PWM);
    if (peripheral == TIM1)
      return TIM1_E;
    if (peripheral == TIM2)
      return TIM2_E;
    if (peripheral == TIM3)
      return TIM3_E;
    if (peripheral == TIM4)
      return TIM4_E;
  }

  /**
  * @brief  This function will set the timer to the required value
  * @param  timer_id : timer_id_e
  * @param  period : Timer period in milliseconds
  * @param  prescaler : clock divider
  * @retval None
  */
  void TimerHandleInit(timer_id_e timer_id, uint16_t period, uint16_t prescaler)
  {
    if (timer_id >= NB_TIMER_MANAGED)
      return;
    switch (timer_id)
    {
#if defined(STM8Sxx)
    case TIM1_E:
      TIM1_DeInit();
      CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER1, ENABLE);
      TIM1_TimeBaseInit(prescaler, TIM1_COUNTERMODE_UP, period, 0);
      TIM1_ITConfig(TIM1_IT_UPDATE, ENABLE);
      TIM1_Cmd(ENABLE);
      g_timer_config[TIM1_E].configured = 1;
      break;

    case TIM2_E:
      TIM2_DeInit();
      if (IS_TIM2_PRESCALER_OK((TIM2_Prescaler_TypeDef)prescaler) == 0)
      {
        return;
      }
      CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER2, ENABLE);
      TIM2_TimeBaseInit((TIM2_Prescaler_TypeDef)prescaler, period);
      TIM2_ITConfig(TIM2_IT_UPDATE, ENABLE);
      TIM2_Cmd(ENABLE);
      g_timer_config[TIM2_E].configured = 1;
      break;

    case TIM3_E:
      TIM3_DeInit();
      if (IS_TIM3_PRESCALER_OK((TIM3_Prescaler_TypeDef)prescaler) == 0)
      {
        return;
      }
      CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER3, ENABLE);
      TIM3_TimeBaseInit((TIM3_Prescaler_TypeDef)prescaler, period);
      TIM3_ITConfig(TIM3_IT_UPDATE, ENABLE);
      TIM3_Cmd(ENABLE);
      g_timer_config[TIM3_E].configured = 1;
      break;

    case TIM4_E:
      TIM4_DeInit();
      if (IS_TIM4_PRESCALER_OK(prescaler) == 0)
      {
        return;
      }
      CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER4, ENABLE);
      TIM4_TimeBaseInit((TIM4_Prescaler_TypeDef)prescaler, (uint8_t)period);
      TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
      TIM4_Cmd(ENABLE);
      g_timer_config[TIM4_E].configured = 1;
      break;

    default:
      break;
#endif
#if defined(STM8Lxx)
    case TIM1_E:
      TIM1_DeInit();
      CLK_PeripheralClockConfig(CLK_Peripheral_TIM1, ENABLE);
      TIM1_TimeBaseInit(prescaler, TIM1_CounterMode_Up, period, 0);
      TIM1_ITConfig(TIM1_IT_Update, ENABLE);
      TIM1_Cmd(ENABLE);
      g_timer_config[TIM1_E].configured = 1;
      break;

    case TIM2_E:
      TIM2_DeInit();
      if (IS_TIM2_PRESCALER((TIM2_Prescaler_TypeDef)prescaler) == 0)
      {
        return;
      }
      CLK_PeripheralClockConfig(CLK_Peripheral_TIM2, ENABLE);
      TIM2_TimeBaseInit((TIM2_Prescaler_TypeDef)prescaler, TIM2_CounterMode_Up, period);
      TIM2_ITConfig(TIM2_IT_Update, ENABLE);
      TIM2_Cmd(ENABLE);
      g_timer_config[TIM2_E].configured = 1;
      break;

    case TIM3_E:
      TIM3_DeInit();
      if (IS_TIM3_PRESCALER((TIM3_Prescaler_TypeDef)prescaler) == 0)
      {
        return;
      }
      CLK_PeripheralClockConfig(CLK_Peripheral_TIM3, ENABLE);
      TIM3_TimeBaseInit((TIM3_Prescaler_TypeDef)prescaler, TIM3_CounterMode_Up, period);
      TIM3_ITConfig(TIM3_IT_Update, ENABLE);
      TIM3_Cmd(ENABLE);
      g_timer_config[TIM3_E].configured = 1;
      break;

    case TIM4_E:
      TIM4_DeInit();
      if (IS_TIM4_Prescaler(prescaler) == 0)
      {
        return;
      }
      CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, ENABLE);
      TIM4_TimeBaseInit((TIM4_Prescaler_TypeDef)prescaler, (uint8_t)period);
      TIM4_ITConfig(TIM4_IT_Update, ENABLE);
      TIM4_Cmd(ENABLE);
      g_timer_config[TIM4_E].configured = 1;
      break;

    default:
      break;
#endif
    }

    enableInterrupts();
  }

  /**
  * @brief  This function will reset the timer
  * @param  timer_id : timer_id_e
  * @retval None
  */
  void TimerHandleDeinit(timer_id_e timer_id)
  {
    switch (timer_id)
    {
    case TIM1_E:
      TIM1_DeInit();
      g_timer_config[TIM1_E].configured = 0;
      break;

    case TIM2_E:
      TIM2_DeInit();
      g_timer_config[TIM2_E].configured = 0;
      break;

    case TIM3_E:
      TIM3_DeInit();
      g_timer_config[TIM3_E].configured = 0;
      break;

    case TIM4_E:
      TIM4_DeInit();
      g_timer_config[TIM4_E].configured = 0;
      break;

    default:
      break;
    }
  }

  /**
  * @brief  Period elapsed callback in non-blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
  void TIM_PeriodElapsedCallback(timer_id_e timer_id)
  {
    if (timer_id >= NB_TIMER_MANAGED)
    {
      return;
    }

    if (g_timer_config[timer_id].irqHandle != 0)
    {
      g_timer_config[timer_id].irqHandle(timer_id);
    }
  }

  /**
  * @brief  Attached an interrupt handler
  * @param  timer_id : id of the timer
  * @param  irqHandle : interrupt handler
  * @retval none
  */
  void attachIntHandle(timer_id_e timer_id, void (*irqHandle)(timer_id_e))
  {
    if (timer_id < NB_TIMER_MANAGED)
    {
      g_timer_config[timer_id].irqHandle = irqHandle;
    }
  }

  /**
  * @brief  This function will set the timer to generate pulse in interrupt mode
            with a particular duty cycle
  * @param  timer_id : timer_id_e
  * @param  period : timer period in microseconds
  * @param  pulseWidth : pulse width in microseconds
  * @param  irqHandle : interrupt routine to call
  * @retval None
  */
  void TimerPulseInit(timer_id_e timer_id, uint16_t period, uint16_t pulseWidth,
                      void (*irqHandle)(timer_id_e, uint8_t))
  {
    if (timer_id >= NB_TIMER_MANAGED)
      return;
    if (g_timer_config[timer_id].configured == 1)
      return;
#if defined(STM8Sxx)
    switch (timer_id)
    {
    case TIM1_E:
      TIM1_DeInit();
      CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER1, ENABLE);
      TIM1_TimeBaseInit((uint16_t)(CLK_GetClockFreq() / 1000000) - 1,
                        TIM1_COUNTERMODE_UP, period, 0);
      TIM1_OC1Init(TIM1_OCMODE_TIMING, TIM1_OUTPUTSTATE_DISABLE,
                   TIM1_OUTPUTNSTATE_DISABLE, pulseWidth, TIM1_OCPOLARITY_LOW,
                   TIM1_OCNPOLARITY_LOW, TIM1_OCIDLESTATE_RESET,
                   TIM1_OCNIDLESTATE_RESET);
      TIM1_ITConfig(TIM1_IT_CC1, ENABLE);
      TIM1_Cmd(ENABLE);
      break;

    case TIM2_E:
      TIM2_DeInit();
      CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER2, ENABLE);
      TIM2_TimeBaseInit(TIM2_PRESCALER_16, period);
      TIM2_OC1Init(TIM2_OCMODE_TIMING, TIM2_OUTPUTSTATE_DISABLE, pulseWidth,
                   TIM2_OCPOLARITY_LOW);
      TIM2_ITConfig(TIM2_IT_CC1, ENABLE);
      TIM2_Cmd(ENABLE);
      g_timer_config[TIM2_E].configured = 1;
      g_timer_config[TIM2_E].irqHandleOC = irqHandle;
      break;

    case TIM3_E:
      TIM3_DeInit();
      CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER3, ENABLE);
      TIM3_TimeBaseInit(TIM3_PRESCALER_16, period);
      TIM3_OC1Init(TIM3_OCMODE_TIMING, TIM3_OUTPUTSTATE_DISABLE, pulseWidth,
                   TIM3_OCPOLARITY_LOW);
      TIM3_ITConfig(TIM3_IT_CC1, ENABLE);
      TIM3_Cmd(ENABLE);
      g_timer_config[TIM3_E].configured = 1;
      g_timer_config[TIM3_E].irqHandleOC = irqHandle;
      break;

    default:
      return;
      break;
    }
#elif defined(STM8Lxx)
  switch (timer_id)
  {
  case TIM1_E:
    TIM1_DeInit();
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM1, ENABLE);
    TIM1_TimeBaseInit((uint16_t)(CLK_GetClockFreq() / 1000000) - 1,
                      TIM1_CounterMode_Up, period, 0);
    TIM1_OC1Init(TIM1_OCMode_Timing, TIM1_OutputState_Disable,
                 TIM1_OutputNState_Disable, pulseWidth, TIM1_OCPolarity_Low,
                 TIM1_OCNPolarity_Low, TIM1_OCIdleState_Reset,
                 TIM1_OCNIdleState_Reset);
    TIM1_ITConfig(TIM1_IT_CC1, ENABLE);
    TIM1_Cmd(ENABLE);
    break;

  case TIM2_E:
    TIM2_DeInit();
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM2, ENABLE);
    TIM2_TimeBaseInit(TIM2_Prescaler_16, TIM2_CounterMode_Up, period);
    TIM2_OC1Init(TIM2_OCMode_Timing, TIM2_OutputState_Disable, pulseWidth,
                 TIM2_OCPolarity_Low, TIM2_OCIdleState_Reset);
    TIM2_ITConfig(TIM2_IT_CC1, ENABLE);
    TIM2_Cmd(ENABLE);
    g_timer_config[TIM2_E].configured = 1;
    g_timer_config[TIM2_E].irqHandleOC = irqHandle;
    break;

  case TIM3_E:
    TIM3_DeInit();
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM3, ENABLE);
    TIM3_TimeBaseInit(TIM2_Prescaler_16, TIM3_CounterMode_Up, period);
    TIM3_OC1Init(TIM3_OCMode_Timing, TIM3_OutputState_Disable, pulseWidth,
                 TIM3_OCPolarity_Low, TIM3_OCIdleState_Reset);
    TIM3_ITConfig(TIM3_IT_CC1, ENABLE);
    TIM3_Cmd(ENABLE);
    g_timer_config[TIM3_E].configured = 1;
    g_timer_config[TIM3_E].irqHandleOC = irqHandle;
    break;

  default:
    return;
    break;
  }
#endif
    g_timer_config[TIM1_E].configured = 1;
    g_timer_config[TIM1_E].irqHandleOC = irqHandle;
  }

  /**
  * @brief  This function will reset the pulse generation
  * @param  timer_id : timer_id_e
  * @retval None
  */
  void TimerPulseDeinit(timer_id_e timer_id)
  {
    if (timer_id >= NB_TIMER_MANAGED)
      return;

    switch (timer_id)
    {
    case TIM1_E:
      TIM1_DeInit();
      break;

    case TIM2_E:
      TIM2_DeInit();
      break;

    case TIM3_E:
      TIM3_DeInit();
      break;

    default:
      return;
      break;
    }

    g_timer_config[timer_id].irqHandleOC = 0;
    g_timer_config[TIM1_E].configured = 0;
  }

  /**
  * @brief  Output Compare callback in non-blocking mode
  * @param  timer_id : timer_id_e
  * @retval None
  */
  void TIM_OC_DelayElapsedCallback(timer_id_e timer_id)
  {
    uint8_t channel = 0;

    if (NB_TIMER_MANAGED == timer_id)
    {
      return;
    }

    if (g_timer_config[timer_id].irqHandleOC != 0)
    {
      switch (timer_id)
      {
#if defined(STM8Sxx)
      case TIM1_E:
        channel = TIM1_CHANNEL_1;
        break;

      case TIM2_E:
        channel = TIM2_CHANNEL_1;
        break;

      case TIM3_E:
        channel = TIM3_CHANNEL_1;
        break;
#elif defined(STM8Lxx)
    case TIM1_E:
      channel = TIM1_Channel_1;
      break;

    case TIM2_E:
      channel = TIM2_Channel_1;
      break;

    case TIM3_E:
      channel = TIM3_Channel_1;
      break;
#endif
      default:
        return;
        break;
      }
      g_timer_config[timer_id].irqHandleOC(timer_id, channel);
    }
  }

  /**
  * @brief  Period elapsed callback in non-blocking mode
  * @param  timer_id : id of the timer
  * @retval None
  */
  static void HAL_TIMx_PeriodElapsedCallback(timer_id_e timer_id)
  {

    PinName p = digitalPinToPinName(g_timer_config[timer_id].toggle_pin.pin);
    GPIO_TypeDef *port = get_GPIO_Port(STM_PORT(p));
    if (port != NULL)
    {
      if (g_timer_config[timer_id].toggle_pin.count != 0)
      {
        if (g_timer_config[timer_id].toggle_pin.count > 0)
          g_timer_config[timer_id].toggle_pin.count--;
        g_timer_config[timer_id].toggle_pin.state = (g_timer_config[timer_id].toggle_pin.state == 0) ? 1 : 0;
        digital_io_write(port, STM_GPIO_PIN(p), g_timer_config[timer_id].toggle_pin.state);
      }
      else
      {
        digital_io_write(port, STM_GPIO_PIN(p), 0);
      }
    }
  }

  /**
  * @brief  This function will set the tone timer to the required value and
  *         configure the pin to toggle.
  * @param  port : pointer to GPIO_TypeDef
  * @param  pin : pin number to toggle
  * @param  frequency : toggle frequency (in hertz)
  * @param  duration : toggle time
  * @retval None
  */
  void TimerPinInit(uint8_t pin, uint32_t frequency, uint32_t duration)
  {
    uint8_t end = 0;
    uint32_t timClkFreq = 0;
    //  TIMER_TONE FREQ is Twice frequency
    uint32_t timFreq = 2 * frequency;
    uint32_t prescaler = 1;
    uint32_t period = 0;
    uint32_t scale = 0;
    timer_id_e timer_id;
#if defined(STM8Sxx)
    timer_id = TIM2_E; // isPinAssociateToTimer(port,pin);
#elif defined(STM8Lxx)
    timer_id = TIM1_E; // isPinAssociateToTimer(port,pin);
#endif

    if (frequency > MAX_FREQ)
      return;

    g_timer_config[timer_id].toggle_pin.pin = pin;
    g_timer_config[timer_id].toggle_pin.state = 0;

    if (frequency == 0)
    {
      TimerPinDeinit(pin);
      return;
    }

    //Calculate the toggle count
    if (duration > 0)
    {
      g_timer_config[timer_id].toggle_pin.count = ((timFreq * duration) / 1000);
    }
    else
    {
      g_timer_config[timer_id].toggle_pin.count = -1;
    }
#if defined(STM8Sxx)
    digital_io_init(digitalPinToPinName(pin), GPIO_MODE_OUT_PP_LOW_FAST, 0);
#endif
#if defined(STM8Lxx)
  digital_io_init(digitalPinToPinName(pin), GPIO_Mode_Out_PP_Low_Fast, 0);
#endif
    timClkFreq = CLK_GetClockFreq();

    //Do this Once
    scale = timClkFreq / timFreq;
    while (end == 0)
    {
      period = ((uint32_t)(scale / prescaler)) - 1;
      if ((period >= 0xFFFF) && (prescaler < 0xFFFF))
        prescaler++; //prescaler *= 2;
      else
        end = 1;
    }

    if ((period < 0xFFFF) && (prescaler < 0xFFFF))
    {
      g_timer_config[timer_id].irqHandle = HAL_TIMx_PeriodElapsedCallback;
      TimerHandleInit(timer_id, period, prescaler - 1);
    }
    else
    {
      TimerHandleDeinit(timer_id);
    }
  }

  /**
  * @brief  This function will reset the tone timer
  * @param  port : pointer to port
  * @param  pin : pin number to toggle
  * @retval None
  */
  void TimerPinDeinit(uint8_t pin)
  {
    timer_id_e timer_id = isPinAssociateToTimer(pin);

    if (timer_id < NB_TIMER_MANAGED)
    {
      TimerHandleDeinit(timer_id);
      g_timer_config[timer_id].toggle_pin.pin = 0;
      g_timer_config[timer_id].toggle_pin.count = 0;
      g_timer_config[timer_id].toggle_pin.state = 0;
    }
  }

  /**
  * @brief  Get the counter value.
  * @param  timer_id : id of the timer
  * @retval Counter value
  */
  uint16_t getTimerCounter(timer_id_e timer_id)
  {
    uint16_t data = 0;

    if (timer_id < NB_TIMER_MANAGED)
    {
      switch (timer_id)
      {
      case TIM1_E:
        data = TIM1_GetCounter();
        break;

      case TIM2_E:
        data = TIM2_GetCounter();
        break;

      case TIM3_E:
        data = TIM3_GetCounter();
        break;

      case TIM4_E:
        data = (uint16_t)TIM4_GetCounter();
        break;

      default:
        break;
      }

      return data;
    }
    else
    {
      return 0;
    }
  }

  /**
  * @brief  Set the counter value.
  * @param  timer_id : id of the timer
  * @param  value : counter value
  * @retval None
  */
  void setTimerCounter(timer_id_e timer_id, uint16_t value)
  {
    if (timer_id < NB_TIMER_MANAGED)
    {
      switch (timer_id)
      {
      case TIM1_E:
        TIM1_SetCounter(value);
        break;

      case TIM2_E:
        TIM2_SetCounter(value);
        break;

      case TIM3_E:
        TIM3_SetCounter(value);
        break;

      case TIM4_E:
        TIM4_SetCounter((uint8_t)value);
        break;

      default:
        break;
      }
    }
  }

  /**
  * @brief  Set the TIM Capture Compare Register value.
  * @param  timer_id : id of the timer
  * @param  channel : TIM Channels to be configured.
  * @param  value : register new register.
  * @retval None
  */
  void setCCRRegister(timer_id_e timer_id, uint8_t channel, uint16_t value)
  {
    if (timer_id < NB_TIMER_MANAGED)
    {
#if defined(STM8Sxx)
      switch (timer_id)
      {
      case TIM1_E:
        if (channel == TIM1_CHANNEL_1)
        {
          TIM1_SetCompare1(value);
        }
        else if (channel == TIM1_CHANNEL_2)
        {
          TIM1_SetCompare2(value);
        }
        else if (channel == TIM1_CHANNEL_3)
        {
          TIM1_SetCompare3(value);
        }
        else if (channel == TIM1_CHANNEL_4)
        {
          TIM1_SetCompare4(value);
        }
        break;

      case TIM2_E:
        if (channel == TIM2_CHANNEL_1)
        {
          TIM2_SetCompare1(value);
        }
        else if (channel == TIM2_CHANNEL_2)
        {
          TIM2_SetCompare2(value);
        }
        else if (channel == TIM2_CHANNEL_3)
        {
          TIM2_SetCompare3(value);
        }
        break;

      case TIM3_E:
        if (channel == TIM3_CHANNEL_1)
        {
          TIM3_SetCompare1(value);
        }
        else if (channel == TIM3_CHANNEL_2)
        {
          TIM3_SetCompare2(value);
        }
        break;

      default:
        break;
      }
#endif
#if defined(STM8Lxx)
    switch (timer_id)
    {
    case TIM1_E:
      if (channel == TIM1_Channel_1)
      {
        TIM1_SetCompare1(value);
      }
      else if (channel == TIM1_Channel_2)
      {
        TIM1_SetCompare2(value);
      }
      else if (channel == TIM1_Channel_3)
      {
        TIM1_SetCompare3(value);
      }
      else if (channel == TIM1_Channel_4)
      {
        TIM1_SetCompare4(value);
      }
      break;

    case TIM2_E:
      if (channel == TIM2_Channel_1)
      {
        TIM2_SetCompare1(value);
      }
      else if (channel == TIM2_Channel_2)
      {
        TIM2_SetCompare2(value);
      }
      break;

    case TIM3_E:
      if (channel == TIM3_Channel_1)
      {
        TIM3_SetCompare1(value);
      }
      else if (channel == TIM3_Channel_2)
      {
        TIM3_SetCompare2(value);
      }
      break;

    default:
      break;
    }
#endif
    }
  }

#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
