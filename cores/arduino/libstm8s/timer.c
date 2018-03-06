/**
  ******************************************************************************
  * @file    timer.c
  * @author  WI6LABS
  * @version V1.0.0
  * @date    16-September-2016
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
/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm8sxx_system
  * @{
  */

/** @addtogroup stm8sxx_System_Private_Includes
  * @{
  */
#include "stm8s.h"
#include "timer.h"
#include "digital_io.h"

#ifdef __cplusplus
 extern "C" {
#endif

/**
  * @}
  */

/** @addtogroup stm8sxx_System_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @addtogroup stm8sxx_System_Private_Defines
  * @{
  */

/**
  * @}
  */

/** @addtogroup stm8sxx_System_Private_FunctionPrototypes
* @{
*/

static void HAL_TIMx_PeriodElapsedCallback(timer_id_e timer_id);
static timer_id_e getInactiveTimer(void);
static timer_id_e isPinAssociateToTimer(GPIO_TypeDef *port, uint32_t pin);

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

/// @brief timer caracteristics

static timer_conf_t g_timer_config[NB_TIMER_MANAGED] = {
  {
    .timer_id = TIM1_E,
    .irqHandle = 0,
    .irqHandleOC = 0,
    .timer_mode = TIMER_OTHER,
    .prescalerLimit = bits_16,
    .toggle_pin = { .port = 0 },
    .configured = 0
  },
  {
    .timer_id = TIM2_E,
    .irqHandle = 0,
    .irqHandleOC = 0,
    .timer_mode = TIMER_OTHER,
    .prescalerLimit = bits_16,
    .toggle_pin = { .port = 0 },
    .configured = 0
  },
  {
    .timer_id = TIM3_E,
    .irqHandle = 0,
    .irqHandleOC = 0,
    .timer_mode = TIMER_OTHER,
    .prescalerLimit = bits_16,
    .toggle_pin = { .port = 0 },
    .configured = 0
  },
  {
    .timer_id = TIM4_E,
    .irqHandle = 0,
    .timer_mode = TIMER_RESERVED,
    .prescalerLimit = bits_8,
    .toggle_pin = { .port = 0 },
    .configured = 0
  }
};


/**
  * @}
  */

/**
  * @brief  Find the first timer not used
  * @param  none
  * @retval The id of the first timer not used if not the number of id.
  */
static timer_id_e getInactiveTimer(void)
{
  uint8_t timer_id = NB_TIMER_MANAGED;
  uint8_t i = 0;
  for(i = 0; i < NB_TIMER_MANAGED; i++) {
    if((g_timer_config[i].configured == 0) &&
       (g_timer_config[i].timer_mode == TIMER_OTHER)){
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
static timer_id_e isPinAssociateToTimer(GPIO_TypeDef *port, uint32_t pin)
{
  uint8_t i = 0;
  uint8_t timer_id = NB_TIMER_MANAGED;

  for(i = 0; i < NB_TIMER_MANAGED; i++) {
    if((g_timer_config[i].toggle_pin.port == port) &&
       (g_timer_config[i].toggle_pin.pin == pin))  {
       timer_id = i;
       break;
    }
  }

  return (timer_id_e)timer_id;
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
  if(timer_id >= NB_TIMER_MANAGED) return;

  switch(timer_id) {
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
      if(IS_TIM2_PRESCALER_OK((TIM2_Prescaler_TypeDef)prescaler) == 0) {return;}
      CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER2, ENABLE);
      TIM2_TimeBaseInit((TIM2_Prescaler_TypeDef)prescaler, period);
      TIM2_ITConfig(TIM2_IT_UPDATE, ENABLE);
      TIM2_Cmd(ENABLE);
      g_timer_config[TIM2_E].configured = 1;
      break;

    case TIM3_E:
      TIM3_DeInit();
      if(IS_TIM3_PRESCALER_OK((TIM3_Prescaler_TypeDef)prescaler) == 0) {return;}
      CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER3, ENABLE);
      TIM3_TimeBaseInit((TIM3_Prescaler_TypeDef)prescaler, period);
      TIM3_ITConfig(TIM3_IT_UPDATE, ENABLE);
      TIM3_Cmd(ENABLE);
      g_timer_config[TIM3_E].configured = 1;
      break;

    case TIM4_E:
      TIM4_DeInit();
      if(IS_TIM4_PRESCALER_OK(prescaler) == 0) {return;}
      CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER4, ENABLE);
      TIM4_TimeBaseInit((TIM4_Prescaler_TypeDef)prescaler, (uint8_t)period);
      TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
      TIM4_Cmd(ENABLE);
      g_timer_config[TIM4_E].configured = 1;
      break;

    default:
      break;
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
  switch(timer_id) {
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
  if(timer_id >= NB_TIMER_MANAGED) {
    return;
  }

  if(g_timer_config[timer_id].irqHandle != 0) {
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
  if(timer_id < NB_TIMER_MANAGED) {
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
  if(timer_id >= NB_TIMER_MANAGED) return;
  if(g_timer_config[timer_id].configured == 1) return;

  switch(timer_id) {
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
  if(timer_id >= NB_TIMER_MANAGED) return;

  switch(timer_id) {
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

  if(NB_TIMER_MANAGED == timer_id) {return;}

  if(g_timer_config[timer_id].irqHandleOC != 0) {
    switch(timer_id) {
      case TIM1_E:
      channel = TIM1_CHANNEL_1;
      break;

      case TIM2_E:
      channel = TIM2_CHANNEL_1;
      break;

      case TIM3_E:
      channel = TIM3_CHANNEL_1;
      break;

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
  if(g_timer_config[timer_id].toggle_pin.port != 0) {
    if(g_timer_config[timer_id].toggle_pin.count > 0){
      g_timer_config[timer_id].toggle_pin.count--;

      if(g_timer_config[timer_id].toggle_pin.state == RESET) {
        g_timer_config[timer_id].toggle_pin.state = SET;
        digital_io_write(g_timer_config[timer_id].toggle_pin.port,
                         (GPIO_Pin_TypeDef)g_timer_config[timer_id].toggle_pin.pin, SET);
      }
      else {
        g_timer_config[timer_id].toggle_pin.state = RESET;
        digital_io_write(g_timer_config[timer_id].toggle_pin.port,
                         (GPIO_Pin_TypeDef)g_timer_config[timer_id].toggle_pin.pin, RESET);
      }
    }
    else if(g_timer_config[timer_id].toggle_pin.count == -1) {
      if(g_timer_config[timer_id].toggle_pin.state == RESET) {
        g_timer_config[timer_id].toggle_pin.state = SET;
        digital_io_write(g_timer_config[timer_id].toggle_pin.port,
                         (GPIO_Pin_TypeDef)g_timer_config[timer_id].toggle_pin.pin, SET);
      }
      else {
        g_timer_config[timer_id].toggle_pin.state = RESET;
        digital_io_write(g_timer_config[timer_id].toggle_pin.port,
                         (GPIO_Pin_TypeDef)g_timer_config[timer_id].toggle_pin.pin, RESET);
      }
    }
    else {
      digital_io_write(g_timer_config[timer_id].toggle_pin.port,
                       (GPIO_Pin_TypeDef)g_timer_config[timer_id].toggle_pin.pin, RESET);
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
void TimerPinInit(GPIO_TypeDef *port, uint32_t pin, uint32_t frequency, uint32_t duration)
{
  uint8_t end = 0;
  uint32_t prescaler = 1;
  uint32_t period = 0;
  timer_id_e timer_id;

  timer_id = isPinAssociateToTimer(port,pin);

  if(timer_id == NB_TIMER_MANAGED) {
    timer_id = getInactiveTimer();
    if(timer_id == NB_TIMER_MANAGED)
      return;
  }

  if(frequency > MAX_FREQ)
    return;

  g_timer_config[timer_id].toggle_pin.port = port;
  g_timer_config[timer_id].toggle_pin.pin = pin;
  g_timer_config[timer_id].toggle_pin.state = 0;

  //Calculate the toggle count
  if (duration > 0) {
    g_timer_config[timer_id].toggle_pin.count = ((frequency * duration) / 1000) * 2;
  }
  else {
    g_timer_config[timer_id].toggle_pin.count = -1;
  }

  digital_io_init(port, (GPIO_Pin_TypeDef)pin, GPIO_MODE_OUT_PP_LOW_FAST);

  while(end == 0) {
    period = ((uint32_t)(CLK_GetClockFreq() / frequency / prescaler) / 2) - 1;

    if((period >= g_timer_config[timer_id].prescalerLimit)
        && (prescaler < g_timer_config[timer_id].prescalerLimit))
      prescaler++; //prescaler *= 2;
    else
      end = 1;
  }

  if((period < g_timer_config[timer_id].prescalerLimit)
      && (prescaler < g_timer_config[timer_id].prescalerLimit)) {
    g_timer_config[timer_id].irqHandle = HAL_TIMx_PeriodElapsedCallback;
    TimerHandleInit(timer_id, period, prescaler-1);
  }
  else {
    TimerHandleDeinit(timer_id);
  }
}

/**
  * @brief  This function will reset the tone timer
  * @param  port : pointer to port
  * @param  pin : pin number to toggle
  * @retval None
  */
void TimerPinDeinit(GPIO_TypeDef *port, uint32_t pin)
{
  timer_id_e timer_id = isPinAssociateToTimer(port,pin);

  if(timer_id < NB_TIMER_MANAGED) {
    TimerHandleDeinit(timer_id);
    g_timer_config[timer_id].toggle_pin.port = 0;
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

  if(timer_id < NB_TIMER_MANAGED) {
    switch(timer_id) {
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
        data =  (uint16_t)TIM4_GetCounter();
        break;

      default:
        break;
    }

    return data;
  }
  else {
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
  if(timer_id < NB_TIMER_MANAGED) {
    switch(timer_id) {
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
  if(timer_id < NB_TIMER_MANAGED) {
    switch(timer_id) {
      case TIM1_E:
        if(channel == TIM1_CHANNEL_1) {
          TIM1_SetCompare1(value);
        } else if(channel == TIM1_CHANNEL_2) {
          TIM1_SetCompare2(value);
        } else if(channel == TIM1_CHANNEL_3) {
          TIM1_SetCompare3(value);
        } else if(channel == TIM1_CHANNEL_4) {
          TIM1_SetCompare4(value);
        }
        break;

      case TIM2_E:
        if(channel == TIM2_CHANNEL_1) {
          TIM2_SetCompare1(value);
        } else if(channel == TIM2_CHANNEL_2) {
          TIM2_SetCompare2(value);
        } else if(channel == TIM2_CHANNEL_3) {
          TIM2_SetCompare3(value);
        }
        break;

      case TIM3_E:
        if(channel == TIM3_CHANNEL_1) {
          TIM3_SetCompare1(value);
        } else if(channel == TIM3_CHANNEL_2) {
          TIM3_SetCompare2(value);
        }
        break;

      default:
        break;
    }
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
