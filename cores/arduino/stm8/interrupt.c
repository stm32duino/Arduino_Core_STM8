/**
******************************************************************************
* @file    interrupt.c
* @author  WI6LABS
* @version V1.0.0
* @date    16-September-2016
* @brief   provide an interface to enable/disable interruptions
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
#include "interrupt.h"

#ifdef __cplusplus
extern "C"
{
#endif
/**
* @}
*/

/** @addtogroup stm8sxx_System_Private_TypesDefinitions
* @{
*/

typedef struct
{
  EXTI_Port_TypeDef EXTI_port;
  GPIO_Pin_TypeDef pin;
  uint8_t current_level;
#if defined(STM8Sxx)
  EXTI_Sensitivity_TypeDef EXTI_mode;
#elif defined(STM8Lxx)
  GPIO_TypeDef *port_IT;
  EXTI_Pin_TypeDef EXTI_pin;
  EXTI_Trigger_TypeDef EXTI_mode;
#endif
  void (*callback)(void);
} gpio_irq_conf_str;

/**
* @}
*/

/** @addtogroup stm8sxx_System_Private_Defines
* @{
*/

#define NB_EXTI ((uint8_t)16)

/*As this port are not usable with exti, use dummy values*/
#define EXTI_Port_A_Int ((uint8_t)0xFD)
#define EXTI_Port_C_Int ((uint8_t)0xFE)

/** @addtogroup stm8sxx_System_Private_Macros
* @{
*/

/**
* @}
*/

/** @addtogroup stm8sxx_System_Private_Variables
* @{
*/
static gpio_irq_conf_str gpio_irq_conf[NB_EXTI];

/**
* @}
*/

/** @addtogroup stm8sxx_System_Private_FunctionPrototypes
* @{
*/

static uint8_t get_pin_id(GPIO_TypeDef *port, GPIO_Pin_TypeDef pin, uint8_t init);

/**
* @}
*/

/**
* @brief  This function returns the pin ID function of the PIN definition
* @param  port : one of the gpio port
* @param  pin : one of the gpio pin
* @retval None
*/
static uint8_t get_pin_id(GPIO_TypeDef *port, GPIO_Pin_TypeDef pin, uint8_t init)
{
  uint8_t id = NC;
  uint8_t i = 0;
  EXTI_Port_TypeDef EXTI_port = 0xFF;

#if defined(STM8Sxx)
  if (port == GPIOA)
  {
    EXTI_port = EXTI_PORT_GPIOA;
  }
  else if (port == GPIOB)
  {
    EXTI_port = EXTI_PORT_GPIOB;
  }
  else if (port == GPIOC)
  {
    EXTI_port = EXTI_PORT_GPIOC;
  }
  else if (port == GPIOD)
  {
    EXTI_port = EXTI_PORT_GPIOD;
  }
  else if (port == GPIOE)
  {
    EXTI_port = EXTI_PORT_GPIOE;
  }
  else
  {
    return id;
  }
#elif defined(STM8Lxx)
  EXTI_Pin_TypeDef EXTI_pin;
 if (port == GPIOA)
  {
      EXTI_port = EXTI_Port_A_Int;
  }
  else if (port == GPIOB)
  {
    EXTI_port = EXTI_Port_B;
  }
  else if (port == GPIOC)
  {
    EXTI_port = EXTI_Port_C_Int;
  }
  else if (port == GPIOD)
  {
    EXTI_port = EXTI_Port_D;
  }
  else if (port == GPIOE)
  {
    EXTI_port = EXTI_Port_E;
  }
  else if (port == GPIOF)
  {
    EXTI_port = EXTI_Port_F;
  }
  else if (port == GPIOG)
  {
    EXTI_port = EXTI_Port_G;
  }
  else if (port == GPIOH)
  {
    EXTI_port = EXTI_Port_H;
  }
  else
  {
    return id;
  }

  if (pin == GPIO_Pin_0)
  {
    EXTI_pin = EXTI_Pin_0;
  }
  else if (pin == GPIO_Pin_1)
  {
    EXTI_pin = EXTI_Pin_1;
  }
  else if (pin == GPIO_Pin_2)
  {
    EXTI_pin = EXTI_Pin_2;
  }
  else if (pin == GPIO_Pin_3)
  {
    EXTI_pin = EXTI_Pin_3;
  }
  else if (pin == GPIO_Pin_4)
  {
    EXTI_pin = EXTI_Pin_4;
  }
  else if (pin == GPIO_Pin_5)
  {
    EXTI_pin = EXTI_Pin_5;
  }
  else if (pin == GPIO_Pin_6)
  {
    EXTI_pin = EXTI_Pin_6;
  }
  else if (pin == GPIO_Pin_7)
  {
    EXTI_pin = EXTI_Pin_7;
  }
  else
  {
    return id;
  }

#endif
  for (id = 0; id < NB_EXTI; id++)
  {
    if ((gpio_irq_conf[id].EXTI_port == EXTI_port) && (gpio_irq_conf[id].pin == pin))
    {
      id = i;
      break;
    }
    else if ((init == 1)&&(gpio_irq_conf[id].pin == 0))
    {
      gpio_irq_conf[id].EXTI_port = EXTI_port;
      gpio_irq_conf[id].pin = pin;
      gpio_irq_conf[id].callback = 0;
#ifdef STM8Lxx
      gpio_irq_conf[id].port_IT = port;
      gpio_irq_conf[id].EXTI_pin = EXTI_pin;
#endif
       break;
    }
  }
  return id;
}

/**
* @brief  This function enable the interruption on the selected port/pin
* @param  port : one of the gpio port
* @param  pin : one of the gpio pin
**@param  callback : callback to call when the interrupt falls
* @param  mode : one of the supported interrupt mode defined in stm32_hal_gpio
* @retval None
*/

#if defined(STM8Sxx)
void stm8_interrupt_enable(GPIO_TypeDef *port, GPIO_Pin_TypeDef pin,
                           void (*callback)(void),
                           EXTI_Sensitivity_TypeDef EXTI_mode)
#elif defined(STM8Lxx)
void stm8_interrupt_enable(GPIO_TypeDef *port, GPIO_Pin_TypeDef pin,
                         void (*callback)(void),
                           EXTI_Trigger_TypeDef EXTI_mode)
#endif
{
  GPIO_Mode_TypeDef GPIO_mode;

  uint8_t id = get_pin_id(port, pin, 1);
  if (id == NC)
  {
    return;
  }

  gpio_irq_conf[id].callback = callback;
  gpio_irq_conf[id].EXTI_mode = EXTI_mode;

  // Enable and set EXTI Interrupt
  disableInterrupts();
#if defined(STM8Sxx)
  GPIO_Init(port, pin, GPIO_MODE_IN_PU_IT);
  gpio_irq_conf[id].current_level = GPIO_ReadInputPin(port, pin);
  EXTI_SetExtIntSensitivity(gpio_irq_conf[id].EXTI_port, EXTI_SENSITIVITY_RISE_FALL/*EXTI_mode*/);

#elif defined(STM8Lxx)
  GPIO_Init(port, pin, GPIO_Mode_In_PU_IT);
  gpio_irq_conf[id].current_level = GPIO_ReadInputDataBit(port, pin);
  if(EXTI_GetPinSensitivity(gpio_irq_conf[id].EXTI_pin) != EXTI_Trigger_Rising_Falling) {
    EXTI_SetPinSensitivity(gpio_irq_conf[id].EXTI_pin, EXTI_Trigger_Rising_Falling);
  }
#endif

  enableInterrupts();
#if defined(STM8Sxx)
  gpio_irq_conf[id].current_level = GPIO_ReadInputPin(port, pin);
#elif defined(STM8Lxx)
  gpio_irq_conf[id].current_level = GPIO_ReadInputDataBit(port, pin);
#endif

  }

/**
* @brief  This function disable the interruption on the selected port/pin
* @param  port : one of the gpio port
* @param  pin : one of the gpio pin
* @retval None
*/
void stm8_interrupt_disable(GPIO_TypeDef *port, GPIO_Pin_TypeDef pin)
{
  GPIO_Mode_TypeDef GPIO_mode = (GPIO_Mode_TypeDef)0;

  uint8_t id = get_pin_id(port, pin, 0);
  if (id == NC)
  {
    return;
  }

  gpio_irq_conf[id].EXTI_port = (EXTI_Port_TypeDef)0;
  gpio_irq_conf[id].pin = (GPIO_Pin_TypeDef)0;
#if defined(STM8Sxx)
  gpio_irq_conf[id].EXTI_mode = (EXTI_Sensitivity_TypeDef)0;
#elif defined(STM8Lxx)
  gpio_irq_conf[id].EXTI_mode = (EXTI_Trigger_TypeDef)0;
#endif
  gpio_irq_conf[id].callback = 0;

#if defined(STM8Sxx)
  if ((port->CR1 & pin) == pin)
  {
    GPIO_mode = GPIO_MODE_IN_PU_NO_IT;
  }
  else
  {
    GPIO_mode = GPIO_MODE_IN_FL_NO_IT;
  }
#elif defined(STM8Lxx)
  if ((port->CR1 & pin) == pin)
  {
    GPIO_mode = GPIO_Mode_In_PU_No_IT;
  }
  else
  {
    GPIO_mode = GPIO_Mode_In_FL_No_IT;
  }
#endif
  GPIO_Init(port, pin, GPIO_mode);
}

/**
* @brief This function his called by EXTI interrupt
* @param  EXTI_port : EXTI_Port_TypeDef
* @retval None
*/
#if defined(STM8Sxx)
void GPIO_EXTI_Callback(EXTI_Port_TypeDef EXTI_port)
{
#endif
#if defined(STM8Lxx)
void GPIO_EXTI_Callback(EXTI_IT_TypeDef EXTI_port_pin)
{
#endif
  GPIO_TypeDef *port;
  GPIO_Pin_TypeDef pin;
  BitStatus status = RESET;
  uint8_t i = 0;
  uint8_t old_level = SET;
#if defined(STM8Sxx)
  if (EXTI_port == EXTI_PORT_GPIOA)
  {
    port = GPIOA;
  }
  else if (EXTI_port == EXTI_PORT_GPIOB)
  {
    port = GPIOB;
  }
  else if (EXTI_port == EXTI_PORT_GPIOC)
  {
    port = GPIOC;
  }
  else if (EXTI_port == EXTI_PORT_GPIOD)
  {
    port = GPIOD;
  }
  else if (EXTI_port == EXTI_PORT_GPIOE)
  {
    port = GPIOE;
  }
  else
  {
    return;
  }

  for (i = 0; i < NB_EXTI; i++)
  {
    if (gpio_irq_conf[i].EXTI_port == EXTI_port)
    {
      status = RESET;
      old_level = gpio_irq_conf[i].current_level;
      gpio_irq_conf[i].current_level = GPIO_ReadInputPin(port, gpio_irq_conf[i].pin);

      switch (gpio_irq_conf[i].EXTI_mode)
      {
      case EXTI_SENSITIVITY_FALL_LOW:
      case EXTI_SENSITIVITY_FALL_ONLY:
        if((old_level != RESET) && (gpio_irq_conf[i].current_level == RESET))
        {
          status = SET;
        }
        break;

      case EXTI_SENSITIVITY_RISE_ONLY:
        if((old_level == RESET) && (gpio_irq_conf[i].current_level != RESET))
        {
          status = SET;
        }
        break;

      case EXTI_SENSITIVITY_RISE_FALL:
        status = SET;
        break;

      default:
        break;
      }
      if ((gpio_irq_conf[i].callback != 0) && (status == SET))
      {
        gpio_irq_conf[i].callback();
      }
    }
  }
#endif
#if defined(STM8Lxx)
  EXTI_Port_TypeDef EXTI_port;
  EXTI_Pin_TypeDef EXTI_pin;
  EXTI_ClearITPendingBit(EXTI_port_pin);

  if (EXTI_port_pin == EXTI_IT_Pin0)
  {
    pin = GPIO_Pin_0;
    EXTI_pin = EXTI_Pin_0;
  }
  else if (EXTI_port_pin == EXTI_IT_Pin1)
  {
    pin = GPIO_Pin_1;
    EXTI_pin = EXTI_Pin_1;
  }
  else if (EXTI_port_pin == EXTI_IT_Pin2)
  {
    pin = GPIO_Pin_2;
    EXTI_pin = EXTI_Pin_2;
  }
  else if (EXTI_port_pin == EXTI_IT_Pin3)
  {
    pin = GPIO_Pin_3;
    EXTI_pin = EXTI_Pin_3;
  }
    else if (EXTI_port_pin == EXTI_IT_Pin4)
  {
    pin = GPIO_Pin_4;
    EXTI_pin = EXTI_Pin_4;
  }
  else if (EXTI_port_pin == EXTI_IT_Pin5)
  {
    pin = GPIO_Pin_5;
    EXTI_pin = EXTI_Pin_5;
  }
  else if (EXTI_port_pin == EXTI_IT_Pin6)
  {
    pin = GPIO_Pin_6;
    EXTI_pin = EXTI_Pin_6;
  }
  else if (EXTI_port_pin == EXTI_IT_Pin7)
  {
    pin = GPIO_Pin_7;
    EXTI_pin = EXTI_Pin_7;
  }
  else
  {
    return;
  }

  for (i = 0; i < NB_EXTI; i++)
  {
    if (gpio_irq_conf[i].EXTI_pin == EXTI_pin)
    {
      status = RESET;
      old_level = gpio_irq_conf[i].current_level;
      gpio_irq_conf[i].current_level = GPIO_ReadInputDataBit(gpio_irq_conf[i].port_IT, gpio_irq_conf[i].pin);

      switch (gpio_irq_conf[i].EXTI_mode)
      {
      case EXTI_Trigger_Falling_Low:
      case EXTI_Trigger_Falling:
        if((old_level != RESET) && (gpio_irq_conf[i].current_level == RESET))
        {
          status = SET;
        }
        break;

      case EXTI_Trigger_Rising:
        if((old_level == RESET) && (gpio_irq_conf[i].current_level != RESET))
        {
          status = SET;
        }
        break;

      case EXTI_Trigger_Rising_Falling:
        status = SET;
        break;

      default:
        break;
      }

      if ((gpio_irq_conf[i].callback != 0) && (status == SET))
      {
        gpio_irq_conf[i].callback();
      }
    }
  }
#endif
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
