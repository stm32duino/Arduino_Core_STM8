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
#include "interrupt.h"

#ifdef __cplusplus
 extern "C" {
#endif
/**
  * @}
  */

/** @addtogroup stm8sxx_System_Private_TypesDefinitions
  * @{
  */

typedef struct {
  EXTI_Port_TypeDef EXTI_port;
  GPIO_Pin_TypeDef pin;
  EXTI_Sensitivity_TypeDef EXTI_mode;
  void (*callback)(void);
}gpio_irq_conf_str;

/**
  * @}
  */

/** @addtogroup stm8sxx_System_Private_Defines
  * @{
  */

#define NB_EXTI   ((uint8_t)16)

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
  uint8_t id = 0xFF;
  uint8_t i = 0;
  EXTI_Port_TypeDef EXTI_port;

  if(port == GPIOA) {
      EXTI_port = EXTI_PORT_GPIOA;
  } else if(port == GPIOB) {
      EXTI_port = EXTI_PORT_GPIOB;
  } else if(port == GPIOC) {
      EXTI_port = EXTI_PORT_GPIOC;
  } else if(port == GPIOD) {
      EXTI_port = EXTI_PORT_GPIOD;
  } else if(port == GPIOE) {
      EXTI_port = EXTI_PORT_GPIOE;
  } else {
    return id;
  }

  for(id = 0; id < NB_EXTI; id++) {
    if((gpio_irq_conf[id].EXTI_port == EXTI_port) && (gpio_irq_conf[id].pin == pin)) {
      id = i;
      break;
    } else if(gpio_irq_conf[id].pin == 0) {
      if(init == 1) {
        gpio_irq_conf[id].EXTI_port = EXTI_port;
        gpio_irq_conf[id].pin = pin;
        gpio_irq_conf[id].callback = 0;
        id = i;
      }
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
void stm8_interrupt_enable(GPIO_TypeDef *port, GPIO_Pin_TypeDef pin,
                                  void (*callback)(void),
                                  EXTI_Sensitivity_TypeDef EXTI_mode)
{
  GPIO_Mode_TypeDef GPIO_mode;

  uint8_t id = get_pin_id(port, pin, 1);
  if(id == 0xFF) {
    return;
  }

  gpio_irq_conf[id].callback = callback;
  gpio_irq_conf[id].EXTI_mode = EXTI_mode;

  if((port->CR1 & pin) == pin) {
    GPIO_mode = GPIO_MODE_IN_PU_IT;
  } else {
    GPIO_mode = GPIO_MODE_IN_FL_IT;
  }

  // Enable and set EXTI Interrupt
  disableInterrupts();
  GPIO_Init(port, pin, GPIO_mode);
  EXTI_SetExtIntSensitivity(gpio_irq_conf[id].EXTI_port, EXTI_mode);
  enableInterrupts();
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
  if(id == 0xFF) {
    return;
  }

  gpio_irq_conf[id].EXTI_port = (EXTI_Port_TypeDef)0;
  gpio_irq_conf[id].pin = (GPIO_Pin_TypeDef)0;
  gpio_irq_conf[id].EXTI_mode = (EXTI_Sensitivity_TypeDef)0;
  gpio_irq_conf[id].callback = 0;

  if((port->CR1 & pin) == pin) {
    GPIO_mode = GPIO_MODE_IN_PU_NO_IT;
  } else {
    GPIO_mode = GPIO_MODE_IN_FL_NO_IT;
  }

  GPIO_Init(port, pin, GPIO_mode);
}

/**
  * @brief This function his called by EXTI interrupt
  * @param  EXTI_port : EXTI_Port_TypeDef
  * @retval None
  */
void GPIO_EXTI_Callback(EXTI_Port_TypeDef EXTI_port)
{
  GPIO_TypeDef *port;
  BitStatus status = RESET;
  uint8_t i = 0;

  if(EXTI_port == EXTI_PORT_GPIOA) {
      port = GPIOA;
  } else if(EXTI_port == EXTI_PORT_GPIOB) {
      port = GPIOB;
  } else if(EXTI_port == EXTI_PORT_GPIOC) {
      port = GPIOC;
  } else if(EXTI_port == EXTI_PORT_GPIOD) {
      port = GPIOD;
  } else if(EXTI_port == EXTI_PORT_GPIOE) {
      port = GPIOE;
  } else {
    return;
  }

  for(i = 0; i < NB_EXTI; i++) {
    if(gpio_irq_conf[i].EXTI_port == EXTI_port) {
      switch(gpio_irq_conf[i].EXTI_mode) {
        case EXTI_SENSITIVITY_FALL_LOW:
        case EXTI_SENSITIVITY_FALL_ONLY:
          if(GPIO_ReadInputPin(port, gpio_irq_conf[i].pin) == RESET) {
            status = SET;
          }
        break;

        case EXTI_SENSITIVITY_RISE_ONLY:
        if(GPIO_ReadInputPin(port, gpio_irq_conf[i].pin) != RESET) {
          status = SET;
        }
        break;

        case EXTI_SENSITIVITY_RISE_FALL:
          status = SET;
        break;

        default:
        break;
      }

      if((gpio_irq_conf[i].callback != 0) && (status == SET)) {
        gpio_irq_conf[i].callback();
      }
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
