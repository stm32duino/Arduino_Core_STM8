/**
  ******************************************************************************
  * @file    digital_io.c
  * @author  WI6LABS
  * @version V1.0.0
  * @date    16-September-2016
  * @brief   Provide an interface to configure hw ios
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

/** @addtogroup stm8sxx_System_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup stm8sxx_System_Private_Variables
  * @{
  */

/**
  * @}
  */

/** @addtogroup stm8sxx_System_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */


/**
  * @brief  This function initialize the IO
  * @param  port : one of the gpio port
  * @param  pin : one of the gpio pin
  * @param  mode : one of the gpio mode (cf stm8s_gpio.h)
  * @retval None
  */
void digital_io_init(GPIO_TypeDef  *port, GPIO_Pin_TypeDef pin, GPIO_Mode_TypeDef mode)
{
  GPIO_Init(port, pin, mode);
}

/**
  * @brief  This function set a value to an IO
  * @param  port : one of the gpio port
  * @param  pin : one of the gpio pin
  * @param  val : 0 to set to low, any other value to set to high
  * @retval None
  */
void digital_io_write(GPIO_TypeDef  *port, GPIO_Pin_TypeDef pin, uint32_t val)
{
  if(val) {
    GPIO_WriteHigh(port, pin);
  } else {
    GPIO_WriteLow(port, pin);
  }
}


/**
  * @brief  This function set a value to an IO
  * @param  port : one of the gpio port
  * @param  pin : one of the gpio pin
  * @retval The pin state (LOW or HIGH)
  */
uint32_t digital_io_read(GPIO_TypeDef  *port, GPIO_Pin_TypeDef pin)
{
  return (uint32_t)GPIO_ReadInputPin(port, pin);
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
