/**
  ******************************************************************************
  * @file    digital_io.h
  * @author  WI6LABS
  * @version V1.0.0
  * @date    16-September-2016
  * @brief   Header for digital_io module
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
#ifndef __DIGITAL_IO_H
#define __DIGITAL_IO_H

/* Includes ------------------------------------------------------------------*/
//#include "stm8s.h"
#include "stm8_def.h"
#include "PeripheralPins.h"
#ifdef __cplusplus
extern "C"
{
#endif

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#if defined(STM8Sxx)
#define GPIO_Mode_In_FL_No_IT GPIO_MODE_IN_FL_NO_IT
#define GPIO_Mode_In_PU_No_IT GPIO_MODE_IN_PU_NO_IT
#define GPIO_Mode_Out_PP_Low_Slow GPIO_MODE_OUT_PP_LOW_SLOW
#define GPIO_Mode_Out_PP_Low_Fast GPIO_MODE_OUT_PP_LOW_FAST
#endif
  /* Exported functions ------------------------------------------------------- */
  void digital_io_init(PinName pin, GPIO_Mode_TypeDef mode, uint32_t pull);
  void digital_io_write(GPIO_TypeDef *port, GPIO_Pin_TypeDef pin, uint32_t val);
  uint32_t digital_io_read(GPIO_TypeDef *port, GPIO_Pin_TypeDef pin);
#ifdef __cplusplus
}
#endif

#endif /* __DIGITAL_IO_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
