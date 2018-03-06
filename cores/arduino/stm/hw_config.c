/**
  ******************************************************************************
  * @file    hw_config.c
  * @author  WI6LABS
  * @version V1.0.0
  * @date    16-September-2016
  * @brief   provide some hw interface for the Arduino interface
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
#include "hw_config.h"
#include "stm8s.h"
#include "clock.h"

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

static void SystemClock_Config(void);

/**
  * @}
  */

/** @addtogroup stm8sxx_System_Private_Functions
  * @{
  */
/**
  * @brief  System Clock Configuration
  *         The HSI is used at 16MHz : clock divider = CLK_PRESCALER_HSIDIV1
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  CLK_DeInit();

  CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);
  CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1);

  CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, CLK_SOURCE_HSE, DISABLE, CLK_CURRENTCLOCKSTATE_DISABLE);

  //CLK_HSECmd(ENABLE);
}

/**
  * @brief  This function performs the global init of the system (HAL, IOs...)
  * @param  None
  * @retval None
  */
void hw_config_init(void)
{
  // Configure the system clock
  SystemClock_Config();

  InitDelayTimer();
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
