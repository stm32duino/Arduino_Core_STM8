/**
  ******************************************************************************
  * @file    stm8s_conf.h
  * @author  MCD Application Team
  * @version V2.2.0
  * @date    30-September-2014
  * @brief   This file is used to configure the Library.
   ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM8S_CONF_H
#define __STM8S_CONF_H

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"

/* Uncomment the line below to enable peripheral header file inclusion */
#if defined(STM8S105) || defined(STM8S005) || defined(STM8S103) || defined(STM8S003) ||\
    defined(STM8S903) || defined (STM8AF626x) || defined (STM8AF622x)
  #include "stm8s_adc1.h"
  #define DRIVER_ACD1_MODULE_ENABLED
#endif /* (STM8S105) ||(STM8S103) || (STM8S903) || (STM8AF626x) || (STM8AF622x) */
#if defined(STM8S208) || defined(STM8S207) || defined(STM8S007) || defined (STM8AF52Ax) ||\
    defined (STM8AF62Ax)
  #include "stm8s_adc2.h"
  #define DRIVER_ACD2_MODULE_ENABLED
#endif /* (STM8S208) || (STM8S207) || (STM8AF62Ax) || (STM8AF52Ax) */
//#include "stm8s_awu.h"
//#define DRIVER_AWU_MODULE_ENABLED
//#include "stm8s_beep.h"
//#define DRIVER_BEEP_MODULE_ENABLED
#if defined (STM8S208) || defined (STM8AF52Ax)
 //#include "stm8s_can.h"
 //#define DRIVER_CAN_MODULE_ENABLED
#endif /* (STM8S208) || (STM8AF52Ax) */
#include "stm8s_clk.h"
#define DRIVER_CLK_MODULE_ENABLED
#include "stm8s_exti.h"
#define DRIVER_EXTI_MODULE_ENABLED
#include "stm8s_flash.h"
#define DRIVER_FLASH_MODULE_ENABLED
#include "stm8s_gpio.h"
#define DRIVER_GPIO_MODULE_ENABLED
#include "stm8s_i2c.h"
#define DRIVER_I2C_MODULE_ENABLED
//#include "stm8s_itc.h"
//#define DRIVER_ITC_MODULE_ENABLED
//#include "stm8s_iwdg.h"
//#define DRIVER_IWDG_MODULE_ENABLED
//#include "stm8s_rst.h"
//#define DRIVER_RST_MODULE_ENABLED
#include "stm8s_spi.h"
#define DRIVER_SPI_MODULE_ENABLED
#include "stm8s_tim1.h"
#define DRIVER_TIM1_MODULE_ENABLED
#if !defined(STM8S903) || !defined(STM8AF622x)
 #include "stm8s_tim2.h"
 #define DRIVER_TIM2_MODULE_ENABLED
#endif /* (STM8S903) || (STM8AF622x) */
#if defined(STM8S208) || defined(STM8S207) || defined(STM8S007) ||defined(STM8S105) ||\
    defined(STM8S005) ||  defined (STM8AF52Ax) || defined (STM8AF62Ax) || defined (STM8AF626x)
 #include "stm8s_tim3.h"
 #define DRIVER_TIM3_MODULE_ENABLED
#endif /* (STM8S208) ||defined(STM8S207) || defined(STM8S007) ||defined(STM8S105) */
#if !defined(STM8S903) || !defined(STM8AF622x)
 #include "stm8s_tim4.h"
 #define DRIVER_TIM4_MODULE_ENABLED
#endif /* (STM8S903) || (STM8AF622x) */
#if defined(STM8S903) || defined(STM8AF622x)
 #include "stm8s_tim5.h"
 #define DRIVER_TIM5_MODULE_ENABLED
 #include "stm8s_tim6.h"
 #define DRIVER_TIM6_MODULE_ENABLED
#endif  /* (STM8S903) || (STM8AF622x) */
#if defined(STM8S208) ||defined(STM8S207) || defined(STM8S007) ||defined(STM8S103) ||\
    defined(STM8S003) || defined(STM8S903) || defined (STM8AF52Ax) || defined (STM8AF62Ax)
 #include "stm8s_uart1.h"
 #define DRIVER_UART1_MODULE_ENABLED
#endif /* (STM8S208) || (STM8S207) || (STM8S103) || (STM8S903) || (STM8AF52Ax) || (STM8AF62Ax) */
#if defined(STM8S105) || defined(STM8S005) ||  defined (STM8AF626x)
 #include "stm8s_uart2.h"
 #define DRIVER_UART2_MODULE_ENABLED
#endif /* (STM8S105) || (STM8AF626x) */
#if defined(STM8S208) ||defined(STM8S207) || defined(STM8S007) || defined (STM8AF52Ax) ||\
    defined (STM8AF62Ax)
 #include "stm8s_uart3.h"
 #define DRIVER_UART3_MODULE_ENABLED
#endif /* STM8S208 || STM8S207 || STM8AF52Ax || STM8AF62Ax */
#if defined(STM8AF622x)
 #include "stm8s_uart4.h"
 #define DRIVER_UART4_MODULE_ENABLED
#endif /* (STM8AF622x) */
//#include "stm8s_wwdg.h"
//#define DRIVER_WWDG_MODULE_ENABLED

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Uncomment the line below to expanse the "assert_param" macro in the
   Standard Peripheral Library drivers code */
//#define USE_FULL_ASSERT    (1)

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT

/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param expr: If expr is false, it calls assert_failed function
  *   which reports the name of the source file and the source
  *   line number of the call that failed.
  *   If expr is true, it returns no value.
  * @retval : None
  */
#define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
void assert_failed(uint8_t* file, uint32_t line);
#else
#define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

#endif /* __STM8S_CONF_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
