/**
  ******************************************************************************
  * @file    analog.c
  * @author  WI6LABS
  * @version V1.0.0
  * @date    16-September-2016
  * @brief   provide analog services (ADC + PWM)
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
#include "analog.h"

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

//Give details about the analog pins.
analog_config_str g_analog_config[NB_ANALOG_CHANNELS] = {
  {
    .port = GPIOB,
    .pin = GPIO_PIN_3,
    .adcChannel = ADC1_CHANNEL_3
  },
  {
    .port = GPIOB,
    .pin = GPIO_PIN_4,
    .adcChannel = ADC1_CHANNEL_4
  },
  {
    .port = GPIOB,
    .pin = GPIO_PIN_6,
    .adcChannel = ADC1_CHANNEL_6
  },
  {
    .port = GPIOB,
    .pin = GPIO_PIN_7,
    .adcChannel = ADC1_CHANNEL_7
  },
  {
    .port = GPIOC,
    .pin = GPIO_PIN_2,
    .timer_id = TIM1_E,
    .timChannel = TIM1_CHANNEL_2,
  },
  {
    .port = GPIOC,
    .pin = GPIO_PIN_4,
    .timer_id = TIM1_E,
    .timChannel = TIM1_CHANNEL_4,
  },
  {
    .port = GPIOD,
    .pin = GPIO_PIN_3,
    .timer_id = TIM2_E,
    .timChannel = TIM2_CHANNEL_2,
  },
  {
    .port = GPIOD,
    .pin = GPIO_PIN_4,
    .timer_id = TIM2_E,
    .timChannel = TIM2_CHANNEL_1,
  },
  {
    .port = GPIOE,
    .pin = GPIO_PIN_6,
    .adcChannel = ADC1_CHANNEL_9
  },
  {
    .port = GPIOE,
    .pin = GPIO_PIN_7,
    .adcChannel = ADC1_CHANNEL_8
  }
};

/**
  * @}
  */

/** @addtogroup stm8sxx_System_Private_FunctionPrototypes
  * @{
  */
static int8_t get_analog_instance(GPIO_TypeDef  *port, uint32_t pin);


/**
  * @brief  This function will return the corresponding adc configuration id
  * @param  port : the gpio port to use
  * @param  pin : the gpio pin to use
  * @retval None
  */
static int8_t get_analog_instance(GPIO_TypeDef  *port, uint32_t pin)
{
  int8_t i;

  for(i = 0; i < NB_ANALOG_CHANNELS ; i++) {
    if((g_analog_config[i].port == port)&&(g_analog_config[i].pin == pin)) {
      return i;
    }
  }
  return -1;
}

////////////////////////// ADC INTERFACE FUNCTIONS /////////////////////////////

/**
  * @brief  This function will set the ADC to the required value
  * @param  port : the gpio port to use
  * @param  pin : the gpio pin to use
  * @param  do_init : if set to 1 the initialization of the adc is done
  * @retval the value of the adc
  */
uint16_t adc_read_value(GPIO_TypeDef  *port, uint32_t pin, uint8_t do_init)
{
  //find the instance in the global
  int8_t id = get_analog_instance(port, pin);
  if(id < 0 ) {return 0;}

  __IO uint16_t uhADCxConvertedValue = 0;

  ADC1_DeInit();
  ADC1_Init(ADC1_CONVERSIONMODE_SINGLE, g_analog_config[id].adcChannel,
            ADC1_PRESSEL_FCPU_D2, ADC1_EXTTRIG_TIM, DISABLE, ADC1_ALIGN_RIGHT,
            ADC1_SCHMITTTRIG_CHANNEL12, DISABLE);

  ADC1_StartConversion();

  uhADCxConvertedValue = ADC1_GetConversionValue();

  ADC1_Cmd(DISABLE);

  return uhADCxConvertedValue;
}

////////////////////////// PWM INTERFACE FUNCTIONS /////////////////////////////

/**
  * @brief  This function will set the PWM to the required value
  * @param  port : the gpio port to use
  * @param  pin : the gpio pin to use
  * @param  clock_freq : frequency of the tim clock
  * @param  period : period of the tim counter
  * @param  value : the value to push on the PWM output
  * @param  do_init : if set to 1 the initialization of the PWM is done
  * @retval None
  */
void pwm_start(GPIO_TypeDef  *port, uint32_t pin, uint32_t clock_freq,
                uint32_t period, uint32_t value, uint8_t do_init)
{
  //find the instance in the global
  int8_t id = get_analog_instance(port, pin);
  if(id < 0) return;

  switch(g_analog_config[id].timer_id) {
    case TIM1_E:
      if(do_init == 1) {
        //TIM1_DeInit();
        TIM1_TimeBaseInit((uint16_t)(CLK_GetClockFreq() / clock_freq) - 1, TIM1_COUNTERMODE_UP, period, 0);
      }

      if(g_analog_config[id].timChannel == TIM1_CHANNEL_1) {
        TIM1_OC1Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE,
                      TIM1_OUTPUTNSTATE_DISABLE, value, TIM1_OCPOLARITY_LOW,
                      TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_SET,
                      TIM1_OCNIDLESTATE_RESET);
      } else if(g_analog_config[id].timChannel == TIM1_CHANNEL_2) {
        TIM1_OC2Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE,
                      TIM1_OUTPUTNSTATE_DISABLE, value, TIM1_OCPOLARITY_LOW,
                      TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_SET,
                      TIM1_OCNIDLESTATE_RESET);
      } else if(g_analog_config[id].timChannel == TIM1_CHANNEL_3) {
        TIM1_OC3Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE,
                      TIM1_OUTPUTNSTATE_DISABLE, value, TIM1_OCPOLARITY_LOW,
                      TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_SET,
                      TIM1_OCNIDLESTATE_RESET);
      } else if(g_analog_config[id].timChannel == TIM1_CHANNEL_4) {
        TIM1_OC4Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, value,
                      TIM1_OCPOLARITY_LOW, TIM1_OCIDLESTATE_SET);
      } else {return;}

      TIM1_Cmd(ENABLE);
      TIM1_CtrlPWMOutputs(ENABLE);
      break;

      case TIM2_E:
        if(do_init == 1) {
          //TIM2_DeInit();
          TIM2_TimeBaseInit(TIM2_PRESCALER_64, period);
        }

        if(g_analog_config[id].timChannel == TIM2_CHANNEL_1) {
          TIM2_OC1Init(TIM2_OCMODE_PWM1, TIM2_OUTPUTSTATE_ENABLE, value,
                       TIM2_OCPOLARITY_HIGH);
          TIM2_OC1PreloadConfig(ENABLE);
        } else if(g_analog_config[id].timChannel == TIM2_CHANNEL_2) {
          TIM2_OC2Init(TIM2_OCMODE_PWM1, TIM2_OUTPUTSTATE_ENABLE, value,
                       TIM2_OCPOLARITY_HIGH);
          TIM2_OC2PreloadConfig(ENABLE);
        } else if(g_analog_config[id].timChannel == TIM2_CHANNEL_3) {
          TIM2_OC3Init(TIM2_OCMODE_PWM1, TIM2_OUTPUTSTATE_ENABLE, value,
                       TIM2_OCPOLARITY_HIGH);
          TIM2_OC3PreloadConfig(ENABLE);
        } else {return;}

        TIM2_ARRPreloadConfig(ENABLE);
        TIM2_Cmd(ENABLE);
        break;

    default:
      break;
  }
}

/**
  * @brief  This function will disable the PWM
  * @param  port : the gpio port to use
  * @param  pin : the gpio pin to use
  * @retval None
  */
void pwm_stop(GPIO_TypeDef  *port, uint32_t pin)
{
  //find the instance in the global
  int8_t id = get_analog_instance(port, pin);
  if(id < 0) return;

  switch(g_analog_config[id].timer_id) {
    case TIM1_E:
      TIM1_DeInit();
      break;

    case TIM2_E:
      TIM2_DeInit();
      break;

    default:
      break;
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

/**
  * @}
  */
#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
