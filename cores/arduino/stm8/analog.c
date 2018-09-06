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
#include "analog.h"
#include "stm8_def.h"
#include "timer.h"
#include "PinAF_STM8Lxx.h"

#ifdef __cplusplus
extern "C"
{
#endif

    static PinName g_current_pin = NC;
    static uint16_t get_adc_channel(PinName pin)
    {
        uint16_t function = pinmap_function(pin, PinMap_ADC);
        uint16_t channel = 0;
#if defined(STM8Sxx)
#if defined(ADC2)
        switch (STM_PIN_CHANNEL(function))
        {
        case 1:
            channel = ADC2_CHANNEL_1;
            break;
        case 2:
            channel = ADC2_CHANNEL_2;
            break;
        case 3:
            channel = ADC2_CHANNEL_3;
            break;
        case 4:
            channel = ADC2_CHANNEL_4;
            break;
        case 5:
            channel = ADC2_CHANNEL_5;
            break;
        case 6:
            channel = ADC2_CHANNEL_6;
            break;
        case 7:
            channel = ADC2_CHANNEL_7;
            break;
        case 8:
            channel = ADC2_CHANNEL_8;
            break;
        case 9:
            channel = ADC2_CHANNEL_9;
            break;
        case 12:
            channel = ADC2_CHANNEL_12;
            break;
        case 13:
            channel = ADC2_CHANNEL_13;
            break;
        case 14:
            channel = ADC2_CHANNEL_14;
            break;
        case 15:
            channel = ADC2_CHANNEL_15;
            break;
        default:
            channel = 0;
            break;
        }
#elif defined(ADC1)
        switch (STM_PIN_CHANNEL(function))
        {
        case 1:
            channel = ADC1_CHANNEL_1;
            break;
        case 2:
            channel = ADC1_CHANNEL_2;
            break;
        case 3:
            channel = ADC1_CHANNEL_3;
            break;
        case 4:
            channel = ADC1_CHANNEL_4;
            break;
        case 5:
            channel = ADC1_CHANNEL_5;
            break;
        case 6:
            channel = ADC1_CHANNEL_6;
            break;
        case 7:
            channel = ADC1_CHANNEL_7;
            break;
        case 8:
            channel = ADC1_CHANNEL_8;
            break;
        case 9:
            channel = ADC1_CHANNEL_9;
            break;
        case 12:
            channel = ADC1_CHANNEL_12;
            break;
        default:
            channel = 0;
            break;
        }
#endif
#elif defined(STM8Lxx)
    switch (STM_PIN_CHANNEL(function))
    {
    case 0:
        channel = ADC_Channel_0;
        break;
    case 1:
        channel = ADC_Channel_1;
        break;
    case 2:
        channel = ADC_Channel_2;
        break;
    case 3:
        channel = ADC_Channel_3;
        break;
    case 4:
        channel = ADC_Channel_4;
        break;
    case 5:
        channel = ADC_Channel_5;
        break;
    case 6:
        channel = ADC_Channel_6;
        break;
    case 7:
        channel = ADC_Channel_7;
        break;
    case 8:
        channel = ADC_Channel_8;
        break;
    case 9:
        channel = ADC_Channel_9;
        break;
    case 10:
        channel = ADC_Channel_10;
        break;
    case 11:
        channel = ADC_Channel_11;
        break;
    case 12:
        channel = ADC_Channel_12;
        break;
    case 13:
        channel = ADC_Channel_13;
        break;
    case 14:
        channel = ADC_Channel_14;
        break;
    case 15:
        channel = ADC_Channel_15;
        break;
    case 16:
        channel = ADC_Channel_16;
        break;
    case 17:
        channel = ADC_Channel_17;
        break;
    case 18:
        channel = ADC_Channel_18;
        break;
    case 19:
        channel = ADC_Channel_19;
        break;
    case 20:
        channel = ADC_Channel_20;
        break;
    case 21:
        channel = ADC_Channel_21;
        break;
    case 22:
        channel = ADC_Channel_22;
        break;
    case 23:
        channel = ADC_Channel_23;
        break;
    case 24:
        channel = ADC_Channel_24;
        break;
    case 25:
        channel = ADC_Channel_25;
        break;
    case 26:
        channel = ADC_Channel_26;
        break;
    case 27:
        channel = ADC_Channel_27;
        break;
    default:
        channel = 0;
        break;
    }

#endif
        return channel;
    }

    static uint8_t get_pwm_channel(PinName pin)
    {
        uint16_t function = pinmap_function(pin, PinMap_PWM);
        uint8_t channel = 0;
        switch (STM_PIN_CHANNEL(function))
        {
        case 1:
            channel = 0x01;
            break;
        case 2:
            channel = 0x02;
            break;
        case 3:
            channel = 0x03;
            break;
        case 4:
            channel = 0x04;
            break;
        default:
            break;
        }

        return channel;
    }

    uint16_t adc_read_value(PinName pin)
    {
        uint16_t channel = get_adc_channel(pin);
        void *peripheral = pinmap_peripheral(pin, PinMap_ADC);
        __IO uint16_t uhADCxConvertedValue = 0;
#if defined(STM8Sxx)
#if defined(ADC2)
        ADC2_DeInit();
        ADC2_Init(ADC2_CONVERSIONMODE_SINGLE, (uint8_t)channel,
                  ADC2_PRESSEL_FCPU_D2, ADC2_EXTTRIG_TIM, DISABLE,
                  ADC2_ALIGN_RIGHT, ADC2_SCHMITTTRIG_CHANNEL12, DISABLE);
        ADC2_StartConversion();
        while(ADC2_GetFlagStatus() == RESET);
        ADC2_ClearFlag();
        uhADCxConvertedValue = ADC2_GetConversionValue();
        ADC2_Cmd(DISABLE);

        return uhADCxConvertedValue;
#endif
#if defined(ADC1)
        ADC1_DeInit();
        ADC1_Init(ADC1_CONVERSIONMODE_SINGLE, (uint8_t)channel,
                ADC1_PRESSEL_FCPU_D2, ADC1_EXTTRIG_TIM, DISABLE,
                ADC1_ALIGN_RIGHT, ADC1_SCHMITTTRIG_CHANNEL12, DISABLE);
        ADC1_StartConversion();
        while(ADC1_GetFlagStatus(ADC1_FLAG_EOC) == RESET);
        ADC1_ClearFlag(ADC1_FLAG_EOC);
        uhADCxConvertedValue = ADC1_GetConversionValue();
        ADC1_Cmd(DISABLE);

        return uhADCxConvertedValue;

#endif
#endif
#if defined(STM8Lxx)
        CLK_PeripheralClockConfig(CLK_Peripheral_ADC1, ENABLE);

        ADC_DeInit(ADC1);
        ADC_Init(ADC1, ADC_ConversionMode_Single, ADC_Resolution_10Bit, ADC_Prescaler_1);
        ADC_SamplingTimeConfig(ADC1, ADC_Group_SlowChannels, ADC_SamplingTime_384Cycles);
        ADC_Cmd(ADC1, ENABLE);
        ADC_ChannelCmd(ADC1, channel, ENABLE);
        ADC_SoftwareStartConv(ADC1);
        while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC) == RESET);
        ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
        uhADCxConvertedValue = ADC_GetConversionValue(ADC1);

        return uhADCxConvertedValue;

#endif
    }

    void pwm_start(PinName pin, uint32_t clock_freq, uint32_t period, uint32_t value, uint8_t do_init)
    {
        void *peripheral = pinmap_peripheral(pin, PinMap_PWM);
        uint16_t function = pinmap_function(pin, PinMap_PWM);
        uint8_t id = get_pwm_channel(pin);
#if defined(STM8Sxx)
        if (peripheral == TIM1)
        {
            TIM1_TimeBaseInit((uint16_t)(CLK_GetClockFreq() / clock_freq) - 1, TIM1_COUNTERMODE_UP, period-1, 0);

            switch (id)
            {
            case 1:
                TIM1_OC1Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_ENABLE,
                             value, TIM1_OCPOLARITY_LOW, TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_SET,
                             TIM1_OCNIDLESTATE_RESET);
                break;
            case 2:
                TIM1_OC2Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_ENABLE,
                             value, TIM1_OCPOLARITY_LOW, TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_SET,
                             TIM1_OCNIDLESTATE_RESET);
                break;
            case 3:
                TIM1_OC3Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_ENABLE,
                             value, TIM1_OCPOLARITY_LOW, TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_SET,
                             TIM1_OCNIDLESTATE_RESET);
                break;
            case 4:
                TIM1_OC4Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, value, TIM1_OCPOLARITY_LOW,
                             TIM1_OCIDLESTATE_SET);
                break;
            default:
                return;
            }
            TIM1_Cmd(ENABLE);
            TIM1_CtrlPWMOutputs(ENABLE);
        }
#ifdef TIM2
        else if (peripheral == TIM2)
        {
            TIM2_TimeBaseInit(TIM2_PRESCALER_64, period-1);
            switch (id)
            {
            case 1:
                TIM2_OC1Init(TIM2_OCMODE_PWM1, TIM2_OUTPUTSTATE_ENABLE, value,
                             TIM2_OCPOLARITY_HIGH);
                break;
            case 2:
                TIM2_OC2Init(TIM2_OCMODE_PWM1, TIM2_OUTPUTSTATE_ENABLE, value,
                             TIM2_OCPOLARITY_HIGH);
                break;
            case 3:
                TIM2_OC3Init(TIM2_OCMODE_PWM1, TIM2_OUTPUTSTATE_ENABLE, value,
                             TIM2_OCPOLARITY_HIGH);
                break;
            default:
                return;
            }
            TIM2_ARRPreloadConfig(ENABLE);
            TIM2_Cmd(ENABLE);
        }
#endif
#ifdef TIM3
        else if (peripheral == TIM3)
        {
            TIM3_TimeBaseInit(TIM3_PRESCALER_64, period-1);
            switch (id)
            {
            case 1:
                TIM3_OC1Init(TIM3_OCMODE_PWM1, TIM3_OUTPUTSTATE_ENABLE, value,
                             TIM3_OCPOLARITY_HIGH);
                break;
            case 2:
                TIM3_OC2Init(TIM3_OCMODE_PWM1, TIM3_OUTPUTSTATE_ENABLE, value,
                             TIM2_OCPOLARITY_HIGH);
                break;

            default:
                return;
            }
            TIM3_ARRPreloadConfig(ENABLE);
            TIM3_Cmd(ENABLE);
        }
#endif
#ifdef TIM5
        else if (peripheral == TIM5)
        {
            TIM5_TimeBaseInit(TIM5_PRESCALER_64, period-1);
            switch (id)
            {
            case 1:
                TIM5_OC1Init(TIM5_OCMODE_PWM1, TIM5_OUTPUTSTATE_ENABLE, value,
                             TIM5_OCPOLARITY_HIGH);
                break;
            case 2:
                TIM5_OC2Init(TIM5_OCMODE_PWM1, TIM5_OUTPUTSTATE_ENABLE, value,
                             TIM5_OCPOLARITY_HIGH);
                break;
            case 3:
                TIM5_OC3Init(TIM5_OCMODE_PWM1, TIM5_OUTPUTSTATE_ENABLE, value,
                             TIM5_OCPOLARITY_HIGH);
                break;
            default:
                return;
            }
            TIM5_ARRPreloadConfig(ENABLE);
            TIM5_Cmd(ENABLE);
        }
#endif
        else
            return;
#elif defined(STM8Lxx)
        if (peripheral == TIM1)
        {
            CLK_PeripheralClockConfig(CLK_Peripheral_TIM1, ENABLE);

            if (do_init == 1)
            {
                TIM1_TimeBaseInit((uint16_t)(CLK_GetClockFreq() / clock_freq) - 1, TIM1_CounterMode_Up, period-1, 0);
            }
            switch (id)
            {
            case 1:
                TIM1_OC1Init(TIM1_OCMode_PWM2, TIM1_OutputState_Enable, TIM1_OutputNState_Enable,
                             value, TIM1_OCPolarity_Low, TIM1_OCNPolarity_High, TIM1_OCIdleState_Set,
                             TIM1_OCNIdleState_Reset);
                break;
            case 2:
                TIM1_OC2Init(TIM1_OCMode_PWM2, TIM1_OutputState_Enable, TIM1_OutputNState_Enable, value,
                             TIM1_OCPolarity_Low, TIM1_OCNPolarity_High, TIM1_OCIdleState_Set,
                             TIM1_OCNIdleState_Reset);
                break;
            case 3:
                TIM1_OC3Init(TIM1_OCMode_PWM2, TIM1_OutputState_Enable, TIM1_OutputNState_Enable,
                             value, TIM1_OCPolarity_Low, TIM1_OCNPolarity_High, TIM1_OCIdleState_Set,
                             TIM1_OCNIdleState_Reset);
                break;
            default:
                return;
            }
            TIM1_Cmd(ENABLE);
            TIM1_CtrlPWMOutputs(ENABLE);
        }
        else if (peripheral == TIM2)
        {
            CLK_PeripheralClockConfig(CLK_Peripheral_TIM2, ENABLE);
            TIM2_TimeBaseInit(TIM2_Prescaler_64, TIM2_CounterMode_Up, period-1);
            pin_Set8LAFPin(STM_PIN_AFNUM(function));
            switch (id)
            {
            case 1:
                TIM2_OC1Init(TIM2_OCMode_PWM1, TIM2_OutputState_Enable, value,
                             TIM2_OCPolarity_High, TIM2_OCIdleState_Set);
                break;
            case 2:
                TIM2_OC2Init(TIM2_OCMode_PWM1, TIM2_OutputState_Enable, value, TIM2_OCPolarity_High,
                             TIM2_OCIdleState_Set);
                break;
            default:
                return;
            }
            TIM2_CtrlPWMOutputs(ENABLE);
            TIM2_Cmd(ENABLE);

        }
        else if (peripheral == TIM3)
        {
            CLK_PeripheralClockConfig(CLK_Peripheral_TIM3, ENABLE);
            TIM3_TimeBaseInit(TIM3_Prescaler_64, TIM3_CounterMode_Up, period-1);
            pin_Set8LAFPin(STM_PIN_AFNUM(function));
            switch (id)
            {
            case 1:
                TIM3_OC1Init(TIM3_OCMode_PWM1, TIM3_OutputState_Enable, value,
                             TIM3_OCPolarity_High, TIM3_OCIdleState_Set);
                break;
            case 2:
                TIM3_OC2Init(TIM3_OCMode_PWM1, TIM3_OutputState_Enable, value, TIM3_OCPolarity_High,
                             TIM3_OCIdleState_Set);
                break;
            default:
                return;
            }
            TIM3_CtrlPWMOutputs(ENABLE);
            TIM3_Cmd(ENABLE);
        }
        else if (peripheral == TIM5)
        {
            CLK_PeripheralClockConfig(CLK_Peripheral_TIM5, ENABLE);
            TIM5_TimeBaseInit(TIM5_Prescaler_64, TIM5_CounterMode_Up, period-1);
            switch (id)
            {
            case 1:
                TIM5_OC1Init(TIM5_OCMode_PWM1, TIM5_OutputState_Enable, value,
                             TIM5_OCPolarity_High, TIM5_OCIdleState_Set);

                break;
            case 2:
                TIM5_OC2Init(TIM5_OCMode_PWM1, TIM5_OutputState_Enable, value, TIM5_OCPolarity_High,
                             TIM5_OCIdleState_Set);

                break;
            default:
                return;
            }
            TIM5_CtrlPWMOutputs(ENABLE);
            TIM5_Cmd(ENABLE);
        }
        else
            return;
#endif
    }

    void pwm_stop(PinName pin)
    {
        void *peripheral = pinmap_peripheral(pin, PinMap_PWM);
        if (peripheral == TIM1)
        {
#if defined(STM8Lxx)
            CLK_PeripheralClockConfig(CLK_Peripheral_TIM1, DISABLE);
#endif
            TIM1_DeInit();
        }
#ifdef TIM2
        else if (peripheral == TIM2)
        {
#if defined(STM8Lxx)
            CLK_PeripheralClockConfig(CLK_Peripheral_TIM2, DISABLE);
#endif
            TIM2_DeInit();
        }
#endif
#ifdef TIM3
    else if (peripheral == TIM3)
        {
#if defined(STM8Lxx)
            CLK_PeripheralClockConfig(CLK_Peripheral_TIM3, DISABLE);
#endif
            TIM3_DeInit();
        }
#endif
#ifdef TIM5
    else if (peripheral == TIM5)
        {
#if defined(STM8Lxx)
            CLK_PeripheralClockConfig(CLK_Peripheral_TIM5, DISABLE);
#endif
            TIM5_DeInit();
        }
#endif
        else
        {
            return;
        }
    }

#ifdef __cplusplus
}
#endif