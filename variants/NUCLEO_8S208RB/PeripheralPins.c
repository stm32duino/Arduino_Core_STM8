/*
 *******************************************************************************
 * Copyright (c) 2018, STMicroelectronics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
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
 *******************************************************************************
 * Automatically generated from STM8S208RBTx.xml
 */
#include "Arduino.h"
#include "PeripheralPins.h"

/* =====
 * Note: Commented lines are alternative possibilities which are not used per default.
 *       If you change them, you will have to know what you do
 * =====
 */

//*** ADC ***

#if !defined(NO_HWADC)
const PinMap PinMap_ADC[] = {
    {PB_0,  ADCx,  STM_PIN_DATA_EXT(STM_MODE_ANALOG, GPIO_MODE_IN_FL_NO_IT, 0, 0, 0)}, // ADC_IN0
    {PB_1,  ADCx,  STM_PIN_DATA_EXT(STM_MODE_ANALOG, GPIO_MODE_IN_FL_NO_IT, 0, 1, 0)}, // ADC_IN1
    {PB_2,  ADCx,  STM_PIN_DATA_EXT(STM_MODE_ANALOG, GPIO_MODE_IN_FL_NO_IT, 0, 2, 0)}, // ADC_IN2
    {PB_3,  ADCx,  STM_PIN_DATA_EXT(STM_MODE_ANALOG, GPIO_MODE_IN_FL_NO_IT, 0, 3, 0)}, // ADC_IN3
    {PB_4,  ADCx,  STM_PIN_DATA_EXT(STM_MODE_ANALOG, GPIO_MODE_IN_FL_NO_IT, 0, 4, 0)}, // ADC_IN4
    {PB_5,  ADCx,  STM_PIN_DATA_EXT(STM_MODE_ANALOG, GPIO_MODE_IN_FL_NO_IT, 0, 5, 0)}, // ADC_IN5
    {PB_6,  ADCx,  STM_PIN_DATA_EXT(STM_MODE_ANALOG, GPIO_MODE_IN_FL_NO_IT, 0, 6, 0)}, // ADC_IN6
    {PB_7,  ADCx,  STM_PIN_DATA_EXT(STM_MODE_ANALOG, GPIO_MODE_IN_FL_NO_IT, 0, 7, 0)}, // ADC_IN7
    {PE_6,  ADCx,  STM_PIN_DATA_EXT(STM_MODE_ANALOG, GPIO_MODE_IN_FL_NO_IT, 0, 9, 0)}, // ADC_IN9
    {PE_7,  ADCx,  STM_PIN_DATA_EXT(STM_MODE_ANALOG, GPIO_MODE_IN_FL_NO_IT, 0, 8, 0)}, // ADC_IN8
    {PF_0,  ADCx,  STM_PIN_DATA_EXT(STM_MODE_ANALOG, GPIO_MODE_IN_FL_NO_IT, 0, 10, 0)}, // ADC_IN10
    {PF_3,  ADCx,  STM_PIN_DATA_EXT(STM_MODE_ANALOG, GPIO_MODE_IN_FL_NO_IT, 0, 11, 0)}, // ADC_IN11
    {PF_4,  ADCx,  STM_PIN_DATA_EXT(STM_MODE_ANALOG, GPIO_MODE_IN_FL_NO_IT, 0, 12, 0)}, // ADC_IN12
    {PF_5,  ADCx,  STM_PIN_DATA_EXT(STM_MODE_ANALOG, GPIO_MODE_IN_FL_NO_IT, 0, 13, 0)}, // ADC_IN13
    {PF_6,  ADCx,  STM_PIN_DATA_EXT(STM_MODE_ANALOG, GPIO_MODE_IN_FL_NO_IT, 0, 14, 0)}, // ADC_IN14
    {PF_7,  ADCx,  STM_PIN_DATA_EXT(STM_MODE_ANALOG, GPIO_MODE_IN_FL_NO_IT, 0, 15, 0)}, // ADC_IN15
    {NC,    NP,    0}
};
#endif

//*** DAC ***

//*** No DAC ***

//*** I2C ***

#if !defined(NO_HWI2C)
const PinMap PinMap_I2C_SDA[] = {
    {PB_5,  I2C, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_MODE_IN_FL_NO_IT, AFIO_NONE)},
    {PE_2,  I2C, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_MODE_IN_FL_NO_IT, AFIO_NONE)},
    {NC,    NP,    0}
};
#endif

#if !defined(NO_HWI2C)
const PinMap PinMap_I2C_SCL[] = {
    {PB_4,  I2C, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_MODE_IN_FL_NO_IT, AFIO_NONE)},
    {PE_1,  I2C, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_MODE_IN_FL_NO_IT, AFIO_NONE)},
    {NC,    NP,    0}
};
#endif

//*** PWM ***

#if !defined(NO_HWTIM)
const PinMap PinMap_PWM[] = {
    {PA_3,  TIM2,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_MODE_OUT_PP_LOW_FAST, AFIO_NONE, 3, 0)},  // TIM2_CH3
    {PA_3,  TIM3,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_MODE_OUT_PP_LOW_FAST, AFIO_NONE, 1, 0)},  // TIM3_CH1
    {PB_0,  TIM1,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_MODE_OUT_PP_LOW_FAST, AFIO_NONE, 1, 1)},  // TIM1_CH1N
    {PB_1,  TIM1,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_MODE_OUT_PP_LOW_FAST, AFIO_NONE, 2, 1)},  // TIM1_CH2N
    {PB_2,  TIM1,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_MODE_OUT_PP_LOW_FAST, AFIO_NONE, 3, 1)},  // TIM1_CH3N
    {PC_1,  TIM1,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_MODE_OUT_PP_LOW_FAST, AFIO_NONE, 1, 0)},  // TIM1_CH1
    {PC_2,  TIM1,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_MODE_OUT_PP_LOW_FAST, AFIO_NONE, 2, 0)},  // TIM1_CH2
    {PC_3,  TIM1,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_MODE_OUT_PP_LOW_FAST, AFIO_NONE, 3, 0)},  // TIM1_CH3
    {PC_4,  TIM1,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_MODE_OUT_PP_LOW_FAST, AFIO_NONE, 4, 0)},  // TIM1_CH4
    {PD_0,  TIM3,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_MODE_OUT_PP_LOW_FAST, AFIO_NONE, 2, 0)},  // TIM3_CH2
    {PD_2,  TIM2,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_MODE_OUT_PP_LOW_FAST, AFIO_NONE, 3, 0)},  // TIM2_CH3
    {PD_2,  TIM3,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_MODE_OUT_PP_LOW_FAST, AFIO_NONE, 1, 0)},  // TIM3_CH1
    {PD_3,  TIM2,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_MODE_OUT_PP_LOW_FAST, AFIO_NONE, 2, 0)},  // TIM2_CH2
    {PD_4,  TIM2,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_MODE_OUT_PP_LOW_FAST, AFIO_NONE, 1, 0)},  // TIM2_CH1
    {NC,    NP,    0}
};
#endif

//*** SERIAL ***

#if !defined(NO_HWSERIAL)
const PinMap PinMap_UART_TX[] = {
    {PA_5,  UART1,   STM_PIN_DATA(STM_MODE_AF_PP, GPIO_MODE_OUT_PP_LOW_FAST, AFIO_NONE)},
    {PD_5,  UART3,   STM_PIN_DATA(STM_MODE_AF_PP, GPIO_MODE_OUT_PP_LOW_FAST, AFIO_NONE)},
    {NC,    NP,    0}
};
#endif

#if !defined(NO_HWSERIAL)
const PinMap PinMap_UART_RX[] = {
    {PA_4,  UART1,   STM_PIN_DATA(STM_MODE_AF_PP, GPIO_MODE_IN_PU_NO_IT, AFIO_NONE)},
    {PD_6,  UART3,   STM_PIN_DATA(STM_MODE_AF_PP, GPIO_MODE_IN_PU_NO_IT, AFIO_NONE)},
    {NC,    NP,    0}
};
#endif

//*** SPI ***

#if !defined(NO_HWSPI)
const PinMap PinMap_SPI_MOSI[] = {
    {PC_6,  SPI1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_MODE_IN_PU_NO_IT, AFIO_NONE)},
    {NC,    NP,    0}
};
#endif

#if !defined(NO_HWSPI)
const PinMap PinMap_SPI_MISO[] = {
    {PC_7,  SPI1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_MODE_IN_PU_NO_IT, AFIO_NONE)},
    {NC,    NP,    0}
};
#endif

#if !defined(NO_HWSPI)
const PinMap PinMap_SPI_SCLK[] = {
    {PC_5,  SPI1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_MODE_IN_PU_NO_IT, AFIO_NONE)},
    {NC,    NP,    0}
};
#endif

#if !defined(NO_HWSPI)
const PinMap PinMap_SPI_SSEL[] = {
    {PE_5,  SPI1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_MODE_IN_PU_NO_IT, AFIO_NONE)},
    {NC,    NP,    0}
};
#endif

//*** CAN ***

#if !defined(NO_HWCAN)
const PinMap PinMap_CAN_RD[] = {
    {PG_1,  CAN, STM_PIN_DATA(STM_MODE_INPUT, GPIO_MODE_IN_PU_NO_IT, AFIO_NONE)},
    {NC,    NP,    0}
};
#endif

#if !defined(NO_HWCAN)
const PinMap PinMap_CAN_TD[] = {
    {PG_0,  CAN, STM_PIN_DATA(STM_MODE_INPUT, GPIO_MODE_IN_PU_NO_IT, AFIO_NONE)},
    {NC,    NP,    0}
};
#endif

//*** QUADSPI ***

//*** No QUADSPI ***
