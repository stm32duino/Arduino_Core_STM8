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
 */


#include "variant.h"

#ifdef __cplusplus
extern "C" {
#endif

// Pin number
// This array allows to wrap Arduino pin number(Dx or x)
// to STM32 PinName (PX_n)

const PinName digitalPin[] = {
    //PX_n, //Dx
    PG_0, //D0 RX
    PG_1, //D1 TX
    PE_0, //D2
    PE_1, //D3 PWM
    PE_2, //D4
    PC_2, //D5 PWM
    PC_3, //D6 PWM
    PD_1, //D7
    PD_3, //D8
    PC_4, //D9 PWM
    PB_4, //D10 SPI Clock
    PB_6, //D11 SPI MOSI
    PB_7, //D12 SPI MISO
    PB_5, //D13 SPI SCK
    PC_0, //D14 I2C SDA
    PC_1, //D15 I2C SCL
    // ST Morpho
    // CN1 Left Side
    PG_4, //D16
    PG_5, //D17
    PE_6, //D18
    PC_5, //D19
    PC_6, //D20
    PA_1, //D21
    PA_2, //D22
    PA_3, //D23
    PA_5, //D24
    PA_7, //D25
    PA_0, //D26
    // CN1 Right Side
    PG_7, //D27
    PG_6, //D28
    // CN9 Left Side
    PD_7, //D29
    // CN9 Right Side
    PD_6, //D30
    PD_5, //D31
    PD_4, //D32
    PF_7, //D33
    PF_6, //D34
    PF_5, //D35
    PF_4, //D36
    PF_1, //D37
    PF_0, //D38
    PD_2, //D39
    PD_0, //D40
    PE_5, //D41
    PE_4, //D42
    PE_3, //D43
    PG_3, //D44
    PG_2, //D45
    PC_7, //D46/A0
    PA_6, //D47/A1
    PB_3, //D48/A2
    PB_2, //D49/A3
    PB_1, //D50/A4
    PB_0, //D51/A5
    // Duplicated pins in order to be aligned with PinMap_ADC
    PA_5, //D52/A6
    PB_4, //D53/A7
    PB_5, //D54/A8
    PB_6, //D55/A9
    PB_7, //D56/A10
    PC_2, //D57/A11
    PC_3, //D58/A12
    PC_4, //D59/A13
    PD_0, //D60/A14
    PD_1, //D61/A15
    PD_2, //D62/A16
    PD_4, //D63/A17
    PD_5, //D64/A18
    PD_6, //D65/A19
    PD_7, //D66/A20
    PE_5, //D67/A21
    PF_0, //D68/A22
    PF_1 //D69/A23
};

#ifdef __cplusplus
}
#endif

// ----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

#pragma weak SystemClock_Config
extern void SystemClock_Config(void);

#ifdef __cplusplus
}
#endif
