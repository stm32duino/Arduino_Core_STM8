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
    PD_6, //D0  RX
    PD_5, //D1  TX
    PE_0, //D2
    PC_1, //D3 PWM
    PG_0, //D4
    PC_2, //D5 PWM
    PC_3, //D6 PWM
    PD_1, //D7
    PD_3, //D8
    PC_4, //D9 PWM
    PE_5, //D10 SPI Clock
    PC_6, //D11 SPI MOSI
    PC_7, //D12 SPI MISO
    PC_5, //D13 SPI SCK
    PE_2, //D14 I2C SDA
    PE_1, //D15 I2C SCL
    // ST Morpho
    // CN1 Left Side
    PG_4, //D16
    PG_5, //D17
    PE_7, //D18
    PE_6, //D19
    PA_3, //D20
    PA_4, //D21
    PA_5, //D22
    PA_6, //D23
    // CN1 Right Side
    PG_7, //D24
    PG_6, //D25
    // CN9 Left Side
    PD_7, //D26
    // CN9 Right Side
    PB_7, //D27
    PB_6, //D28
    PD_4, //D29
    PF_7, //D30
    PF_6, //D31
    PF_5, //D32
    PF_4, //D33
    PF_3, //D34
    PF_0, //D35
    PD_2, //D36
    PD_0, //D37
    PG_1, //D38
    PE_4, //D39
    PE_3, //D40
    PG_3, //D41
    PG_2, //D42
    PB_5, //D43/A0
    PB_4, //D44/A1
    PB_3, //D45/A2
    PB_2, //D46/A3
    PB_1, //D47/A4
    PB_0, //D48/A5
        // Duplicated pins in order to be aligned with PinMap_ADC
    PB_6, //D49/A6 = D29
    PB_7, //D50/A7 = D28
    PE_6, //D51/A8 = D19
    PE_7, //D52/A9 = D18
    PF_0, //D53/A10 = D36
    PF_3, //D54/A11 = D35
    PF_4, //D55/A12 = D34
    PF_5, //D56/A13 = D33
    PF_6, //D57/A14 = D32
    PF_7 //D58/A15 = D31
};

#ifdef __cplusplus
}
#endif

// ----------------------------------------------------------------------------

#ifdef __cplusplus
    extern "C" {
#endif


#ifdef __cplusplus
    }
#endif