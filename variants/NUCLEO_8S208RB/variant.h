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

#ifndef _VARIANT_ARDUINO_STM8_
#define _VARIANT_ARDUINO_STM8_

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "PeripheralPins.h"

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/
extern const PinName digitalPin[];

enum {
    PD6, //D0  RX
    PD5, //D1  TX
    PE0, //D2
    PC1, //D3
    PG0, //D4
    PC2, //D5
    PC3, //D6
    PD1, //D7
    PD3, //D8
    PC4, //D9
    PE5, //D10
    PC6, //D11
    PC7, //D12
    PC5, //D13 LED
    PE2, //D14
    PE1, //D15
    // ST Morpho
    // CN1 Left Side
    PG4, //D16
    PG5, //D17
    PE7, //D18
    PE6, //D19
    PA3, //D20
    PA4, //D21
    PA5, //D22
    PA6, //D23
    // CN1 Right Side
    PG7, //D24
    PG6, //D25
    // CN9 Left Side
    PD7, //D26
    // CN9 Right Side
    PB7, //D27
    PB6, //D28
    PD4, //D29
    PF7, //D30
    PF6, //D31
    PF5, //D32
    PF4, //D33
    PF3, //D34
    PF0, //D35
    PD2, //D36
    PD0, //D37
    PG1, //D38
    PE4, //D39 User button
    PE3, //D40
    PG3, //D41
    PG2, //D42
    PB5, //D43/A0
    PB4, //D44/A1
    PB3, //D45/A2
    PB2, //D46/A3
    PB1, //D47/A4
    PB0, //D48/A5
    // Duplicated pins in order to be aligned with PinMap_ADC
    PB6_2, //D49/A6 = D29
    PB7_2, //D50/A7 = D28
    PE6_2, //D51/A8 = D19
    PE7_2, //D52/A9 = D18
    PF0_2, //D53/A10 = D36
    PF3_2, //D54/A11 = D35
    PF4_2, //D55/A12 = D34
    PF5_2, //D56/A13 = D33
    PF6_2, //D57/A14 = D32
    PF7_2, //D58/A15 = D31
    PEND
};

// This must be a literal with the same value as PEND
#define NUM_DIGITAL_PINS    59
// This must be a literal with a value less than or equal to to MAX_ANALOG_INPUTS
#define NUM_ANALOG_INPUTS   16
#define NUM_ANALOG_FIRST    43

// On-board LED pin number
#define LED_BUILTIN         13
#define LED_GREEN           LED_BUILTIN

// On-board User button
#define USER_BTN            PE4

// Timer Definitions
//Do not use timer used by PWM pins when possible. See PinMap_PWM in PeripheralPins.c
#define TIMER_TONE          TIM2

// Do not use basic timer: OC is required
#define TIMER_SERVO         TIM1

// UART Definitions

#define SERIAL_UART_INSTANCE    1 //Connected to ST-Link
// Default pin used for 'Serial' instance (ex: ST-Link)

#define PIN_SERIAL_RX           PA_4
#define PIN_SERIAL_TX           PA_5

// Default pin used for Serial printed on Arduino connector
#define PIN_SERIAL3_RX           PD_6
#define PIN_SERIAL3_TX           PD_5

#ifdef __cplusplus
} // extern "C"
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR     Serial
#define SERIAL_PORT_HARDWARE    Serial
#endif


#endif /* _VARIANT_ARDUINO_STM8_ */
