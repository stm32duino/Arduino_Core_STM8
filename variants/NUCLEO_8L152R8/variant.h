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
    PG0, //D0 RX
    PG1, //D1 TX
    PE0, //D2
    PE1, //D3 PWM
    PE2, //D4
    PC2, //D5 PWM
    PC3, //D6 PWM
    PD1, //D7
    PD3, //D8
    PC4, //D9 PWM
    PB4, //D10 SPI Clock
    PB6, //D11 SPI MOSI
    PB7, //D12 SPI MISO
    PB5, //D13 SPI SCK
    PC0, //D14 I2C SDA
    PC1, //D15 I2C SCL
    // ST Morpho
    // CN1 Left Side
    PG4, //D16
    PG5, //D17
    PE6, //D18
    PC5, //D19
    PC6, //D20
    PA1, //D21
    PA2, //D22
    PA3, //D23
    PA5, //D24
    PA7, //D25
    PA0, //D26
    // CN1 Right Side
    PG7, //D27
    PG6, //D28
    // CN9 Left Side
    PD7, //D29
    // CN9 Right Side
    PD6, //D30
    PD5, //D31
    PD4, //D32
    PF7, //D33
    PF6, //D34
    PF5, //D35
    PF4, //D36
    PF1, //D37
    PF0, //D38
    PD2, //D39
    PD0, //D40
    PE5, //D41
    PE4, //D42
    PE3, //D43
    PG3, //D44
    PG2, //D45
    PC7, //D46/A0
    PA6, //D47/A1
    PB3, //D48/A2
    PB2, //D49/A3
    PB1, //D50/A4
    PB0, //D51/A5
    // Duplicated pins in order to be aligned with PinMap_ADC
    PA5_2, //D52/A6
    PB4_2, //D53/A7
    PB5_2, //D54/A8
    PB6_2, //D55/A9
    PB7_2, //D56/A10
    PC2_2, //D57/A11
    PC3_2, //D58/A12
    PC4_2, //D59/A13
    PD0_2, //D60/A14
    PD1_2, //D61/A15
    PD2_2, //D62/A16
    PD4_2, //D63/A17
    PD5_2, //D64/A18
    PD6_2, //D65/A19
    PD7_2, //D66/A20
    PE5_2, //D67/A21
    PF0_2, //D68/A22
    PF1_2, //D69/A23
    PEND
};

// This must be a literal with the same value as PEND
#define NUM_DIGITAL_PINS        70
// This must be a literal with a value less than or equal to to MAX_ANALOG_INPUTS
#define NUM_ANALOG_INPUTS       24
#define NUM_ANALOG_FIRST        46

// On-board LED pin number
#define LED_BUILTIN             13
#define LED_GREEN               LED_BUILTIN

// On-board user button
#define USER_BTN                PG4

// Timer Definitions

//Do not use timer used by PWM pins when possible. See PinMap_PWM in PeripheralPins.c
#define TIMER_TONE              TIM2

// Do not use basic timer: OC is required
#define TIMER_SERVO             TIM1  //TODO: advanced-control timers don't work

// UART Definitions
#define SERIAL_UART_INSTANCE    2 //Connected to ST-Link
// Default pin used for 'Serial' instance (ex: ST-Link)

#define PIN_SERIAL_RX           PE_3
#define PIN_SERIAL_TX           PE_4

// Default pin used for Serial printed on Arduino connector
#define PIN_SERIAL3_RX           PG_0
#define PIN_SERIAL3_TX           PG_1

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
#define SERIAL_PORT_HARDWARE    Serial1
#endif

#endif /* _VARIANT_ARDUINO_STM8_ */
