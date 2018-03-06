/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _VARIANT_ARDUINO
#define _VARIANT_ARDUINO

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
//#define VARIANT_MAINOSC		12000000

/** Master clock frequency */
//#define VARIANT_MCK			84000000

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "Arduino.h"
#ifdef __cplusplus
#include "UARTClass.h"
#include "USARTClass.h"
#endif

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

/**
 * Libc porting layers
 */
#if defined (  __GNUC__  ) /* GCC CS3 */
	#include <syscalls.h> /** RedHat Newlib minimal stub */
#endif

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

#define digitalPinToPort(P)        ( g_APinDescription[P].ulPort )
#define digitalPinToBitMask(P)     ( g_APinDescription[P].ulPin )

//defines here the type of the pin
#define GPIO_PIN_MORPHO_ONLY          0x00000000
#define GPIO_PIN_ADC                  0x00000001
#define GPIO_PIN_DAC                  0x00000002
#define GPIO_PIN_PWM                  0x00000004
#define GPIO_PIN_IO                   0x00000008
#define GPIO_PIN_I2C_SCL              0x00000010
#define GPIO_PIN_I2C_SDA              0x00000020
#define GPIO_PIN_SPI_SCK              0x00000100
#define GPIO_PIN_SPI_MISO             0x00000200
#define GPIO_PIN_SPI_MOSI             0x00000400
#define GPIO_PIN_SPI_CS               0x00000800
#define GPIO_PIN_UART_TX              0x00001000
#define GPIO_PIN_UART_RX              0x00002000

//defines here the ardduino IOs type
//digital pins
#define ARDUINO_PIN_D0                0x00000000
#define ARDUINO_PIN_D1                0x00000001
#define ARDUINO_PIN_D2                0x00000002
#define ARDUINO_PIN_D3                0x00000003
#define ARDUINO_PIN_D4                0x00000004
#define ARDUINO_PIN_D5                0x00000005
#define ARDUINO_PIN_D6                0x00000006
#define ARDUINO_PIN_D7                0x00000007
#define ARDUINO_PIN_D8                0x00000008
#define ARDUINO_PIN_D9                0x00000009
#define ARDUINO_PIN_D10               0x0000000A
#define ARDUINO_PIN_D11               0x0000000B
#define ARDUINO_PIN_D12               0x0000000C
#define ARDUINO_PIN_D13               0x0000000D
#define ARDUINO_PIN_D14               0x0000000E
#define ARDUINO_PIN_D15               0x0000000F

#define MAX_DIGITAL_IOS               22 //All pins can be use as digital pins

//analog pins
#define ARDUINO_PIN_A0                0x00000010
#define ARDUINO_PIN_A1                0x00000011
#define ARDUINO_PIN_A2                0x00000012
#define ARDUINO_PIN_A3                0x00000013
#define ARDUINO_PIN_A4                0x00000014
#define ARDUINO_PIN_A5                0x00000015
//D13 can be used as a DAC
#define MAX_ANALOG_IOS                7


//Morpho connector IOS
#define ARDUINO_PIN_EXT               0x10000000


//Analog pins
static const uint8_t A0  = ARDUINO_PIN_A0;
static const uint8_t A1  = ARDUINO_PIN_A1;
static const uint8_t A2  = ARDUINO_PIN_A2;
static const uint8_t A3  = ARDUINO_PIN_A3;
static const uint8_t A4  = ARDUINO_PIN_A4;
static const uint8_t A5  = ARDUINO_PIN_A5;

//On-board LED pin number
#define LED_BUILTIN                   4

//ADC resolution is 12bits
#define ADC_RESOLUTION                10

//PWR resolution
#define PWM_RESOLUTION                ((uint8_t)8)
#define PWM_FREQUENCY                 ((uint16_t)1000)
#define PWM_MAX_DUTY_CYCLE            ((uint8_t)255)


//SPI defintions
//define 16 channels. As many channel as digital IOs
#define SPI_CHANNELS_NUM        16

//default chip salect pin
#define BOARD_SPI_DEFAULT_SS    10

//In case SPI CS channel is not used we define a default one
#define BOARD_SPI_OWN_SS        SPI_CHANNELS_NUM

#define SPI_INTERFACES_COUNT    1

static const uint8_t SS   = BOARD_SPI_DEFAULT_SS;
static const uint8_t SS1  = 4;
static const uint8_t SS2  = 7;
static const uint8_t SS3  = 8;
static const uint8_t MOSI = 11;
static const uint8_t MISO = 12;
static const uint8_t SCK  = 13;

//Enable Firmata
#ifndef STM8S105
#define STM8S105 1
#endif

#ifdef __cplusplus
}
#endif // __cplusplus


/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
extern UARTClass Serial;
#endif

#endif /* _VARIANT_ARDUINO */
