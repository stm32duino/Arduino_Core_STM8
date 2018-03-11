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

#include "variant.h"

/* GPIO ID    | label
  PA0         |
  PA1         |
  PA2         |
  PA3         |
  PA4         |
  PA5         |
  PA6         |
  PA7		  |
  PB0         |
  PB1         |
  PB2         |
  PB3         | A5
  PB4         | A4
  PB5         |
  PB6         | A0
  PB7         | A1
  PC0         |
  PC1         |
  PC2         | D5/PWM
  PC3         |
  PC4         | D9/PWM
  PC5         | D13/SCK
  PC6         | D11/MOSI
  PC7		  | D12/MISO
  PD0		  | D4
  PD1		  |
  PD2		  | D7
  PD3		  | D3/PWM
  PD4		  | D6/PWM
  PD5		  | D1/TX
  PD6		  | D0/RX
  PD7		  | D8
  PE0		  |
  PE1		  | D15/SCL
  PE2		  | D14/SDA
  PE3		  | D2
  PE4		  |
  PE5		  | D10/CS
  PE6		  | A3
  PE7		  | A2
  PG0		  |
  PG1		  |

*/

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Pins descriptions
 */
extern const PinDescription g_APinDescription[]=

//  arduino_id  |     ulPin    |   ulPort | mode               |          configured
{
  { ARDUINO_PIN_D0,   	GPIO_PIN_6,  GPIOD, GPIO_PIN_IO|GPIO_PIN_UART_RX,   false },
  { ARDUINO_PIN_D1,   	GPIO_PIN_5,  GPIOD, GPIO_PIN_IO|GPIO_PIN_UART_TX,   false },
  { ARDUINO_PIN_D2,   	GPIO_PIN_3,  GPIOE, GPIO_PIN_IO,                    false },
  { ARDUINO_PIN_D3,  	  GPIO_PIN_3,  GPIOD, GPIO_PIN_IO|GPIO_PIN_PWM,       false },
  { ARDUINO_PIN_D5,  	  GPIO_PIN_2,  GPIOC, GPIO_PIN_IO|GPIO_PIN_PWM,       false },
  { ARDUINO_PIN_D4,   	GPIO_PIN_0,  GPIOD, GPIO_PIN_IO,                    false },
  { ARDUINO_PIN_D6,  	  GPIO_PIN_4,  GPIOD, GPIO_PIN_IO|GPIO_PIN_PWM,       false },
  { ARDUINO_PIN_D7,   	GPIO_PIN_2,  GPIOD, GPIO_PIN_IO,                    false },
  { ARDUINO_PIN_D8,   	GPIO_PIN_7,  GPIOD, GPIO_PIN_IO,                    false },
  { ARDUINO_PIN_D9,  	  GPIO_PIN_4,  GPIOC, GPIO_PIN_IO|GPIO_PIN_PWM,       false },
  { ARDUINO_PIN_D10,   	GPIO_PIN_5,  GPIOE, GPIO_PIN_IO|GPIO_PIN_SPI_CS,    false },
  { ARDUINO_PIN_D11,   	GPIO_PIN_6,  GPIOC, GPIO_PIN_IO|GPIO_PIN_SPI_MOSI,  false },
  { ARDUINO_PIN_D12,   	GPIO_PIN_7,  GPIOC, GPIO_PIN_IO|GPIO_PIN_SPI_MISO,  false },
  { ARDUINO_PIN_D13,  	GPIO_PIN_5,  GPIOC, GPIO_PIN_IO|GPIO_PIN_SPI_SCK,	  false },
  { ARDUINO_PIN_D14,   	GPIO_PIN_2,  GPIOE, GPIO_PIN_I2C_SDA,               false },
  { ARDUINO_PIN_D15,   	GPIO_PIN_1,  GPIOE, GPIO_PIN_I2C_SCL,               false },
  { ARDUINO_PIN_A0,  	  GPIO_PIN_6,  GPIOB, GPIO_PIN_IO|GPIO_PIN_ADC,     	false },
  { ARDUINO_PIN_A1,  	  GPIO_PIN_7,  GPIOB, GPIO_PIN_IO|GPIO_PIN_ADC,       false },
  { ARDUINO_PIN_A2,   	GPIO_PIN_7,  GPIOE, GPIO_PIN_IO|GPIO_PIN_ADC,       false },
  { ARDUINO_PIN_A3,   	GPIO_PIN_6,  GPIOE, GPIO_PIN_IO|GPIO_PIN_ADC,       false },
  { ARDUINO_PIN_A4,   	GPIO_PIN_4,  GPIOB, GPIO_PIN_IO|GPIO_PIN_ADC,       false },
  { ARDUINO_PIN_A5,   	GPIO_PIN_3,  GPIOB, GPIO_PIN_IO|GPIO_PIN_ADC,       false }
} ;

#ifdef __cplusplus
}
#endif

/*
 * UART objects
 */

UARTClass Serial(UART2_E);    //available on PD5/PD6

#ifdef __CSMC__
__weak void serialEvent(void){}
#else
__weak void serialEvent(void);
#endif

void serialEventRun(void)
{
  if (Serial.available()) serialEvent();
}

// ----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

void __libc_init_array(void);


void init( void )
{
  hw_config_init();
}

#ifdef __cplusplus
}
#endif
