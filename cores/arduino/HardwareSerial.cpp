/*
  HardwareSerial.cpp - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 23 November 2006 by David A. Mellis
  Modified 28 September 2010 by Mark Sproul
  Modified 14 August 2012 by Alarus
  Modified 3 December 2013 by Matthijs Kooijman
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <assert.h>
#include "Arduino.h"
#include "HardwareSerial.h"

#if defined(NO_HWSERIAL)
#pragma weak serialEventRun
void serialEventRun(void) {}
#else
#if defined(HAVE_HWSERIAL1) || defined(HAVE_HWSERIAL2) || defined(HAVE_HWSERIAL3) ||\
defined(HAVE_HWSERIAL4)
// SerialEvent functions are weak, so when the user doesn't define them,
// the linker just sets their address to 0 (which is checked below).
#if defined(HAVE_HWSERIAL1)
#if defined(PIN_SERIAL1_RX) && defined(PIN_SERIAL1_TX)
HardwareSerial Serial1(PIN_SERIAL1_RX, PIN_SERIAL1_TX);
#else
HardwareSerial Serial1(USART1);
#endif
#pragma weak serialEvent1
void serialEvent1(void) {}
#endif

#if defined(HAVE_HWSERIAL2)
#if defined(PIN_SERIAL2_RX) && defined(PIN_SERIAL2_TX)
HardwareSerial Serial2(PIN_SERIAL2_RX, PIN_SERIAL2_TX);
#else
HardwareSerial Serial2(USART2);
#endif
#pragma weak serialEvent2
void serialEvent2(void) {}
#endif

#if defined(HAVE_HWSERIAL3)
#if defined(PIN_SERIAL3_RX) && defined(PIN_SERIAL3_TX)
HardwareSerial Serial3(PIN_SERIAL3_RX, PIN_SERIAL3_TX);
#else
HardwareSerial Serial3(USART3);
#endif
#pragma weak serialEvent3
void serialEvent3(void) {}
#endif

#if defined(HAVE_HWSERIAL4)
#if defined(PIN_SERIAL4_RX) && defined(PIN_SERIAL4_TX)
HardwareSerial Serial4(PIN_SERIAL4_RX, PIN_SERIAL4_TX);
#else
HardwareSerial Serial4(USART4);
#endif
#pragma weak serialEvent4
void serialEvent4(void) {}
#endif
#endif // HAVE_HWSERIALx

#pragma weak serialEventRun
void serialEventRun(void)
{
#if defined(HAVE_HWSERIAL1)
  if (serialEvent1 && Serial1.available()) serialEvent1();
#endif
#if defined(HAVE_HWSERIAL2)
  if (serialEvent2 && Serial2.available()) serialEvent2();
#endif
#if defined(HAVE_HWSERIAL3)
  if (serialEvent3 && Serial3.available()) serialEvent3();
#endif
#if defined(HAVE_HWSERIAL4)
  if (serialEvent4 && Serial4.available()) serialEvent4();
#endif
}

// Constructors ////////////////////////////////////////////////////////////////
HardwareSerial::HardwareSerial(uint32_t _rx, uint32_t _tx)
{
  _serial.pin_rx = digitalPinToPinName(_rx);
  _serial.pin_tx = digitalPinToPinName(_tx);
  init();
}

HardwareSerial::HardwareSerial(PinName _rx, PinName _tx)
{
  _serial.pin_rx = _rx;
  _serial.pin_tx = _tx;
  init();
}

HardwareSerial::HardwareSerial(void *peripheral)
{
  // If Serial is defined in variant set
  // the Rx/Tx pins for com port if defined
#if defined(Serial) && defined(PIN_SERIAL_RX) && defined(PIN_SERIAL_TX)
  if (this == &Serial)
  {
    setRx(PIN_SERIAL_RX);
    setTx(PIN_SERIAL_TX);
  }
  else
#endif
#if defined(PIN_SERIAL1_RX) && defined(PIN_SERIAL1_TX) && defined(USART1)
    if (peripheral == USART1)
    {
      setRx(PIN_SERIAL1_RX);
      setTx(PIN_SERIAL1_TX);
    } else
#endif
#if defined(PIN_SERIAL2_RX) && defined(PIN_SERIAL2_TX) && defined(USART2)
    if (peripheral == USART2)
    {
      setRx(PIN_SERIAL2_RX);
      setTx(PIN_SERIAL2_TX);
    } else
#endif
#if defined(PIN_SERIAL3_RX) && defined(PIN_SERIAL3_TX) && defined(USART3)
    if (peripheral == USART3)
    {
      setRx(PIN_SERIAL3_RX);
      setTx(PIN_SERIAL3_TX);
    }else
#endif
#if defined(PIN_SERIAL4_RX) && defined(PIN_SERIAL4_TX) && defined(USART4)
    if (peripheral == USART4)
    {
      setRx(PIN_SERIAL4_RX);
      setTx(PIN_SERIAL4_TX);
    } else
#endif
// else get the pins of the first peripheral occurence in PinMap
  {
    _serial.pin_rx = pinmap_pin(peripheral, PinMap_UART_RX);
    _serial.pin_tx = pinmap_pin(peripheral, PinMap_UART_TX);
  }
  init();
}

void HardwareSerial::init(void)
{
  _serial.rx_buff = _rx_buffer;
  _serial.rx_head = 0;
  _serial.rx_tail = 0;
}

// Actual interrupt handlers //////////////////////////////////////////////////////////////

void HardwareSerial::_rx_complete_irq(serial_t* obj)
{
  // No Parity error, read byte and store it in the buffer if there is room
  unsigned char c;

  if (uart_getc(obj, &c) == 0) {

    rx_buffer_index_t i = (unsigned int)(obj->rx_head + 1) % SERIAL_RX_BUFFER_SIZE;

    // if we should be storing the received character into the location
    // just before the tail (meaning that the head would advance to the
    // current location of the tail), we're about to overflow the buffer
    // and so we don't write the character or advance the head.
    if (i != obj->rx_tail) {
      obj->rx_buff[obj->rx_head] = c;
      obj->rx_head = i;
    }
  }
}

// Public Methods //////////////////////////////////////////////////////////////

void HardwareSerial::begin(unsigned long baud, byte config)
{
  uint32_t databits = 0;

  _serial.baudrate = (uint32_t)baud;

  // Manage databits
  switch(config & 0x07) {
    case 0x02:
      databits = 6;
      break;
    case 0x04:
      databits = 7;
      break;
    case 0x06:
      databits = 8;
      break;
    default:
      databits = 0;
      break;
	}

  if((config & 0x30) == 0x30) {
    _serial.parity = UART_PARITY_ODD;
    databits++;
  } else if((config & 0x20) == 0x20) {
    _serial.parity = UART_PARITY_EVEN;
    databits++;
  } else {
    _serial.parity = UART_PARITY_NONE;
  }

  if((config & 0x08) == 0x08) {
    _serial.stopbits = UART_STOPBITS_2;
  } else {
    _serial.stopbits = UART_STOPBITS_1;
  }

  switch(databits) {
    case 8:
      _serial.databits = UART_WORDLENGTH_8B;
      break;
    case 9:
      _serial.databits = UART_WORDLENGTH_9B;
      break;
    default:
    case 0:
      databits = 0;
      break;
  }

  uart_init(&_serial);
  uart_attach_rx_callback(&_serial, _rx_complete_irq);
}

void HardwareSerial::end()
{
  // wait for transmission of outgoing data
  flush();

  uart_deinit(&_serial);

  // clear any received data
  _serial.rx_head = _serial.rx_tail;
}

int HardwareSerial::available(void)
{
  return ((unsigned int)(SERIAL_RX_BUFFER_SIZE + _serial.rx_head - _serial.rx_tail)) % SERIAL_RX_BUFFER_SIZE;
}

int HardwareSerial::peek(void)
{
  if (_serial.rx_head == _serial.rx_tail) {
    return -1;
  } else {
    return _serial.rx_buff[_serial.rx_tail];
  }
}

int HardwareSerial::read(void)
{
  // if the head isn't ahead of the tail, we don't have any characters
  if (_serial.rx_head == _serial.rx_tail) {
    return -1;
  } else {
    unsigned char c = _serial.rx_buff[_serial.rx_tail];
    _serial.rx_tail = (rx_buffer_index_t)(_serial.rx_tail + 1) % SERIAL_RX_BUFFER_SIZE;
    return c;
  }
}

void HardwareSerial::flush()
{

}

size_t HardwareSerial::write(uint8_t c)
{
    uart_write(&_serial, c);
    return 1;
}

void HardwareSerial::setRx(uint32_t _rx) {
  _serial.pin_rx = digitalPinToPinName(_rx);
}

void HardwareSerial::setTx(uint32_t _tx) {
  _serial.pin_tx = digitalPinToPinName(_tx);
}

void HardwareSerial::setRx(PinName _rx) {
  _serial.pin_rx = _rx;
}

void HardwareSerial::setTx(PinName _tx){
  _serial.pin_tx = _tx;
}

#endif // !NO_HWSERIAL
