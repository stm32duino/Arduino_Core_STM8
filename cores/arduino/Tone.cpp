/* Tone.cpp

  A Tone Generator Library

  Written by Brett Hagman

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
*/

#include "Arduino.h"

uint8_t g_lastPin = NUM_DIGITAL_PINS;

// frequency (in hertz) and duration (in milliseconds).

void tone(uint8_t _pin, unsigned int frequency, unsigned long duration)
{

  if ((g_lastPin == NUM_DIGITAL_PINS) || (g_lastPin == _pin))
  {
    TimerPinInit(_pin, frequency, duration);
    g_lastPin = _pin;
  }
}

void noTone(uint8_t _pin)
{
  TimerPinDeinit(_pin);
  digitalWrite((uint32_t)_pin, 0);
  g_lastPin = NUM_DIGITAL_PINS;
}
