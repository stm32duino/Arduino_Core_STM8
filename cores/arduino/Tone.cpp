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

uint8_t g_lastPin = 0xff;


// frequency (in hertz) and duration (in milliseconds).

void tone(uint8_t _pin, unsigned int frequency, unsigned long duration)
{
  uint32_t i = 0;

  if((g_lastPin == 0xff) || (g_lastPin == _pin)) {
    for(i = 0; i < NB_PIN_DESCRIPTIONS; i++) {
      if(g_APinDescription[i].arduino_id == _pin) {
        TimerPinInit(g_APinDescription[i].ulPort, g_APinDescription[i].ulPin, frequency, duration);
        g_lastPin = _pin;
        break;
      }
    }
  }
}


void noTone(uint8_t _pin)
{
  uint32_t i = 0;

  for(i = 0; i < NB_PIN_DESCRIPTIONS; i++) {
    if(g_APinDescription[i].arduino_id == _pin) 
      TimerPinDeinit(g_APinDescription[i].ulPort, g_APinDescription[i].ulPin);
  }
  digitalWrite(_pin, 0);
  g_lastPin = 0xff;
}
