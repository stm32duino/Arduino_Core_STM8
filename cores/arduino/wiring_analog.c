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

#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif


//This is the list of the digital IOs configured
PinDescription g_anInputPinConfigured[MAX_ANALOG_IOS];
PinDescription g_anOutputPinConfigured[MAX_DIGITAL_IOS];


static int _readResolution = 10;
static int _writeResolution = 8;


static inline int32_t get_pin_description(uint32_t apin, uint32_t ulPin)
{
  int32_t i;

  //find the pin.
  for(i = 0; i < NB_PIN_DESCRIPTIONS; i++) {
    if(g_APinDescription[i].arduino_id == ulPin) {
      return i;
    }
  }
  return -1;
}

static inline int32_t pinConvert(uint32_t ulPin)
{
  if(ulPin < ARDUINO_PIN_A0)
    return ulPin | ARDUINO_PIN_A0;
  else
    return ulPin;
}


void analogReadResolution(int res) {
  _readResolution = res;
}

void analogWriteResolution(int res) {
  _writeResolution = res;
}

static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to) {
  if (from == to)
    return value;
  if (from > to)
    return value >> (from-to);
  else
    return value << (to-from);
}


//perform the read operation on the selected analog pin.
//the initialization of the analog PIN is done through this function
uint32_t analogRead(uint32_t ulPin)
{
  uint32_t ulValue = 0;
  uint8_t do_init = 0;

  //put mask only to have the lower digits
  uint32_t apin = ulPin&0x0000000F;
  int i;

  if(ulPin>MAX_DIGITAL_IOS) {
    return 0;
  }

  ulPin = pinConvert(ulPin);

  //find the pin.
  i = get_pin_description(apin, ulPin);
  if((i<0) && (apin < MAX_ANALOG_IOS))
    return 0;

  g_anInputPinConfigured[apin] = g_APinDescription[i];

  if((g_anInputPinConfigured[apin].mode & GPIO_PIN_ADC) == GPIO_PIN_ADC) {
    if(g_anInputPinConfigured[apin].configured == false) {
      do_init = 1;
      g_anInputPinConfigured[apin].configured = true;
    }

    ulValue = adc_read_value(g_anInputPinConfigured[apin].ulPort, g_anInputPinConfigured[apin].ulPin, do_init);

    ulValue = mapResolution(ulValue, ADC_RESOLUTION, _readResolution);
  }

  return ulValue;
}


void analogOutputInit(void) {
}

// Right now, PWM output only works on the pins with
// hardware support.  These are defined in the appropriate
// variant.cpp file.  For the rest of the pins, we default
// to digital output.
void analogWrite(uint32_t ulPin, uint32_t ulValue) {

  //put mask only to have the lower digits
  uint32_t apin = ulPin&0x0000000F;
  uint32_t attr = 0;
  int i;
  uint8_t do_init = 0;

  if(ulPin>MAX_DIGITAL_IOS) {
    return;
  }

  //find the pin.
  i = get_pin_description(apin, ulPin);
  if((i<0) && (apin < MAX_DIGITAL_IOS))
    return;

  g_anOutputPinConfigured[apin] = g_APinDescription[i];

  if(g_anOutputPinConfigured[apin].configured == false) {
    do_init = 1;
    g_anOutputPinConfigured[apin].configured = true;
  }

  attr = g_anOutputPinConfigured[apin].mode;

  if((attr & GPIO_PIN_PWM) == GPIO_PIN_PWM) {

    ulValue = mapResolution(ulValue, _writeResolution, PWM_RESOLUTION);
    pwm_start(g_anOutputPinConfigured[apin].ulPort,
                    g_anOutputPinConfigured[apin].ulPin,
                    (uint32_t)((uint32_t)PWM_FREQUENCY*(uint32_t)PWM_MAX_DUTY_CYCLE),
                    PWM_MAX_DUTY_CYCLE,
                    ulValue, do_init);

  } else { //DIGITAL PIN ONLY
    // Defaults to digital write
    pinMode(ulPin, OUTPUT);
      ulValue = mapResolution(ulValue, _writeResolution, 8);
    if (ulValue < 128) {
      digitalWrite(ulPin, LOW);
    }
    else {
      digitalWrite(ulPin, HIGH);
    }
  }
}

#ifdef __cplusplus
}
#endif
