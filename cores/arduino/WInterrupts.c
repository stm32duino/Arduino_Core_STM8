/*
  Copyright (c) 2011-2012 Arduino.  All right reserved.

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

#include "WInterrupts.h"
#include "Arduino.h"

#ifdef __cplusplus
extern "C"
{
#endif

  void attachInterrupt(uint32_t pin, void (*callback)(void), uint32_t mode)
  {

    //not a valid pin
    if (pin > NUM_DIGITAL_PINS)
    {
      return;
    }
    PinName p = digitalPinToPinName(pin);
    GPIO_TypeDef *port = get_GPIO_Port(STM_PORT(p));
#if defined(STM8Sxx)
    EXTI_Sensitivity_TypeDef it_mode;
    switch (mode)
    {

    case CHANGE:
      it_mode = EXTI_SENSITIVITY_RISE_FALL;
      break;

    case FALLING:
      it_mode = EXTI_SENSITIVITY_FALL_ONLY;
      break;

    case LOW:
      it_mode = EXTI_SENSITIVITY_FALL_LOW;
      break;

    case RISING:
    case HIGH:
      it_mode = EXTI_SENSITIVITY_RISE_ONLY;
      break;

    default:
      it_mode = EXTI_SENSITIVITY_RISE_ONLY;
      break;
    }

#elif defined(STM8Lxx)

  EXTI_Trigger_TypeDef it_mode;
  switch (mode)
  {

  case CHANGE:
    it_mode = EXTI_Trigger_Rising_Falling;
    break;

  case FALLING:
    it_mode = EXTI_Trigger_Falling;
    break;

  case LOW:
    it_mode = EXTI_Trigger_Falling_Low;
    break;

  case RISING:
  case HIGH:
    it_mode = EXTI_Trigger_Rising;
    break;

  default:
    it_mode = EXTI_Trigger_Rising;
    break;
  }

#endif
    stm8_interrupt_enable(port, STM_GPIO_PIN(p), callback, it_mode);
  }

  void detachInterrupt(uint32_t pin)
  {
    //not a valid pin
    if (pin > NUM_DIGITAL_PINS)
    {
      return;
    }
    PinName p = digitalPinToPinName(pin);
    GPIO_TypeDef *port = get_GPIO_Port(STM_PORT(p));
    stm8_interrupt_disable(port, STM_GPIO_PIN(p));
  }

#ifdef __cplusplus
}
#endif
