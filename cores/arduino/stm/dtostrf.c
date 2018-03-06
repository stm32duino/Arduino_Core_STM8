/*
  dtostrf - Emulation for dtostrf function from avr-libc
  Copyright (c) 2013 Arduino.  All rights reserved.
  Written by Cristian Maglie <c.maglie@arduino.cc>

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

#include <stdio.h>

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

char *dtostrf (double val, signed char width, unsigned char prec, char *sout) {
  //Commented code is the original version
  /*char fmt[20];
  sprintf(fmt, "%%%d.%df", width, prec);
  sprintf(sout, fmt, val);
  return sout;*/

  // Handle negative numbers
  unsigned int negative = 0;
  if (val < 0.0)
  {
     negative = 1;
     val = -val;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  int i;
  for (i=0; i<prec; ++i)
    rounding /= 10.0;

  val += rounding;

  // Extract the integer part of the number
  unsigned long int_part = (unsigned long)val;
  double remainder = val - (double)int_part;

  if(negative)
    int_part = -int_part;

  // Extract digits from the remainder
  unsigned long dec_part = 0;
  double decade = 1.0;
  for(i=0; i < prec; i++)
    decade *= 10.0;
  remainder *= decade;
  dec_part = (int)remainder;

  sprintf(sout, "%ld.%ld", int_part, dec_part);

  return sout;
}

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus
