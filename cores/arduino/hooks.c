/*
  Copyright (c) 2012 Arduino.  All right reserved.

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
/**
 * Empty yield() hook.
 *
 * This function is intended to be used by library writers to build
 * libraries or sketches that supports cooperative threads.
 *
 * Its defined as a weak symbol and it can be redefined to implement a
 * real cooperative scheduler.
 */


#pragma weak yield
void yield(void){
  __asm("NOP");
}


/**
 * SysTick hook
 *
 * This function is called from SysTick handler, before the default
 * handler provided by Arduino.
 */
static int __false()
{
  // Return false
  return 0;
}

#ifdef __CSMC__
int sysTickHook(void);
#pragma sysTickHook = __false
#else
int sysTickHook(void) __attribute__((weak, alias("__false")));
#endif

/**
 * SVC hook
 * PendSV hook
 *
 * These functions are called from SVC handler, and PensSV handler.
 * Default action is halting.
 */
static void __halt()
{
  // Halts
  while (1)
    ;
}

#ifdef __CSMC__
void svcHook(void);
#pragma weak svcHook = __halt
void pendSVHook(void);
#pragma weak pendSVHook = __halt
#else
void svcHook(void) __attribute__((weak, alias("__halt")));
void pendSVHook(void) __attribute__((weak, alias("__halt")));
#endif