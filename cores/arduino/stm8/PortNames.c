/*
 *******************************************************************************
 * Copyright (c) 2017, STMicroelectronics
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
#include "PortNames.h"
#include "stm8_def.h"

GPIO_TypeDef *get_GPIO_Port(uint32_t port_idx)
{
    GPIO_TypeDef *gpioPort = 0;
    switch (port_idx)
    {
    case PortA:
        gpioPort = GPIOA;
        break;
    case PortB:
        gpioPort = GPIOB;
        break;

#if defined(GPIOC_BaseAddress) || defined(GPIOC_BASE)
    case PortC:
        gpioPort = GPIOC;
        break;
#endif
#if defined(GPIOD_BaseAddress) || defined(GPIOD_BASE)
    case PortD:
        gpioPort = GPIOD;
        break;
#endif
#if defined(GPIOE_BaseAddress) || defined(GPIOE_BASE)
    case PortE:
        gpioPort = GPIOE;
        break;
#endif
#if defined(GPIOF_BaseAddress) || defined(GPIOF_BASE)
    case PortF:
        gpioPort = GPIOF;
        break;
#endif
#if defined(GPIOG_BaseAddress) || defined(GPIOG_BASE)
    case PortG:
        gpioPort = GPIOG;
        break;
#endif
#if defined(GPIOH_BaseAddress) || defined(GPIOH_BASE)
    case PortH:
        gpioPort = GPIOH;
        break;
#endif
#if defined(GPIOI_BaseAddress) || defined(GPIOI_BASE)
    case PortI:
        gpioPort = GPIOI;
        break;
#endif
    default:
        // wrong port number
        //TBD: error management
        gpioPort = 0;
        break;
    }
    return gpioPort;
}
