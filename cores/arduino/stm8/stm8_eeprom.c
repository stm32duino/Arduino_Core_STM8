/**
  ******************************************************************************
  * @file    stm8s_eeprom.c
  * @author  WI6LABS
  * @version V1.0.0
  * @date    16-September-2016
  * @brief   provide emulated eeprom from flash
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
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
  *
  ******************************************************************************
  */

#include "stm8_eeprom.h"
#include <string.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
  * @brief  Function read a byte from eeprom
  * @param  __p : address to read
  * @retval byte : data read from eeprom
  */
#if defined(STM8Sxx)
  uint8_t eeprom_read_byte(const uint16_t __p)
  {
    return FLASH_ReadByte((uint32_t)(FLASH_DATA_START_PHYSICAL_ADDRESS + __p));
    ;
  }

  /**
  * @brief  Function write a byte to eeprom
  * @param  __p : address to write
  * @param  __value : value to write
  * @retval none
  */
  void eeprom_write_byte(uint16_t __p, uint8_t __value)
  {
    FLASH_Unlock(FLASH_MEMTYPE_DATA);
    FLASH_ProgramByte((uint32_t)(FLASH_DATA_START_PHYSICAL_ADDRESS + __p), __value);
    FLASH_Lock(FLASH_MEMTYPE_DATA);
  }

#elif defined(STM8Lxx)
uint8_t eeprom_read_byte(const uint16_t __p)
{
  return FLASH_ReadByte((uint32_t)(FLASH_DATA_EEPROM_START_PHYSICAL_ADDRESS + __p));
  ;
}

/**
  * @brief  Function write a byte to eeprom
  * @param  __p : address to write
  * @param  __value : value to write
  * @retval none
  */
void eeprom_write_byte(uint16_t __p, uint8_t __value)
{
  FLASH_Unlock(FLASH_MemType_Data);
  FLASH_ProgramByte((uint32_t)(FLASH_DATA_EEPROM_START_PHYSICAL_ADDRESS + __p), __value);
  FLASH_Lock(FLASH_MemType_Data);
}
#endif

#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
