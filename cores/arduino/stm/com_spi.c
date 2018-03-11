/**
  ******************************************************************************
  * @file    com_spi.c
  * @author  WI6LABS
  * @version V1.0.0
  * @date    16-September-2016
  * @brief   provide the SPI interface
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
/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm8sxx_system
  * @{
  */

/** @addtogroup stm8sxx_System_Private_Includes
  * @{
  */
#include "stm8s.h"
#include "com_spi.h"

#ifdef __cplusplus
 extern "C" {
#endif

/**
  * @}
  */

/** @addtogroup stm8sxx_System_Private_Defines
  * @{
  */

/**
  * @}
  */

/** @addtogroup stm8sxx_System_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @addtogroup stm8sxx_System_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup stm8sxx_System_Private_Variables
  * @{
  */

/**
  * @}
  */

/** @addtogroup stm8sxx_System_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/** @addtogroup stm8sxx_System_Private_Functions
  * @{
  */
/**
  * @brief  SPI initialization function
  * @param  spi_id : one of the SPI ids
  * @param  speed : spi output speed
  * @param  mode : one of the spi modes
  * @param  msb : set to 1 in msb first
  * @retval None
  */
void spi_init(spi_instance_e spi_id, spi_speed_e speed, spi_mode_e mode, uint8_t msb)
{
  SPI_FirstBit_TypeDef FirstBit;
  SPI_BaudRatePrescaler_TypeDef BaudRatePrescaler;
  SPI_ClockPolarity_TypeDef ClockPolarity;
  SPI_ClockPhase_TypeDef ClockPhase;

  if(spi_id >= NB_SPI_INSTANCES) {return;}

  SPI_DeInit();

  if(msb == 0) {
    FirstBit = SPI_FIRSTBIT_LSB;
  } else {
    FirstBit = SPI_FIRSTBIT_MSB;
  }

  if(speed >= SPI_SPEED_8MHZ) {
      BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  } else if (speed >= SPI_SPEED_4MHZ) {
      BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  } else if (speed >= SPI_SPEED_2MHZ) {
      BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  } else if (speed >= SPI_SPEED_1MHZ) {
      BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  } else if (speed >= SPI_SPEED_500KHZ) {
      BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  } else if (speed >= SPI_SPEED_250KHZ) {
      BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  } else if (speed >= SPI_SPEED_125KHZ) {
      BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  } else {
      BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  }

  if((mode == SPI_MODE_0)||(mode == SPI_MODE_1)) {
    ClockPolarity = SPI_CLOCKPOLARITY_LOW;
  } else {
    ClockPolarity = SPI_CLOCKPOLARITY_HIGH;
  }

  if((mode == SPI_MODE_0)||(mode == SPI_MODE_2)) {
    ClockPhase = SPI_CLOCKPHASE_1EDGE;
  } else {
    ClockPhase = SPI_CLOCKPHASE_2EDGE;
  }

  SPI_Init(FirstBit, BaudRatePrescaler, SPI_MODE_MASTER, ClockPolarity,
           ClockPhase, SPI_DATADIRECTION_2LINES_FULLDUPLEX, SPI_NSS_SOFT, 7);
  SPI_Cmd(ENABLE);
}

/**
  * @brief This function is implemented to deinitialize the SPI interface
  *        (IOs + SPI block)
  * @param  spi_id : one of the SPI ids
  * @retval None
  */
void spi_deinit(spi_instance_e spi_id)
{
  if(spi_id >= NB_SPI_INSTANCES) {return;}
  SPI_DeInit();
}

/**
  * @brief This function is implemented by user to send data over SPI interface
  * @param  spi_id : one of the SPI ids
  * @param  Data : data to be sent
  * @param  len : length in bytes of the data to be sent
  * @retval status of the send operation (0) in case of error
  */
spi_status_e spi_send(spi_instance_e spi_id, uint8_t *Data, uint16_t len)
{
  if((spi_id >= NB_SPI_INSTANCES) || (len == 0)) {
    return SPI_ERROR;
  }

  while(len > 0) {
    while (SPI_GetFlagStatus(SPI_FLAG_TXE) == RESET);
    SPI_SendData(*Data++);
    len--;
  }

  return SPI_OK;
}

/**
  * @brief This function is implemented by user to send/receive data over
  *         SPI interface
  * @param  spi_id : one of the SPI ids
  * @param  tx_buffer : tx data to send before reception
  * @param  rx_buffer : data to receive
  * @param  len : length in byte of the data to send and receive
  * @retval status of the send operation (0) in case of error
  */
spi_status_e spi_transfer(spi_instance_e spi_id, uint8_t * tx_buffer,
                      uint8_t * rx_buffer, uint16_t len)
{
  if((spi_id >= NB_SPI_INSTANCES) || (len == 0)) {
    return SPI_ERROR;
  }

  while(len > 0) {
    while (SPI_GetFlagStatus(SPI_FLAG_TXE) == RESET);
    SPI_SendData(*tx_buffer++);
    while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET);
    *rx_buffer++ = SPI_ReceiveData();
    len--;
  }

  return SPI_OK;
}


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
