/**
******************************************************************************
* @file    spi_com.c
* @author  WI6LABS
* @version V1.0.0
* @date
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
/** @addtogroup stm8sxx_system
* @{
*/

/** @addtogroup stm8sxx_System_Private_Includes
* @{
*/
#include "stm8_def.h"
#include "spi_com.h"

#if defined(STM8Lxx)
#include "PinAF_STM8Lxx.h"
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#if defined(STM8Lxx)
#define SPI_ClockPolarity_TypeDef SPI_CPOL_TypeDef
#define SPI_ClockPhase_TypeDef SPI_CPHA_TypeDef
#define SPI_FIRSTBIT_LSB SPI_FirstBit_LSB
#define SPI_FIRSTBIT_MSB SPI_FirstBit_MSB
#define SPI_BAUDRATEPRESCALER_2 SPI_BaudRatePrescaler_2
#define SPI_BAUDRATEPRESCALER_4 SPI_BaudRatePrescaler_4
#define SPI_BAUDRATEPRESCALER_8 SPI_BaudRatePrescaler_8
#define SPI_BAUDRATEPRESCALER_16 SPI_BaudRatePrescaler_16
#define SPI_BAUDRATEPRESCALER_32 SPI_BaudRatePrescaler_32
#define SPI_BAUDRATEPRESCALER_64 SPI_BaudRatePrescaler_64
#define SPI_BAUDRATEPRESCALER_128 SPI_BaudRatePrescaler_128
#define SPI_BAUDRATEPRESCALER_256 SPI_BaudRatePrescaler_256
#define SPI_CLOCKPOLARITY_LOW SPI_CPOL_Low
#define SPI_CLOCKPOLARITY_HIGH SPI_CPOL_High
#define SPI_CLOCKPHASE_1EDGE SPI_CPHA_1Edge
#define SPI_CLOCKPHASE_2EDGE SPI_CPHA_2Edge
#define SPI_DATADIRECTION_2LINES_FULLDUPLEX SPI_Direction_2Lines_FullDuplex
#define SPI_NSS_SOFT SPI_NSS_Soft
#endif

/**
* @brief  SPI initialization function
* @param  obj : pointer to spi_t structure
* @param  speed : spi output speed
* @param  mode : one of the spi modes
* @param  msb : set to 1 in msb first
* @retval None
*/
void spi_init(spi_t *obj, uint32_t speed, spi_mode_e mode, uint8_t msb)
{
  if (obj == NULL)
    return;

  SPI_FirstBit_TypeDef FirstBit;
  SPI_BaudRatePrescaler_TypeDef BaudRatePrescaler;
  SPI_ClockPolarity_TypeDef ClockPolarity;
  SPI_ClockPhase_TypeDef ClockPhase;
  uint16_t function = pinmap_function(obj->pin_miso, PinMap_SPI_MISO);

  // Determine the SPI to use
  SPI_TypeDef *spi_mosi = pinmap_peripheral(obj->pin_mosi, PinMap_SPI_MOSI);
#if defined(STM8Lxx)
  SPI_TypeDef *spi_instance = spi_mosi;
#endif
  SPI_TypeDef *spi_miso = pinmap_peripheral(obj->pin_miso, PinMap_SPI_MISO);
  SPI_TypeDef *spi_sclk = pinmap_peripheral(obj->pin_sclk, PinMap_SPI_SCLK);
  SPI_TypeDef *spi_ssel = pinmap_peripheral(obj->pin_ssel, PinMap_SPI_SSEL);

  /* Pins MOSI/MISO/SCLK must not be NP. ssel can be NP. */
  if(spi_mosi == NP || spi_miso == NP || spi_sclk == NP) {
    return;
  }

  SPI_TypeDef *spi_data = pinmap_merge_peripheral(spi_mosi, spi_miso);
  SPI_TypeDef *spi_cntl = pinmap_merge_peripheral(spi_sclk, spi_ssel);

  // Are all pins connected to the same SPI instance?
  if((spi_data == NP) | (spi_cntl == NP)){
    return;
  }

#if defined(STM8Lxx)
  SPI_DeInit(spi_instance);
#elif defined(STM8Sxx)
  SPI_DeInit();
#endif

  if (msb == 0)
  {
    FirstBit = SPI_FIRSTBIT_LSB;
  }
  else
  {
    FirstBit = SPI_FIRSTBIT_MSB;
  }

  if (speed >= SPI_SPEED_CLOCK_DIV8_MHZ)
  {
    BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  }
  else if (speed >= SPI_SPEED_CLOCK_DIV4_MHZ)
  {
    BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  }
  else if (speed >= SPI_SPEED_CLOCK_DIV2_MHZ)
  {
    BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  }
  else if (speed >= SPI_SPEED_CLOCK_DIV1_MHZ)
  {
    BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  }
  else if (speed >= SPI_SPEED_CLOCK_DIV500_KHZ)
  {
    BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  }
  else if (speed >= SPI_SPEED_CLOCK_DIV250_KHZ)
  {
    BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  }
  else if (speed >= SPI_SPEED_CLOCK_DIV125_KHZ)
  {
    BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  }
  else
  {
    BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  }

  if ((mode == SPI_MODE_0) || (mode == SPI_MODE_1))
  {
    ClockPolarity = SPI_CLOCKPOLARITY_LOW;
  }
  else
  {
    ClockPolarity = SPI_CLOCKPOLARITY_HIGH;
  }

  if ((mode == SPI_MODE_0) || (mode == SPI_MODE_2))
  {
    ClockPhase = SPI_CLOCKPHASE_1EDGE;
  }
  else
  {
    ClockPhase = SPI_CLOCKPHASE_2EDGE;
  }

#if defined(STM8Lxx)

  if(spi_instance == SPI1) {
    CLK_PeripheralClockConfig(CLK_Peripheral_SPI1, ENABLE);
  } else if(spi_instance == SPI2) {
    CLK_PeripheralClockConfig(CLK_Peripheral_SPI2, ENABLE);
  }

  pin_Set8LAFPin(STM_PIN_AFNUM(function));

  //Set the MOSI, MISO and SCk at high Level.
  GPIO_ExternalPullUpConfig(get_GPIO_Port(STM_PORT(obj->pin_miso)),
                             STM_GPIO_PIN(obj->pin_miso),
                             ENABLE);
  GPIO_ExternalPullUpConfig(get_GPIO_Port(STM_PORT(obj->pin_mosi)),
                             STM_GPIO_PIN(obj->pin_mosi),
                             ENABLE);
  GPIO_ExternalPullUpConfig(get_GPIO_Port(STM_PORT(obj->pin_sclk)),
                             STM_GPIO_PIN(obj->pin_sclk),
                             ENABLE);

  // SPI Configuration Init.
  SPI_Init(spi_instance, FirstBit, BaudRatePrescaler, SPI_Mode_Master,
            ClockPolarity, ClockPhase, SPI_DATADIRECTION_2LINES_FULLDUPLEX,
            SPI_NSS_SOFT, 7);
  SPI_Cmd(spi_instance, ENABLE);

#elif defined(STM8Sxx)
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_SPI, ENABLE);
  SPI_Init(FirstBit, BaudRatePrescaler, SPI_MODE_MASTER, ClockPolarity,
           ClockPhase, SPI_DATADIRECTION_2LINES_FULLDUPLEX, SPI_NSS_SOFT, 7);
  SPI_Cmd(ENABLE);
#endif
}

/**
* @brief This function is implemented to deinitialize the SPI interface
*        (IOs + SPI block)
* @param  obj : pointer to spi_t structure
* @retval None
*/
void spi_deinit(spi_t *obj)
{
#if defined(STM8Lxx)
  SPI_TypeDef *spi_instance;
#endif
  if (obj == NULL)
  {
    return;
  }
#if defined(STM8Lxx)
  spi_instance = pinmap_peripheral(obj->pin_mosi, PinMap_SPI_MOSI);
  SPI_DeInit(spi_instance);
#elif defined(STM8Sxx)
  SPI_DeInit();
#endif
}

/**
* @brief This function is implemented by user to send data over SPI interface
* @param  obj : pointer to spi_t structure
* @param  Data : data to be sent
* @param  len : length in bytes of the data to be sent
* @retval status of the send operation (0) in case of error
*/
spi_status_e spi_send(spi_t *obj, uint8_t *Data, uint16_t len)
{
#if defined(STM8Lxx)  
  SPI_TypeDef *spi_instance;
#endif
  spi_status_e ret = SPI_OK;
  int i = 0;
  uint16_t tx_len = len;
  if ((obj == NULL) || (tx_len == 0))
  {
    return SPI_ERROR;
  }
#if defined(STM8Lxx)

  spi_instance = pinmap_peripheral(obj->pin_mosi, PinMap_SPI_MOSI);
  while (tx_len > 0)
  {
    while (SPI_GetFlagStatus(spi_instance, SPI_FLAG_TXE) == RESET)
      ;
    SPI_SendData(spi_instance, Data[i++]);
    tx_len--;
  }

#elif defined(STM8Sxx)
  while (tx_len > 0)
  {
    while (SPI_GetFlagStatus(SPI_FLAG_TXE) == RESET)
      ;
    SPI_SendData(Data[i++]);
    tx_len--;
  }
#endif
  return SPI_OK;
}

/**
* @brief This function is implemented by user to send/receive data over
*         SPI interface
* @param  obj : pointer to spi_t structure
* @param  tx_buffer : tx data to send before reception
* @param  rx_buffer : data to receive
* @param  len : length in byte of the data to send and receive
* @retval status of the send operation (0) in case of error
*/
spi_status_e spi_transfer(spi_t *obj, uint8_t *tx_buffer,
                          uint8_t *rx_buffer, uint16_t len)
{
#if defined(STM8Lxx)  
  SPI_TypeDef *spi_instance;
#endif
  spi_status_e ret = SPI_OK;
  int i = 0;
  uint16_t tx_len = len;
  if ((obj == NULL) || (tx_len == 0))
  {
    return SPI_ERROR;
  }

#if defined(STM8Lxx)
  spi_instance = pinmap_peripheral(obj->pin_mosi, PinMap_SPI_MOSI);
  
  
  while (tx_len > 0)
  {
    while (SPI_GetFlagStatus(spi_instance, SPI_FLAG_TXE) == RESET);
    SPI_SendData(spi_instance, tx_buffer[i]);

    while (SPI_GetFlagStatus(spi_instance, SPI_FLAG_RXNE) == RESET);
    rx_buffer[i] = SPI_ReceiveData(spi_instance);

    i++;
    tx_len--;
  }

#elif defined(STM8Sxx)
  while (tx_len > 0)
  {
    while (SPI_GetFlagStatus(SPI_FLAG_TXE) == RESET);
    SPI_SendData(tx_buffer[i]);

    while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET);
    rx_buffer[i] = SPI_ReceiveData();

    i++;
    tx_len--;
  }
#endif
  return SPI_OK;
}

#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
