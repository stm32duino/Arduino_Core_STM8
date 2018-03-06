/**
  ******************************************************************************
  * @file    com_spi.h
  * @author  WI6LABS
  * @version V1.0.0
  * @date    16-September-2016
  * @brief   Header for spi module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COM_SPI_H
#define __COM_SPI_H

/* Includes ------------------------------------------------------------------*/

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/

///@brief define the possible SPI instances
typedef enum {
  SPI_1,
  NB_SPI_INSTANCES
}spi_instance_e;

///@brief specifies the SPI speed bus in HZ
typedef enum {
  SPI_SPEED_8MHZ = 8000000,
  SPI_SPEED_4MHZ = 4000000,
  SPI_SPEED_2MHZ = 2000000,
  SPI_SPEED_1MHZ = 1000000,
  SPI_SPEED_500KHZ = 500000,
  SPI_SPEED_250KHZ = 250000,
  SPI_SPEED_125KHZ = 125000
}spi_speed_e;

///@brief speficies the SPI mode to use
//Mode          Clock Polarity (CPOL)       Clock Phase (CPHA)
//SPI_MODE0             0                         0
//SPI_MODE1             0                         1
//SPI_MODE2             1                         0
//SPI_MODE3             1                         1
//enum definitions coming from SPI.h of SAM
typedef enum {
  SPI_MODE_0 = 0x00,
  SPI_MODE_1 = 0x01,
  SPI_MODE_2 = 0x02,
  SPI_MODE_3 = 0x03
}spi_mode_e;

///@brief SPI errors
typedef enum {
  SPI_OK = 0,
  SPI_TIMEOUT = 1,
  SPI_ERROR = 2
}spi_status_e;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void spi_init(spi_instance_e spi_id, spi_speed_e speed, spi_mode_e mode, uint8_t msb);
void spi_deinit(spi_instance_e spi_id);
spi_status_e spi_send(spi_instance_e spi_id, uint8_t *Data, uint16_t len);
spi_status_e spi_transfer(spi_instance_e spi_id, uint8_t * tx_buffer,
                      uint8_t * rx_buffer, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __COM_SPI_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
