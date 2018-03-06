/**
  ******************************************************************************
  * @file    twi.c
  * @author  WI6LABS
  * @version V1.0.0
  * @date    16-September-2016
  * @brief   provide the TWI interface
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

#include "twi.h"

/**
  * @}
  */

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup stm8sxx_System_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */

/** @addtogroup stm8sxx_System_Private_Defines
  * @{
  */

/// @brief I2C timout in tick unit
#define I2C_TX_RX_BUFFER_SIZE    32

/**
  * @}
  */

/** @addtogroup stm8sxx_System_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup stm8sxx_System_Private_TypesDefinitions
  * @{
  */

/// @brief defines the global attributes of the I2C
typedef struct {
  uint8_t init_done;
  i2c_instance_e  i2c_instance;
  I2C_AddMode_TypeDef addressingMode;
  uint16_t addr;
  void (*i2c_onSlaveReceive)(i2c_instance_e, uint8_t *, int);
  void (*i2c_onSlaveTransmit)(i2c_instance_e);
  uint8_t i2cTxRxBuffer[I2C_TX_RX_BUFFER_SIZE];
  uint8_t i2cTxRxBufferSize;
  uint8_t txCounter;
  uint8_t slaveMode;
} i2c_init_info_t;

/**
  * @}
  */

/** @addtogroup stm8sxx_System_Private_Variables
  * @{
  */
static i2c_init_info_t g_i2c_init_info[NB_I2C_INSTANCES] = {
  {
    .init_done = 0,
    .i2c_instance = I2C_1,
    .i2c_onSlaveReceive = 0,
    .i2c_onSlaveTransmit = 0,
    .i2cTxRxBufferSize = 0,
    .slaveMode = 0
  }
};

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
  * @brief  Default init and setup GPIO and I2C peripheral
  * @param  i2c_id : i2c instance to use
  * @retval none
  */
void i2c_init(i2c_instance_e i2c_id)
{
  if(i2c_id == I2C_1) {
    I2C_DeInit();
    I2C_Init(I2C_DEFAULT_SPEED, (uint16_t)0x0033, I2C_DUTYCYCLE_2, I2C_ACK_CURR, I2C_ADDMODE_7BIT, (uint8_t)(CLK_GetClockFreq() / 1000000));

    g_i2c_init_info[i2c_id].addressingMode = I2C_ADDMODE_7BIT;
    g_i2c_init_info[i2c_id].addr = (uint16_t)0x0033;
    g_i2c_init_info[i2c_id].slaveMode = 0;
    g_i2c_init_info[i2c_id].init_done = 1;
  }
}

/**
  * @brief  Initialize and setup GPIO and I2C peripheral
  * @param  i2c_id : i2c instance to use
  * @param  timing : one of the i2c_timing_e
  * @param  addressingMode : I2C_ADDRESSINGMODE_7BIT or I2C_ADDRESSINGMODE_10BIT
  * @param  ownAddress : device address
  * @param  master : set to 1 to choose the master mode
  * @retval none
  */
void i2c_custom_init(i2c_instance_e i2c_id, uint32_t timing, I2C_AddMode_TypeDef addressingMode, uint16_t ownAddress, uint8_t master)
{
  if(i2c_id == I2C_1) {
    I2C_DeInit();
    I2C_Init(timing, ownAddress, I2C_DUTYCYCLE_2, I2C_ACK_CURR, addressingMode, (uint8_t)(CLK_GetClockFreq() / 1000000));

    g_i2c_init_info[i2c_id].addressingMode = addressingMode;
    g_i2c_init_info[i2c_id].addr = ownAddress;

    if(master == 0) {
      g_i2c_init_info[i2c_id].slaveMode = 1;

      I2C_ITConfig((I2C_IT_TypeDef)(I2C_IT_ERR | I2C_IT_EVT | I2C_IT_BUF), ENABLE);
      enableInterrupts();
    }

    g_i2c_init_info[i2c_id].init_done = 1;
  }
}

/**
  * @brief  Initialize and setup GPIO and I2C peripheral
  * @param  i2c_id : i2c instance to use
  * @retval none
  */
void i2c_deinit(i2c_instance_e i2c_id)
{
  if(i2c_id == I2C_1) {
    I2C_DeInit();
    g_i2c_init_info[i2c_id].init_done = 0;
  }
}

/**
  * @brief  Setup transmission speed. I2C must be configured before.
  * @param  i2c_id : i2c instance to use
  * @param  frequency : i2c transmission speed
  * @retval none
  */
void i2c_setTiming(i2c_instance_e i2c_id, uint32_t frequency)
{
  if(frequency <= I2C_MAX_FAST_FREQ) {
    if((i2c_id == I2C_1) && (g_i2c_init_info[I2C_1].init_done == 1)) {
      i2c_custom_init(i2c_id, frequency, g_i2c_init_info[i2c_id].addressingMode, g_i2c_init_info[i2c_id].addr, ~g_i2c_init_info[i2c_id].slaveMode);
    }
  }
}

/**
  * @brief  Write bytes at a given address
  * @param  i2c_id : i2c instance to use
  * @param  dev_address: specifies the address of the device.
  * @param  data: pointer to data to be write
  * @param  size: number of bytes to be write.
  * @retval read status
  */
i2c_status_e i2c_master_write(i2c_instance_e i2c_id, uint8_t dev_address,
                        uint8_t *data, uint8_t size)

{
  if((i2c_id == I2C_1) && (g_i2c_init_info[I2C_1].init_done == 1)) {
    /* Send START condition */
    I2C_GenerateSTART(ENABLE);
    while(I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT) == ERROR);

    /* Send slave address + write condition */
    I2C_Send7bitAddress(dev_address, I2C_DIRECTION_TX);
    while (I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == ERROR);

    /* Send data */
    while(size != 0) {
      I2C_SendData(*data++);
      size--;

      if(size > 0) {
        while (I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTING) == ERROR);
      }
    }

    /* Send STOP condition */
    while (I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR);
    I2C_GenerateSTOP(ENABLE);

    return I2C_OK;

  } else {
    return I2C_ERROR;
  }
}

/**
  * @brief  read bytes in master mode at a given address
  * @param  i2c_id : i2c instance to use
  * @param  dev_address: specifies the address of the device.
  * @param  data: pointer to data to be read
  * @param  size: number of bytes to be read.
  * @retval read status
  */
i2c_status_e i2c_master_read(i2c_instance_e i2c_id, uint8_t dev_address,
                              uint8_t *data, uint8_t size)
{
  uint8_t tmpSize = size;
  uint8_t idx = 0;

  if((i2c_id == I2C_1) && (g_i2c_init_info[I2C_1].init_done == 1)) {
    /* Enable ACK */
    I2C_AcknowledgeConfig(I2C_ACK_CURR);

    /* Send START condition */
    I2C_GenerateSTART(ENABLE);
    while(I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT) == ERROR);

    /* Send slave address + read condition */
    I2C_Send7bitAddress(dev_address, I2C_DIRECTION_RX);
    while (I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == ERROR);

    /* Send data */
    while(size != 0) {
      if(tmpSize == 1) {
        /* Disable ACK */
        I2C_AcknowledgeConfig(I2C_ACK_NONE);

        /* Send STOP condition */
        I2C_GenerateSTOP(ENABLE);

        tmpSize = 0;

      } else if(tmpSize == 2) {
        while(I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_RECEIVED) == ERROR);

        /* Disable ACK */
        I2C_AcknowledgeConfig(I2C_ACK_NONE);

        while(I2C_GetFlagStatus(I2C_FLAG_TRANSFERFINISHED) == RESET);

        /* Send STOP condition */
        I2C_GenerateSTOP(ENABLE);

        while(size != 0) {
          data[idx++] = I2C_ReceiveData();
          size--;
        }

        tmpSize = 0;
      } else if((tmpSize > 2) && (size == 3)) {
        while(I2C_GetFlagStatus(I2C_FLAG_TRANSFERFINISHED) == RESET);

        /* Disable ACK */
        I2C_AcknowledgeConfig(I2C_ACK_NONE);

        data[idx++] = I2C_ReceiveData();
        size--;

        /* Send STOP condition */
        I2C_GenerateSTOP(ENABLE);

        data[idx++] = I2C_ReceiveData();
        size--;
        tmpSize = 0;
      }

      /* Read data received */
      if(I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_RECEIVED) == SUCCESS) {
        data[idx++] = I2C_ReceiveData();
        size--;
      }
    }
  } else {
    return I2C_ERROR;
  }

  return I2C_OK;
}

/**
  * @brief  Write bytes to master
  * @param  i2c_id : i2c instance to use
  * @param  data: pointer to data to be write
  * @param  size: number of bytes to be write.
  * @retval none
  */
void i2c_slave_write_IT(i2c_instance_e i2c_id, uint8_t *data, uint8_t size)
{
  uint8_t i = 0;

  if(g_i2c_init_info[i2c_id].init_done == 1) {
    // Check the communication status
    for(i = 0; (i < size) && (i < I2C_TX_RX_BUFFER_SIZE); i++) {
      g_i2c_init_info[i2c_id].i2cTxRxBuffer[i] = *(data+i);
      g_i2c_init_info[i2c_id].i2cTxRxBufferSize++;
    }
  }
}

/** @brief  sets function called before a slave read operation
  * @param  i2c_id : i2c instance to use
  * @param  function: callback function to use
  * @retval None
  */
void i2c_attachSlaveRxEvent(i2c_instance_e i2c_id, void (*function)(i2c_instance_e, uint8_t*, int) )
{
  if((i2c_id == I2C_1) && (g_i2c_init_info[I2C_1].init_done == 1)){
    g_i2c_init_info[i2c_id].i2c_onSlaveReceive = function;
  }
}

/** @brief  sets function called before a slave write operation
  * @param  i2c_id : i2c instance to use
  * @param  function: callback function to use
  * @retval None
  */
void i2c_attachSlaveTxEvent(i2c_instance_e i2c_id, void (*function)(i2c_instance_e) )
{
  if((i2c_id == I2C_1) && (g_i2c_init_info[I2C_1].init_done == 1)) {
    g_i2c_init_info[i2c_id].i2c_onSlaveTransmit = function;
  }
}

/**
  * @brief  Slave printf callback.
  * @param  none
  * @retval None
  */
void I2C_slaveITCallback(void)
{
  __IO uint16_t Event = 0x0000;

  /* Read SR2 register to get I2C error */
  if ((I2C->SR2) != 0) {
    I2C_ClearFlag((I2C_Flag_TypeDef)(I2C_FLAG_OVERRUNUNDERRUN | I2C_FLAG_BUSERROR));
  }

  Event = I2C_GetLastEvent();

  switch (Event) {
      /******* Slave transmitter ******/
      /* check on EV1 */
    case I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED:
      g_i2c_init_info[I2C_1].i2c_onSlaveTransmit(I2C_1);
      g_i2c_init_info[I2C_1].txCounter = 0;
      break;

      /* check on EV3 */
    case I2C_EVENT_SLAVE_BYTE_TRANSMITTING:
      /* Transmit data */
      I2C_SendData(g_i2c_init_info[I2C_1].i2cTxRxBuffer[g_i2c_init_info[I2C_1].txCounter++]);
      break;

      /* check on EV3-2 */
    case I2C_EVENT_SLAVE_ACK_FAILURE:
      /* Clear Ack failure flag */
      I2C_ClearFlag(I2C_FLAG_ACKNOWLEDGEFAILURE);
      break;
      /******* Slave receiver **********/
      /* check on EV1*/
    case I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED:
      I2C_CheckEvent(I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED);
      g_i2c_init_info[I2C_1].i2cTxRxBufferSize = 0;
      break;

      /* Check on EV2*/
    case I2C_EVENT_SLAVE_BYTE_RECEIVED:
      if(g_i2c_init_info[I2C_1].i2cTxRxBufferSize < I2C_TX_RX_BUFFER_SIZE) {
        g_i2c_init_info[I2C_1].i2cTxRxBuffer[g_i2c_init_info[I2C_1].i2cTxRxBufferSize++] = I2C_ReceiveData();
      } else {
        I2C_ReceiveData();
      }
      break;

    default:
      break;
  }

  /* Check on EV4 */
  /* Do not check event EV4 in switch above because flag is never "SUCCESS" */
  if(I2C_GetFlagStatus(I2C_FLAG_STOPDETECTION) == SET) {
    I2C_AcknowledgeConfig(I2C_ACK_CURR);
    if(g_i2c_init_info[I2C_1].i2cTxRxBufferSize < I2C_TX_RX_BUFFER_SIZE) {
      g_i2c_init_info[I2C_1].i2cTxRxBuffer[g_i2c_init_info[I2C_1].i2cTxRxBufferSize++] = I2C_ReceiveData();
    } else {
      I2C_ReceiveData();
    }
    g_i2c_init_info[I2C_1].i2c_onSlaveReceive(
                                  I2C_1,
                                  g_i2c_init_info[I2C_1].i2cTxRxBuffer,
                                  g_i2c_init_info[I2C_1].i2cTxRxBufferSize);
  }
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
