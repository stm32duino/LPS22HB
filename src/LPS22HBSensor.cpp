/**
 ******************************************************************************
 * @file    LPS22HBSensor.cpp
 * @author  AST
 * @version V1.0.0
 * @date    7 September 2017
 * @brief   Implementation of a LPS22HB Pressure sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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


/* Includes ------------------------------------------------------------------*/

#include "Arduino.h"
#include "Wire.h"
#include "LPS22HBSensor.h"


/* Class Implementation ------------------------------------------------------*/

/** Constructor
 * @param i2c object of an helper class which handles the I2C peripheral
 * @param address the address of the component's instance
 */
LPS22HBSensor::LPS22HBSensor(TwoWire *i2c) : dev_i2c(i2c)
{
  address = LPS22HB_ADDRESS_HIGH;

  if ( LPS22HB_Set_PowerMode( (void *)this, LPS22HB_LowPower) == LPS22HB_ERROR )
  {
    return;
  }

  /* Power down the device */
  if ( LPS22HB_Set_Odr( (void *)this, LPS22HB_ODR_ONE_SHOT ) == LPS22HB_ERROR )
  {
    return;
  }

  /* Disable low-pass filter on LPS22HB pressure data */
  if( LPS22HB_Set_LowPassFilter( (void *)this, LPS22HB_DISABLE) == LPS22HB_ERROR )
  {
    return;
  }

  /* Set low-pass filter cutoff configuration*/
  if( LPS22HB_Set_LowPassFilterCutoff( (void *)this, LPS22HB_ODR_9) == LPS22HB_ERROR )
  {
    return;
  }

  /* Set block data update mode */
  if ( LPS22HB_Set_Bdu( (void *)this, LPS22HB_BDU_NO_UPDATE ) == LPS22HB_ERROR )
  {
    return;
  }

  /* Set automatic increment for multi-byte read/write */
  if( LPS22HB_Set_AutomaticIncrementRegAddress( (void *)this, LPS22HB_ENABLE) == LPS22HB_ERROR )
  {
    return;
  }

  isEnabled = 0;
  Last_ODR = 25.0f;
};


/** Constructor
 * @param i2c object of an helper class which handles the I2C peripheral
 * @param address the address of the component's instance
 */
LPS22HBSensor::LPS22HBSensor(TwoWire *i2c, uint8_t address) : dev_i2c(i2c), address(address)
{
  if ( LPS22HB_Set_PowerMode( (void *)this, LPS22HB_LowPower) == LPS22HB_ERROR )
  {
    return;
  }

  /* Power down the device */
  if ( LPS22HB_Set_Odr( (void *)this, LPS22HB_ODR_ONE_SHOT ) == LPS22HB_ERROR )
  {
    return;
  }

  /* Disable low-pass filter on LPS22HB pressure data */
  if( LPS22HB_Set_LowPassFilter( (void *)this, LPS22HB_DISABLE) == LPS22HB_ERROR )
  {
    return;
  }

  /* Set low-pass filter cutoff configuration*/
  if( LPS22HB_Set_LowPassFilterCutoff( (void *)this, LPS22HB_ODR_9) == LPS22HB_ERROR )
  {
    return;
  }

  /* Set block data update mode */
  if ( LPS22HB_Set_Bdu( (void *)this, LPS22HB_BDU_NO_UPDATE ) == LPS22HB_ERROR )
  {
    return;
  }

  /* Disable automatic increment for multi-byte read/write */
  if( LPS22HB_Set_AutomaticIncrementRegAddress( (void *)this, LPS22HB_DISABLE) == LPS22HB_ERROR )
  {
    return;
  }

  isEnabled = 0;
  Last_ODR = 25.0f;
};


/**
 * @brief  Enable LPS22HB
 * @retval LPS22HB_STATUS_OK in case of success, an error code otherwise
 */
LPS22HBStatusTypeDef LPS22HBSensor::Enable(void)
{
  /* Check if the component is already enabled */
  if ( isEnabled == 1 )
  {
    return LPS22HB_STATUS_OK;
  }

  if(SetODR_When_Enabled(Last_ODR) == LPS22HB_STATUS_ERROR)
  {
    return LPS22HB_STATUS_ERROR;
  }

  isEnabled = 1;

  return LPS22HB_STATUS_OK;
}

/**
 * @brief  Disable LPS22HB
 * @retval LPS22HB_STATUS_OK in case of success, an error code otherwise
 */
LPS22HBStatusTypeDef LPS22HBSensor::Disable(void)
{
  /* Check if the component is already disabled */
  if ( isEnabled == 0 )
  {
    return LPS22HB_STATUS_OK;
  }

  /* Power down the device */
  if ( LPS22HB_Set_Odr( (void *)this, LPS22HB_ODR_ONE_SHOT ) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  isEnabled = 0;

  return LPS22HB_STATUS_OK;
}

/**
 * @brief  Read ID address of LPS22HB
 * @param  ht_id the pointer where the ID of the device is stored
 * @retval LPS22HB_STATUS_OK in case of success, an error code otherwise
 */
LPS22HBStatusTypeDef LPS22HBSensor::ReadID(uint8_t *p_id)
{
  if(!p_id)
  {
    return LPS22HB_STATUS_ERROR;
  }

  /* Read WHO AM I register */
  if ( LPS22HB_Get_DeviceID( (void *)this, p_id ) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  return LPS22HB_STATUS_OK;
}

/**
 * @brief  Reboot memory content of LPS22HB
 * @param  None
 * @retval LPS22HB_STATUS_OK in case of success, an error code otherwise
 */
LPS22HBStatusTypeDef LPS22HBSensor::Reset(void)
{
  /* Read WHO AM I register */
  if ( LPS22HB_MemoryBoot((void *)this) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  return LPS22HB_STATUS_OK;
}

/**
 * @brief  Read LPS22HB output register, and calculate the pressure in mbar
 * @param  pfData the pressure value in mbar
 * @retval LPS22HB_STATUS_OK in case of success, an error code otherwise
 */
LPS22HBStatusTypeDef LPS22HBSensor::GetPressure(float* pfData)
{
  int32_t int32data = 0;

  /* Read data from LPS22HB. */
  if ( LPS22HB_Get_Pressure( (void *)this, &int32data ) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  *pfData = ( float )int32data / 100.0f;

  return LPS22HB_STATUS_OK;
}

/**
 * @brief  Read LPS22HB output register, and calculate the temperature
 * @param  pfData the temperature value
 * @retval LPS22HB_STATUS_OK in case of success, an error code otherwise
 */
LPS22HBStatusTypeDef LPS22HBSensor::GetTemperature(float *pfData)
{
  int16_t int16data = 0;

  /* Read data from LPS22HB. */
  if ( LPS22HB_Get_Temperature( (void *)this, &int16data ) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  *pfData = ( float )int16data / 10.0f;

  return LPS22HB_STATUS_OK;
}

/**
 * @brief  Read LPS22HB output data rate
 * @param  odr the pointer to the output data rate
 * @retval LPS22HB_STATUS_OK in case of success, an error code otherwise
 */
LPS22HBStatusTypeDef LPS22HBSensor::GetODR(float* odr)
{
  LPS22HB_Odr_et odr_low_level;

  if ( LPS22HB_Get_Odr( (void *)this, &odr_low_level ) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  switch( odr_low_level )
  {
    case LPS22HB_ODR_ONE_SHOT:
      *odr = 0.0f;
      break;
    case LPS22HB_ODR_1HZ:
      *odr = 1.0f;
      break;
    case LPS22HB_ODR_10HZ:
      *odr = 10.0f;
      break;
    case LPS22HB_ODR_25HZ:
      *odr = 25.0f;
      break;
    case LPS22HB_ODR_50HZ:
      *odr = 50.0f;
      break;
    case LPS22HB_ODR_75HZ:
      *odr = 75.0f;
      break;
    default:
      *odr = -1.0f;
      return LPS22HB_STATUS_ERROR;
  }

  return LPS22HB_STATUS_OK;
}

/**
 * @brief  Set ODR
 * @param  odr the output data rate to be set
 * @retval LPS22HB_STATUS_OK in case of success, an error code otherwise
 */
LPS22HBStatusTypeDef LPS22HBSensor::SetODR(float odr)
{
  if(isEnabled == 1)
  {
    if(SetODR_When_Enabled(odr) == LPS22HB_STATUS_ERROR)
    {
      return LPS22HB_STATUS_ERROR;
    }
  }
  else
  {
    if(SetODR_When_Disabled(odr) == LPS22HB_STATUS_ERROR)
    {
      return LPS22HB_STATUS_ERROR;
    }
  }

  return LPS22HB_STATUS_OK;
}


/**
 * @brief Set the LPS22HB sensor output data rate when enabled
 * @param odr the functional output data rate to be set
 * @retval LPS22HB_STATUS_OK in case of success
 * @retval LPS22HB_STATUS_ERROR in case of failure
 */
LPS22HBStatusTypeDef LPS22HBSensor::SetODR_When_Enabled( float odr )
{
  LPS22HB_Odr_et new_odr;

  new_odr = ( odr <=  1.0f ) ? LPS22HB_ODR_1HZ
          : ( odr <= 10.0f ) ? LPS22HB_ODR_10HZ
          : ( odr <= 25.0f ) ? LPS22HB_ODR_25HZ
          : ( odr <= 50.0f ) ? LPS22HB_ODR_50HZ
          :                    LPS22HB_ODR_75HZ;

  if ( LPS22HB_Set_Odr( (void *)this, new_odr ) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  if ( GetODR( &Last_ODR ) == LPS22HB_STATUS_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  return LPS22HB_STATUS_OK;
}

/**
 * @brief Set the LPS22HB sensor output data rate when disabled
 * @param odr the functional output data rate to be set
 * @retval LPS22HB_STATUS_OK in case of success
 * @retval LPS22HB_STATUS_ERROR in case of failure
 */
LPS22HBStatusTypeDef LPS22HBSensor::SetODR_When_Disabled( float odr )
{
  Last_ODR = ( odr <=  1.0f ) ? 1.0f
           : ( odr <= 10.0f ) ? 10.0f
           : ( odr <= 25.0f ) ? 25.0f
           : ( odr <= 50.0f ) ? 50.0f
           :                    75.0f;

  return LPS22HB_STATUS_OK;
}


/**
 * @brief Read the data from register
 * @param reg register address
 * @param data register data
 * @retval LPS22HB_STATUS_OK in case of success
 * @retval LPS22HB_STATUS_ERROR in case of failure
 */
LPS22HBStatusTypeDef LPS22HBSensor::ReadReg( uint8_t reg, uint8_t *data )
{

  if ( LPS22HB_ReadReg( (void *)this, reg, 1, data ) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  return LPS22HB_STATUS_OK;
}

/**
 * @brief Write the data to register
 * @param reg register address
 * @param data register data
 * @retval LPS22HB_STATUS_OK in case of success
 * @retval LPS22HB_STATUS_ERROR in case of failure
 */
LPS22HBStatusTypeDef LPS22HBSensor::WriteReg( uint8_t reg, uint8_t data )
{

  if ( LPS22HB_WriteReg( (void *)this, reg, 1, &data ) == LPS22HB_ERROR )
  {
    return LPS22HB_STATUS_ERROR;
  }

  return LPS22HB_STATUS_OK;
}


uint8_t LPS22HB_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite )
{
  return ((LPS22HBSensor *)handle)->IO_Write(pBuffer, WriteAddr, nBytesToWrite);
}

uint8_t LPS22HB_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead )
{
  return ((LPS22HBSensor *)handle)->IO_Read(pBuffer, ReadAddr, nBytesToRead);
}
