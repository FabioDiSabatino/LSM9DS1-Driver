/* Includes ------------------------------------------------------------------*/
#include "LSM9DS1_Driver.h"






/** @defgroup LSM9DS1_Private_Functions
* @{
*/

/*******************************************************************************
 * @brief  Read a register of the device through BUS
 * @param  B_Addr Device address on BUS
 * @param  Reg The target register address to read
 * @param  pBuffer The data to be read
 * @param  Size Number of bytes to be read
 * @retval 1 in case of success
 * @retval 0 in case of failure
 *******************************************************************************/
static uint8_t I2C_ReadData(uint8_t B_Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t size)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Read(&hi2c1, B_Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, size, I2C_Timeout);
  /* Check the communication status */
  if( status != HAL_OK )
  {
    /* Execute user timeout callback
    I2C_Error( B_Addr ); */
    return 0;
  }
  else
  {
    return 1;
  }
}

/**
 * @brief  Write data to the register of the device through BUS
 * @param  B_Addr Device address on BUS
 * @param  Reg The target register address to be written
 * @param  pBuffer The data to be written
 * @param  Size Number of bytes to be written
 * @retval 1 in case of success
 * @retval 0 in case of failure
 */
static uint8_t I2C_WriteData(uint8_t B_Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t Size)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Write(&hi2c1, B_Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Size, I2C_Timeout);
  /* Check the communication status */
  if( status != HAL_OK )
  {
    /* Execute user timeout callback */
    // I2C_EXPBD_Error( B_Addr );
    return 0;
  }
  else
  {
    return 1;
  }
}


/****************************************************************************
 * Name: lsm9ds1_midpoint
 *
 * Description:
 *   Find the midpoint between two numbers.
 *
 ****************************************************************************/
static uint32_t lsm9ds1_midpoint(uint32_t a, uint32_t b)
{
  return (uint32_t)(((uint64_t)a + (uint64_t)b + (uint64_t)1) / (uint64_t)2);
}


/****************************************************************************
 * Name: LSM9DS1_XL_SetOdr
 *
 * Description:
 *   Set accelerometer power mode and ODR.
 *
 * Parameter:
 *		int odr: value to set
 *
 * Return:
 * 	    1: set successful
 * 	    0: otherwise
 ****************************************************************************/
static int LSM9DS1_XL_SetOdr(int odr)
{
	uint8_t setbits[1];

		if (odr < lsm9ds1_midpoint(10, 50))
		    {
		      setbits[0] = LSM9DS1_CTRL_REG6_XL_ODR_XL_10HZ;
		    }
		  else if (odr < lsm9ds1_midpoint(50, 119))
		    {
			  setbits[0] = LSM9DS1_CTRL_REG6_XL_ODR_XL_50HZ;
		    }
		  else if (odr < lsm9ds1_midpoint(119, 238))
		    {
			  setbits[0] = LSM9DS1_CTRL_REG6_XL_ODR_XL_119HZ;
		    }
		  else if (odr < lsm9ds1_midpoint(238, 476))
		    {
			  setbits[0] = LSM9DS1_CTRL_REG6_XL_ODR_XL_238HZ;
		    }
		  else if (odr < lsm9ds1_midpoint(476, 952))
		    {
			  setbits[0] = LSM9DS1_CTRL_REG6_XL_ODR_XL_476HZ;
		    }
		  else
		    {
			  setbits[0] = LSM9DS1_CTRL_REG6_XL_ODR_XL_952HZ;
		    }

		 //Set power mode and ODR
		 return I2C_WriteData(LSM9DS1_I2C_BADD_XLG,LSM9DS1_CTRL_REG6_XL,&setbits[0],8);


}


/****************************************************************************
 * Name: LSM9DS1_XL_SetFs
 *
 * Description:
 *   Set accelerometer full scale
 *
 * Parameter:
 *		int fullscale: value to set
 *
 * Return:
 * 	    1: set successful
 * 	    0: otherwise
 ****************************************************************************/
static int LSM9DS1_XL_SetFs(int fullscale)
{
	  uint8_t setbits[1];

	  if (fullscale < lsm9ds1_midpoint(2, 4))
	     {
		  setbits[0] = LSM9DS1_CTRL_REG6_XL_FS_XL_2G;
	     }
	   else if (fullscale < lsm9ds1_midpoint(4, 8))
	     {
		   setbits[0] = LSM9DS1_CTRL_REG6_XL_FS_XL_4G;
	     }
	   else if (fullscale < lsm9ds1_midpoint(8, 16))
	     {
		   setbits[0] = LSM9DS1_CTRL_REG6_XL_FS_XL_8G;
	     }
	   else
	     {
		   setbits[0] = LSM9DS1_CTRL_REG6_XL_FS_XL_16G;
	     }
	  //Set full-scale
	  return I2C_WriteData(LSM9DS1_I2C_BADD_XLG,LSM9DS1_CTRL_REG6_XL,&setbits[0],8);

}


/****************************************************************************
 * Name: lsm9ds1accel_start
 *
 * Description:
 *   Start the accelerometer.
 *
 ****************************************************************************/
 LSM9DS1_XL_START LSM9DS1_XL_Start(int odr, int fs)
{

	if(LSM9DS1_XL_SetOdr(odr))
	{
		if(LSM9DS1_XL_SetFs(fs))
		{
			return LSM9DS1_XL_START_SUCCESS;
		}
		else
			return LSM9DS1_XL_START_ERROR_FS;

	}
	else
		return LSM9DS1_XL_START_ERROR_ODR;



}



 LSM9DS1_XL_READ LSM9DS1_Read_XL(SensorAxes_t *acceleration)
{
	  int16_t pData[3];
	  int i,j,k;
	  float sensitivity= ( float )LSM6DS3_ACC_SENSITIVITY_FOR_FS_4G;
	  /*********************************************************
	   *  regValue:          H			 L
	   *  		X axes= regValue[1] + regValue[0]
	   *  		Y axes= regValue[3] + regValue[2]
	   *  		z axes= regValue[5] + regValue[4]
	   *
	   *********************************************************/
	  uint8_t regValue[6] = {0, 0, 0, 0, 0, 0};


	  int numberOfByteForDimension = 6 / 3;

	  /* Read output registers from LSM9DS1_OUT_X_XL to LSM9DS1_OUT_Z_XL. */

	  k = 0;
	   for (i = 0; i < 3; i++ )
	   {
	     for (j = 0; j < numberOfByteForDimension; j++ )
	     {
	       if( !I2C_ReadData(LSM9DS1_I2C_BADD_XLG, LSM9DS1_OUT_X_L_G + k, &regValue[k],8))
	         return LSM9DS1_XL_READ_ERROR ;
	       k++;
	     }
	   }

	   /* Format the data. */
	    pData[0] = ( ( ( ( int16_t )regValue[1] ) << 8 ) + ( int16_t )regValue[0] );
	    pData[1] = ( ( ( ( int16_t )regValue[3] ) << 8 ) + ( int16_t )regValue[2] );
	    pData[2] = ( ( ( ( int16_t )regValue[5] ) << 8 ) + ( int16_t )regValue[4] );

	    /* Calculate the data. */
	     acceleration->axis_x = ( int32_t )( pData[0] * sensitivity );
	     acceleration->axis_y = ( int32_t )( pData[1] * sensitivity );
	     acceleration->axis_z = ( int32_t )( pData[2] * sensitivity );

	    return LSM9DS1_XL_READ_SUCCESS;

}



 LSM9DS1_State_Connection LSM9DS1_IsConnected()
 {

 		uint8_t resultXLG[1];
 		uint8_t resultM[1];
 	if(!I2C_ReadData(LSM9DS1_I2C_BADD_XLG,LSM9DS1_WHO_AM_I,&resultXLG[0],1))
 	{
 		if (!I2C_ReadData(LSM9DS1_I2C_BADD_M,LSM9DS1_WHO_AM_I_M,&resultM[0],1))
 		{
 			if(resultXLG[0] == LSM9DS1_WHO_AM_I_VALUE)
 			{
 				if(resultM[0] == LSM9DS1_WHO_AM_I_M_VALUE)
 				{
 					return LSM9DS1_OK;
 				}
 				else
 				{
 					return LSM9DS1_M_ERROR;
 				}
 			}
 			else
 			{
 				   return LSM9DS1_XLG_ERROR;
 			}
 		}
 		else
 		{
 			return LSM9DS1_ERROR;
 		}


 	}
 	else
 	{
 		return LSM9DS1_ERROR;
 	}


 }


