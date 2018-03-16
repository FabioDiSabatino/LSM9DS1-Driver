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
 * @retval 0 in case of success
 * @retval 1 in case of failure
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
    return 1;
  }
  else
  {
    return 0;
  }
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


