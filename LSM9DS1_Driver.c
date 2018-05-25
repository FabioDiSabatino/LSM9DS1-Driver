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
 * Name: LSM9DS1_modifyReg8
 *
 * Description:
 *   Modify an 8-bit register.
 *
 ****************************************************************************/

static LSM9DS1_MEM_WRITE LSM9DS1_modifyReg8(uint8_t B_Addr,uint8_t Reg, uint8_t pBuffer, uint8_t mask)
{
	uint8_t regValue;

	if(I2C_ReadData(B_Addr,Reg,&regValue,1))
	{
		regValue &= ~mask;
		//safety and to clear bits in pBuffer not in the mask
		pBuffer &= mask;
		regValue |= pBuffer;
		if(I2C_WriteData(B_Addr,Reg,&regValue,1))
		{
			return LSM9DS1_MEM_WRITE_SUCCESS;
		}

	}
	return LSM9DS1_MEM_WRITE_ERROR;

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

void LSM9DS1_XLG_TurnOff()
{
	uint8_t offBits=0;

	I2C_WriteData(LSM9DS1_I2C_BADD,LSM9DS1_CTRL_REG6_XL,&offBits,1);
	I2C_WriteData(LSM9DS1_I2C_BADD,LSM9DS1_CTRL_REG1_G,&offBits,1);


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
	uint8_t setbits;

		if (odr < lsm9ds1_midpoint(10, 50))
		    {
		      setbits = LSM9DS1_CTRL_REG6_XL_ODR_XL_10HZ;
		    }
		  else if (odr < lsm9ds1_midpoint(50, 119))
		    {
			  setbits = LSM9DS1_CTRL_REG6_XL_ODR_XL_50HZ;
		    }
		  else if (odr < lsm9ds1_midpoint(119, 238))
		    {
			  setbits = LSM9DS1_CTRL_REG6_XL_ODR_XL_119HZ;
		    }
		  else if (odr < lsm9ds1_midpoint(238, 476))
		    {
			  setbits = LSM9DS1_CTRL_REG6_XL_ODR_XL_238HZ;
		    }
		  else if (odr < lsm9ds1_midpoint(476, 952))
		    {
			  setbits = LSM9DS1_CTRL_REG6_XL_ODR_XL_476HZ;
		    }
		  else
		    {
			  setbits = LSM9DS1_CTRL_REG6_XL_ODR_XL_952HZ;
		    }

		 //Set power mode and ODR
		 return LSM9DS1_modifyReg8(LSM9DS1_I2C_BADD,LSM9DS1_CTRL_REG6_XL,setbits,LSM9DS1_CTRL_REG6_XL_ODR_XL_MASK);



}

static int LSM9DS1_G_SetOdr(int odr){
	  uint8_t setbits;

		  if ( odr < lsm9ds1_midpoint(14, 59))
		     {
		       setbits = LSM9DS1_CTRL_REG1_G_ODR_G_14p9HZ;
		     }
		   else if (odr < lsm9ds1_midpoint(59, 119))
		     {
		       setbits = LSM9DS1_CTRL_REG1_G_ODR_G_59p5HZ;
		     }
		   else if (odr < lsm9ds1_midpoint(119, 238))
		     {
		       setbits = LSM9DS1_CTRL_REG1_G_ODR_G_119HZ;
		     }
		   else if (odr < lsm9ds1_midpoint(238, 476))
		     {
		       setbits = LSM9DS1_CTRL_REG1_G_ODR_G_238HZ;
		     }
		   else if (odr < lsm9ds1_midpoint(476, 952))
		     {
		       setbits = LSM9DS1_CTRL_REG1_G_ODR_G_476HZ;
		     }
		   else
		     {
		       setbits = LSM9DS1_CTRL_REG1_G_ODR_G_952HZ;
		     }


		  //Set full-scale
		return LSM9DS1_modifyReg8(LSM9DS1_I2C_BADD,LSM9DS1_CTRL_REG1_G,setbits,LSM9DS1_CTRL_REG1_G_ODR_G_MASK );

}

static int LSM9DS1_M_SetOdr(int odr){

	  uint8_t setbits;


	if (odr < lsm9ds1_midpoint(0, 1))
	    {
	      setbits = LSM9DS1_CTRL_REG1_M_DO_0p625HZ;
	    }
	  else if (odr < lsm9ds1_midpoint(1, 2))
	    {
	      setbits = LSM9DS1_CTRL_REG1_M_DO_1p25HZ;
	    }
	  else if (odr < lsm9ds1_midpoint(2, 5))
	    {
	      setbits = LSM9DS1_CTRL_REG1_M_DO_2p5HZ;
	    }
	  else if (odr < lsm9ds1_midpoint(5, 10))
	    {
	      setbits = LSM9DS1_CTRL_REG1_M_DO_5HZ;
	    }
	  else if (odr < lsm9ds1_midpoint(10, 20))
	    {
	      setbits = LSM9DS1_CTRL_REG1_M_DO_10HZ;
	    }
	  else if (odr < lsm9ds1_midpoint(20, 40))
	    {
	      setbits = LSM9DS1_CTRL_REG1_M_DO_20HZ;
	    }
	  else if (odr < lsm9ds1_midpoint(40, 80))
	    {
	      setbits = LSM9DS1_CTRL_REG1_M_DO_40HZ;
	    }
	  else
	    {
	      setbits = LSM9DS1_CTRL_REG1_M_DO_80HZ;
	    }

	return LSM9DS1_modifyReg8(LSM9DS1_I2C_BADD_M,LSM9DS1_CTRL_REG1_M,setbits,LSM9DS1_CTRL_REG1_M_DO_MASK );


}


static int LSM9DS1_M_Start(){

	return LSM9DS1_modifyReg8(LSM9DS1_I2C_BADD_M,LSM9DS1_CTRL_REG3_M,LSM9DS1_CTRL_REG3_M_MD_CONT,LSM9DS1_CTRL_REG3_M_MD_MASK );


}


uint8_t LSM9DS1_get_Odr_XL(){

	uint8_t odr;


	if(I2C_ReadData(LSM9DS1_I2C_BADD,LSM9DS1_CTRL_REG6_XL,&odr,1))
	{
	    odr= odr & LSM9DS1_CTRL_REG6_XL_ODR_XL_MASK;
	    odr= odr >> LSM9DS1_CTRL_REG6_XL_ODR_XL_SHIFT;

	    if (odr == 1)
	     {
	    	return LSM9DS1_CTRL_REG6_XL_ODR_XL_10HZ;
	     }
	    else if (odr == 2)
	    {
	    	return LSM9DS1_CTRL_REG6_XL_ODR_XL_50HZ;
	    }
	    else if (odr == 3)
	    {
	    return LSM9DS1_CTRL_REG6_XL_ODR_XL_119HZ;
	    }
	    else if (odr == 4)
	    {
	    	return LSM9DS1_CTRL_REG6_XL_ODR_XL_238HZ;
	    }
	    else if (odr == 5)
	    {
	    	return LSM9DS1_CTRL_REG6_XL_ODR_XL_476HZ;
	    }
	    else if (odr == 6)
	    {
	      return LSM9DS1_CTRL_REG6_XL_ODR_XL_952HZ;
	    }
	    else
	    {
	    	return LSM9DS1_CTRL_REG6_XL_ODR_XL_POWERDOWN;
	    }

	}
	else
	{
		return 0;
	}
}

uint8_t LSM9DS1_get_Odr_G(){
	uint8_t odr;


	if(I2C_ReadData(LSM9DS1_I2C_BADD,LSM9DS1_CTRL_REG1_G,&odr,1))
	{
	    odr= odr & LSM9DS1_CTRL_REG1_G_ODR_G_MASK;
	    odr= odr >> LSM9DS1_CTRL_REG1_G_ODR_G_SHIFT;

	    if (odr == 1)
	     {
	    	return LSM9DS1_CTRL_REG1_G_ODR_G_14p9HZ;
	     }
	    else if (odr == 2)
	    {
	    	return LSM9DS1_CTRL_REG1_G_ODR_G_59p5HZ;
	    }
	    else if (odr == 3)
	    {
	    return LSM9DS1_CTRL_REG1_G_ODR_G_119HZ;
	    }
	    else if (odr == 4)
	    {
	    	return LSM9DS1_CTRL_REG1_G_ODR_G_238HZ;
	    }
	    else if (odr == 5)
	    {
	    	return LSM9DS1_CTRL_REG1_G_ODR_G_476HZ;
	    }
	    else if (odr == 6)
	    {
	      return LSM9DS1_CTRL_REG1_G_ODR_G_952HZ;
	    }
	    else
	    {
	    	return LSM9DS1_CTRL_REG1_G_ODR_G_POWERDOWN;
	    }

	}
	else
	{
		return 0;
	}
}

uint8_t LSM9DS1_get_Fs_XL(){

	uint8_t fs;

	if(I2C_ReadData(LSM9DS1_I2C_BADD,LSM9DS1_CTRL_REG6_XL,&fs,1))
		{
		    fs= fs & LSM9DS1_CTRL_REG6_XL_FS_XL_MASK;
		    fs= fs >> LSM9DS1_CTRL_REG6_XL_FS_XL_SHIFT;


		    if (fs == 1)
		    {
		    	return LSM9DS1_CTRL_REG6_XL_FS_XL_16G;
		    }
		    else if (fs == 2)
		    {
		    return LSM9DS1_CTRL_REG6_XL_FS_XL_4G;
		    }
		    else if (fs == 3)
		    {
		    	return LSM9DS1_CTRL_REG6_XL_FS_XL_8G;
		    }
		    else
		    {
		    	return LSM9DS1_CTRL_REG6_XL_FS_XL_2G;
		    }

		}
		else
		{
			return 0;
		}
}

double LSM9DS1_get_Sens_XL(){

	uint8_t fs= LSM9DS1_get_Fs_XL();

		    if (fs == LSM9DS1_CTRL_REG6_XL_FS_XL_16G)
		    {
		    	return  LSM9DS1_ACC_SENSITIVITY_FOR_FS_16G;
		    }
		    else if (fs == LSM9DS1_CTRL_REG6_XL_FS_XL_4G)
		    {
		    return  LSM9DS1_ACC_SENSITIVITY_FOR_FS_4G;
		    }
		    else if (fs == LSM9DS1_CTRL_REG6_XL_FS_XL_8G)
		    {
		    	return LSM9DS1_ACC_SENSITIVITY_FOR_FS_8G;
		    }
		    else
		    {
		    	return LSM9DS1_ACC_SENSITIVITY_FOR_FS_2G;
		    }

}

uint8_t LSM9DS1_get_Fs_G(){

	uint8_t fs;

	if(I2C_ReadData(LSM9DS1_I2C_BADD,LSM9DS1_CTRL_REG1_G,&fs,1))
		{
		    fs= fs & LSM9DS1_CTRL_REG1_G_FS_G_MASK;
		    fs= fs >> LSM9DS1_CTRL_REG1_G_FS_G_SHIFT;


		    if (fs == 1)
		    {
		    	return LSM9DS1_CTRL_REG1_G_FS_G_500DPS;
		    }
		    else if (fs == 3)
		    {
		    return LSM9DS1_CTRL_REG1_G_FS_G_2000DPS;
		    }
		    else
		    {
		    	return LSM9DS1_CTRL_REG1_G_FS_G_245DPS;
		    }

		}
		else
		{
			return 0;
		}
}

double LSM9DS1_get_Sens_G(){



	uint8_t fs= LSM9DS1_get_Fs_G();

		    if (fs == LSM9DS1_CTRL_REG1_G_FS_G_500DPS)
		    {
		    	return LSM9DS1_GYRO_SENSITIVITY_FOR_FS_500DPS;
		    }
		    else if (fs == LSM9DS1_CTRL_REG1_G_FS_G_2000DPS)
		    {
		    	return LSM9DS1_GYRO_SENSITIVITY_FOR_FS_2000DPS;
		    }
		    else
		    {
		    	return LSM9DS1_GYRO_SENSITIVITY_FOR_FS_245DPS;
		    }

}

uint8_t LSM9DS1_get_Fs_M(){
	uint8_t fs;

	if(I2C_ReadData(LSM9DS1_I2C_BADD_M,LSM9DS1_CTRL_REG2_M,&fs,1))
			{
			    fs= fs & LSM9DS1_CTRL_REG2_M_FS_MASK;
			    fs= fs >> LSM9DS1_CTRL_REG2_M_FS_SHIFT;


			    if (fs == 0)
			    {
			    	return LSM9DS1_CTRL_REG2_M_FS_4GAUSS;
			    }
			    else if (fs == 1)
			    {
			    	return LSM9DS1_CTRL_REG2_M_FS_8GAUSS;
			    }
			    else if (fs==2)
			    {
			    	return LSM9DS1_CTRL_REG2_M_FS_12GAUSS;
			    }
			    else
			    {
			    	return LSM9DS1_CTRL_REG2_M_FS_16GAUSS;
			    }

			}
			else
			{
				return 0;
			}

}


double LSM9DS1_get_Sens_M(){

	uint8_t fs= LSM9DS1_get_Fs_M();

	if (fs==LSM9DS1_CTRL_REG2_M_FS_4GAUSS)
	{
		return LSM9DS1_M_SENSITIVITY_FOR_FS_4GAUSS;
	}
	else if( fs == LSM9DS1_CTRL_REG2_M_FS_8GAUSS)
	{
		return LSM9DS1_M_SENSITIVITY_FOR_FS_8GAUSS;
	}
	else if ( fs == LSM9DS1_CTRL_REG2_M_FS_12GAUSS)
	{
		return LSM9DS1_M_SENSITIVITY_FOR_FS_12GAUSS;
	}
	else
	{
		return LSM9DS1_M_SENSITIVITY_FOR_FS_16GAUSS;
	}



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
	  uint8_t setbits;

	  if (fullscale < lsm9ds1_midpoint(2, 4))
	     {
		  setbits = LSM9DS1_CTRL_REG6_XL_FS_XL_2G;
	     }
	   else if (fullscale < lsm9ds1_midpoint(4, 8))
	     {
		   setbits = LSM9DS1_CTRL_REG6_XL_FS_XL_4G;
	     }
	   else if (fullscale < lsm9ds1_midpoint(8, 16))
	     {
		   setbits = LSM9DS1_CTRL_REG6_XL_FS_XL_8G;
	     }
	   else
	     {
		   setbits = LSM9DS1_CTRL_REG6_XL_FS_XL_16G;
	     }
	  //Set full-scale
	return LSM9DS1_modifyReg8(LSM9DS1_I2C_BADD,LSM9DS1_CTRL_REG6_XL,setbits,LSM9DS1_CTRL_REG6_XL_FS_XL_MASK );




}

static int LSM9DS1_G_SetFs(int fullscale)
{
	  uint8_t setbits;

	  if (fullscale < lsm9ds1_midpoint(245, 500))
	     {
	       setbits = LSM9DS1_CTRL_REG1_G_FS_G_245DPS;
	     }
	   else if (fullscale < lsm9ds1_midpoint(500, 2000))
	     {
	       setbits = LSM9DS1_CTRL_REG1_G_FS_G_500DPS;
	     }
	   else
	     {
	       setbits = LSM9DS1_CTRL_REG1_G_FS_G_2000DPS;
	     }


	  //Set full-scale
	return LSM9DS1_modifyReg8(LSM9DS1_I2C_BADD,LSM9DS1_CTRL_REG1_G,setbits,LSM9DS1_CTRL_REG1_G_FS_G_MASK );
}

static int LSM9DS1_M_SetFs(int fullscale){
	 uint8_t setbits;

	 if (fullscale < lsm9ds1_midpoint(4, 8))
	     {
	       setbits = LSM9DS1_CTRL_REG2_M_FS_4GAUSS;
	     }
	   else if (fullscale < lsm9ds1_midpoint(8, 12))
	     {
	       setbits = LSM9DS1_CTRL_REG2_M_FS_8GAUSS;
	     }
	   else if (fullscale < lsm9ds1_midpoint(12, 16))
	     {
	       setbits = LSM9DS1_CTRL_REG2_M_FS_12GAUSS;
	     }
	   else
	     {
	       setbits = LSM9DS1_CTRL_REG2_M_FS_16GAUSS;
	     }

	 return LSM9DS1_modifyReg8(LSM9DS1_I2C_BADD_M,LSM9DS1_CTRL_REG2_M,setbits,LSM9DS1_CTRL_REG2_M_FS_MASK );

}



static int LSM9SD1_SetFIFO()
{
	return LSM9DS1_modifyReg8(LSM9DS1_I2C_BADD,LSM9DS1_FIFO_CTRL,LSM9DS1_FIFO_CTRL_FMODE_CONT,LSM9DS1_FIFO_CTRL_FMODE_MASK );
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

LSM9DS1_XLG_START LSM9DS1_XLG_Start(int odr, int fsG, int fsXl)
 {

	 if(LSM9DS1_G_SetOdr(odr))
	 {
		 if(LSM9DS1_XL_SetOdr(odr))
		{
			 if(LSM9DS1_XL_SetFs(fsXl))
			 {
			 	if(LSM9DS1_G_SetFs(fsG))
			 	  {
			 		 return LSM9DS1_XLG_START_SUCCESS;
			 	   }
			 	else
			 	    return LSM9DS1_XLG_START_ERROR_FS;
			 }
			 else
			    return LSM9DS1_XL_START_ERROR_FS;
		 }
		 else
			 return LSM9DS1_XLG_START_ERROR;

	 }
	 else
		 return LSM9DS1_XLG_START_ERROR;
 }

LSM9DS1_XLGM_START LSM9DS1_XLGM_Start(int odr,int odrM, int fsG, int fsXl, int fsM)
 {
	 if(!LSM9DS1_M_Start())
			 return LSM9DS1_XLGM_START_ERROR;

	if(!LSM9DS1_G_SetOdr(odr))
		return LSM9DS1_XLG_START_ERROR;

	if(!LSM9DS1_XL_SetOdr(odr))
		return LSM9DS1_XLG_START_ERROR;

	if (!LSM9DS1_M_SetOdr(odrM))
		return LSM9DS1_M_START_ERROR_ODR;

	 if(!LSM9DS1_XL_SetFs(fsXl))
		return LSM9DS1_XLG_START_ERROR;

	 if(!LSM9DS1_G_SetFs(fsG))
		return LSM9DS1_XLG_START_ERROR;

	 if(!LSM9DS1_M_SetFs(fsM))
		 return LSM9DS1_XLGM_START_ERROR;


	 return LSM9DS1_XLGM_START_ERROR;

 }




LSM9DS1_XL_READ LSM9DS1_Read_XL(struct SensorXLAxes_t *acceleration,int samples)
{
	  int16_t pData[3]={0,0,0};
	  int16_t data=0;
	  int8_t regadd;
	  int i,j,k;
	  float sensitivity= ( float )LSM9DS1_ACC_SENSITIVITY_FOR_FS_2G;

	  uint8_t hi;
	  uint8_t lo;



	  /* Read output registers from LSM9DS1_OUT_X_XL to LSM9DS1_OUT_Z_XL. */
	   for (i = 0; i < samples; i++ )
	   {
		   regadd=LSM9DS1_OUT_X_L_XL;

	     for (j = 0; j < 3; j++ )
	     {
	       if( !I2C_ReadData(LSM9DS1_I2C_BADD, regadd, &lo,1))
	       {
		         return LSM9DS1_XL_READ_ERROR ;

	       }
	       regadd++;
	       if( !I2C_ReadData(LSM9DS1_I2C_BADD,regadd, &hi,1))
	       {
	    	   return LSM9DS1_XL_READ_ERROR ;
	       }
	       regadd++;

	       data = ((uint16_t)hi << 8) | (uint16_t)lo;
	       /* The value is positive */
	       if (data < 0x8000)
	          {
	           pData[j] = (int16_t)data;
	          }

	       /* The value is negative, so find its absolute value by taking the
	        * two's complement
	        */
	       else if (data > 0x8000)
	       {
	         data = ~data + 1;
	         pData[j] = -(int16_t)data;
	       }

	       /* The value is negative and can't be represented as a positive
	        * int16_t value
	        */
	       else
	        {
	           pData[j] = (int16_t)(-32768);
	        }
	     }
	   }

	    /* Format the data. */
	     acceleration->axis_x = ( int32_t ) (pData[0] * sensitivity );
	     acceleration->axis_y = ( int32_t ) (pData[1] * sensitivity );
	     acceleration->axis_z = ( int32_t )(pData[2] * sensitivity );

	    return LSM9DS1_XL_READ_SUCCESS;

}

LSM9DS1_XLG_READ LSM9DS1_Read_XLG(MFX_input_t *data_in, int samples)
{
	  int16_t pData[6]={0,0,0,0,0,0};
	  int16_t data=0;
	  int8_t regadd;
	  int i,j,k;
	  float divisor=1000;
	  float sensitivityXL= (float) LSM9DS1_get_Sens_XL();
	  float sensitivityG= (float) LSM9DS1_get_Sens_G();

	  uint8_t hi;
	  uint8_t lo;



	  /* Read output registers from LSM9DS1_OUT_X_L_G to LSM9DS1_OUT_Z_XL. */
	   for (i = 0; i < samples; i++ )
	   {
		   regadd=LSM9DS1_OUT_X_L_G;

		  //read Gyroscope output
	     for (j = 0; j < 3; j++ )
	     {
	       if( !I2C_ReadData(LSM9DS1_I2C_BADD, regadd, &lo,1))
	       {
		         return 0;

	       }
	       regadd++;
	       if( !I2C_ReadData(LSM9DS1_I2C_BADD,regadd, &hi,1))
	       {
	    	   return 0 ;
	       }
	       regadd++;

	       data = ((uint16_t)hi << 8) | (uint16_t)lo;
	       pData[j]=data;

	     }
	     /* Format the data. */
	     data_in->gyro[0] = ( int32_t ) (pData[0] * sensitivityG )/divisor;
	     data_in->gyro[1] = ( int32_t ) (pData[1] * sensitivityG )/divisor;
	     data_in->gyro[2] = ( int32_t )(pData[2] * sensitivityG )/divisor;

	     regadd=LSM9DS1_OUT_X_L_XL;
	     //read Accelerometer output
	   	 for (j = 3; j < 6; j++ )
	   	 {
	   	  if( !I2C_ReadData(LSM9DS1_I2C_BADD, regadd, &lo,1))
	   	  {
	   		   return 0;
	   	  }
	   	  regadd++;
	   	  if( !I2C_ReadData(LSM9DS1_I2C_BADD,regadd, &hi,1))
	   	  {
	   	       return 0 ;
	   	  }
	   	  regadd++;

	   	  data = ((uint16_t)hi << 8) | (uint16_t)lo;
	       pData[j]=data;


	   	 }
	   	     /* Format the data. */
	   	 	 data_in->acc[0] = ( int32_t ) (pData[3] * sensitivityXL)/divisor;
	   	 	 data_in->acc[1] = ( int32_t ) (pData[4] * sensitivityXL)/divisor;
	   	 	 data_in->acc[2] = ( int32_t )(pData[5] * sensitivityXL)/divisor;
	   }
	    return 1;
}


LSM9DS1_XLG_READ LSM9DS1_Read_XLGM(MFX_input_t *data_in, int samples)
{
	  int16_t pData[9]={0,0,0,0,0,0,0,0,0};
	  int16_t data=0;
	  int8_t regadd;
	  int i,j,k;
	  float divisor=1000;
	  float sensitivityXL= (float) LSM9DS1_get_Sens_XL();
	  float sensitivityG= (float) LSM9DS1_get_Sens_G();
	  float sensitivityM= (float) LSM9DS1_get_Sens_M();

	  uint8_t hi;
	  uint8_t lo;



	  /* Read output registers from LSM9DS1_OUT_X_L_G to LSM9DS1_OUT_Z_XL. */
	   for (i = 0; i < samples; i++ )
	   {
		   regadd=LSM9DS1_OUT_X_L_G;

		  //read Gyroscope output
	     for (j = 0; j < 3; j++ )
	     {
	       if( !I2C_ReadData(LSM9DS1_I2C_BADD, regadd, &lo,1))
	       {
		         return 0;

	       }
	       regadd++;
	       if( !I2C_ReadData(LSM9DS1_I2C_BADD,regadd, &hi,1))
	       {
	    	   return 0 ;
	       }
	       regadd++;

	       data = ((uint16_t)hi << 8) | (uint16_t)lo;
	       pData[j]=data;

	     }
	     /* Format the data. */
	     data_in->gyro[0] = ( int32_t ) (pData[0] * sensitivityG )/divisor;
	     data_in->gyro[1] = ( int32_t ) (pData[1] * sensitivityG )/divisor;
	     data_in->gyro[2] = ( int32_t )(pData[2] * sensitivityG )/divisor;

	     regadd=LSM9DS1_OUT_X_L_XL;
	     //read Accelerometer output
	   	 for (j = 3; j < 6; j++ )
	   	 {
	   	  if( !I2C_ReadData(LSM9DS1_I2C_BADD, regadd, &lo,1))
	   	  {
	   		   return 0;
	   	  }
	   	  regadd++;
	   	  if( !I2C_ReadData(LSM9DS1_I2C_BADD,regadd, &hi,1))
	   	  {
	   	       return 0 ;
	   	  }
	   	  regadd++;

	   	  data = ((uint16_t)hi << 8) | (uint16_t)lo;
	       pData[j]=data;


	   	 }
	   	     /* Format the data. */
	   	 	 data_in->acc[0] = ( int32_t ) (pData[3] * sensitivityXL)/divisor;
	   	 	 data_in->acc[1] = ( int32_t ) (pData[4] * sensitivityXL)/divisor;
	   	 	 data_in->acc[2] = ( int32_t )(pData[5] * sensitivityXL)/divisor;


	   	 regadd=LSM9DS1_OUT_X_L_M;

	   	 //read Magnetometer output
	   	 for(j=6; j < 9; j++){

	   		 if( !I2C_ReadData(LSM9DS1_I2C_BADD_M, regadd, &lo,1))
	   		 {
	   			   return 0;
	   		 }

	   		 regadd++;
	   		if( !I2C_ReadData(LSM9DS1_I2C_BADD_M, regadd, &hi,1))
	   		{
	   			  return 0;
	   		}
	   		regadd++;

	   	    data = ((uint16_t)hi << 8) | (uint16_t)lo;
	   		pData[j]=data;

	   	 }
	   		/* Format the data in uT  */
	   		data_in->mag[0] = ( int32_t ) ((pData[6] * sensitivityM)/10);
	   		data_in->mag[1] = ( int32_t ) ((pData[7] * sensitivityM)/10);
	   		data_in->mag[2] = ( int32_t ) ((pData[8] * sensitivityM)/10);




	   }
	    return 1;
}







LSM9DS1_State_Connection LSM9DS1_IsConnected()
 {

 		uint8_t resultXLG[1];
 		uint8_t resultM[1];
 	if(I2C_ReadData(LSM9DS1_I2C_BADD,LSM9DS1_WHO_AM_I,&resultXLG[0],1))
 	{
 		if (I2C_ReadData(LSM9DS1_I2C_BADD,LSM9DS1_WHO_AM_I_M,&resultM[0],1))
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

