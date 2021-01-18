/*
 * LIS3DSH.C
 *
 *  Created on: 12 janv. 2021
 *      Author: matte
 */

#include <LIS3DSH.h>


/* LOCAL VARIABLES */
struct {
	GPIO_TypeDef *GPIO_Port;
	uint16_t GPIO_Pin;
}l_s_gpio_config;

uint8_t l_v_calibre;


/**
 * @brief : Use this function to read a register of lis3dsh
 *
 * @param : SPI_HandleTypeDef *hspi - SPI handler pointer used to communicate with the component
 * @param : uint8_t reg_addr - address of the targeted register in the LIS3DSH
 * @param : uint8_t *dataR - pointer to store the read value
 * @param : uint8_t size - size (in byte) of the value to read
 *
 * @retval : t_e_lis3dsh_error - returns the error code if any, or a no error
 *
 */
t_e_lis3dsh_error LIS3DSH_Read_reg(SPI_HandleTypeDef *hspi,
								   uint8_t reg_addr,
								   uint8_t *dataR,
								   uint8_t size)
{
	dataR[0] = 0x80 | reg_addr;

	HAL_GPIO_WritePin(l_s_gpio_config.GPIO_Port, l_s_gpio_config.GPIO_Pin, GPIO_PIN_RESET);
	if(HAL_SPI_Receive(hspi, dataR, size, 10) == HAL_OK)
	{
		HAL_GPIO_WritePin(l_s_gpio_config.GPIO_Port, l_s_gpio_config.GPIO_Pin, GPIO_PIN_SET);
		return LIS3DSH_OK;
	}
	HAL_GPIO_WritePin(l_s_gpio_config.GPIO_Port, l_s_gpio_config.GPIO_Pin, GPIO_PIN_SET);

	return LIS3DSH_READ_ERROR;
}


/**
 * @brief : Use this function to read a register of lis3dsh
 *
 * @param : SPI_HandleTypeDef *hspi - SPI handler pointer used to communicate with the component
 * @param : uint8_t reg_addr - address of the targeted register in the LIS3DSH
 * @param : uint8_t *dataW - pointer to the value to write
 * @param : uint8_t size - size (in byte) of the value to write
 *
 * @retval : t_e_lis3dsh_error - returns the error code if any, or a no error
 *
 */
t_e_lis3dsh_error LIS3DSH_Write_reg(SPI_HandleTypeDef *hspi,
									uint8_t reg_addr,
									uint8_t *dataW,
									uint8_t size)
{
	dataW[0] = 0xEF & reg_addr;
	uint8_t dataR[] = {0x00, 0x00};

	HAL_GPIO_WritePin(l_s_gpio_config.GPIO_Port, l_s_gpio_config.GPIO_Pin, GPIO_PIN_RESET);
	if(HAL_SPI_Transmit(hspi, dataW, size, 10) == HAL_OK)
	{
		HAL_GPIO_WritePin(l_s_gpio_config.GPIO_Port, l_s_gpio_config.GPIO_Pin, GPIO_PIN_SET);
		if(LIS3DSH_Read_reg(hspi, reg_addr, dataR, 2) == LIS3DSH_OK)
		{
			if(dataR[1] == dataW[1])
			{
				return LIS3DSH_OK;
			}
		}
	}

	HAL_GPIO_WritePin(l_s_gpio_config.GPIO_Port, l_s_gpio_config.GPIO_Pin, GPIO_PIN_SET);

	return LIS3DSH_WRITE_ERROR;
}

/**
 * @brief : Use this function to init the lis3dsh
 *
 * @param : SPI_HandleTypeDef *hspi - SPI handler pointer used to communicate with the component
 * @param : t_s_lis3dsh_init *init_struct - pointer to the init structure
 *
 * @retval : t_e_lis3dsh_error - returns the error code if any, or a no error
 *
 */
t_e_lis3dsh_error LIS3DSH_Init(SPI_HandleTypeDef *hspi,
							   GPIO_TypeDef *GPIO_Port,
							   uint16_t GPIO_Pin,
							   t_s_lis3dsh_init *init_struct)
{
	uint8_t l_reg_val[] = {0x00, 0x00};

	/* Configuring GPIO (CS) */
	l_s_gpio_config.GPIO_Port = GPIO_Port;
	l_s_gpio_config.GPIO_Pin = GPIO_Pin;

	HAL_GPIO_WritePin(l_s_gpio_config.GPIO_Port, l_s_gpio_config.GPIO_Pin, GPIO_PIN_SET);

	/* Sending dummy word on the SPI to avoid bugs */
	LIS3DSH_Read_reg(hspi, 0x00, l_reg_val, 2);

	/* Configuring CTRL_REG5 */
	l_reg_val[1] = (init_struct->full_scale & 0x38) \
				 | (init_struct->SPI_Mode & 0x01);

	if(LIS3DSH_Write_reg(hspi, LIS3DSH_REG_CTRL_REG5_ADDR, l_reg_val, 2) != LIS3DSH_OK)
	{
		return LIS3DSH_INIT_ERROR;
	}

	/* Configuring CTRL_REG3 */
	l_reg_val[1] = (init_struct->int_struct->dataReadyEnable & 0x80) \
				 | (init_struct->int_struct->polarity & 0x40) \
				 | (init_struct->int_struct->latching & 0x20) \
				 | (init_struct->int_struct->int2_enable & 0x10) \
				 | (init_struct->int_struct->int1_enable & 0x08);

	if(LIS3DSH_Write_reg(hspi, LIS3DSH_REG_CTRL_REG3_ADDR, l_reg_val, 2) != LIS3DSH_OK)
	{
		return LIS3DSH_INIT_ERROR;
	}

	/* Configuring CTRL_REG4 */
	l_reg_val[1] = (init_struct->dataRate & 0xF0) 	\
				 | (init_struct->dataUpdate & 0x08) \
				 | (init_struct->z_enable & 0x04) 	\
				 | (init_struct->y_enable & 0x02) 	\
				 | (init_struct->x_enable & 0x01);

	if(LIS3DSH_Write_reg(hspi, LIS3DSH_REG_CTRL_REG4_ADDR, l_reg_val, 2) != LIS3DSH_OK)
	{
		return LIS3DSH_INIT_ERROR;
	}

	switch(init_struct->full_scale)
	{
	case LIS3DSH_CTRL_REG5_FSCALE_2:
		l_v_calibre = 2;
		break;
	case LIS3DSH_CTRL_REG5_FSCALE_4:
		l_v_calibre = 4;
		break;
	case LIS3DSH_CTRL_REG5_FSCALE_6:
		l_v_calibre = 6;
		break;
	case LIS3DSH_CTRL_REG5_FSCALE_8:
		l_v_calibre = 8;
		break;
	case LIS3DSH_CTRL_REG5_FSCALE_16:
		l_v_calibre = 16;
		break;
	default:
		return LIS3DSH_GET_ACCELERATION_ERROR;
	}

	return LIS3DSH_OK;
}


/**
 * @brief : This functions get the bare values on all of the 3 axis [X, Y, Z]
 *
 * @param : SPI_HandleTypeDef *hspi - SPI handler pointer used to communicate with the component
 * @param : int16_t *axis - pointer to an array, containing the bare values [x,y,z] of the axis
 * 		@note : those values are bare from the 16 bits axis_HIGH/axis_LOW registers.
 * 		@note : those values are signed, so 0 m.s-2 is around 32768.
 *
 * @retval : t_e_lis3dsh_error - returns the error code if any, or a no error
 */
t_e_lis3dsh_error LIS3DSH_Get_axis(SPI_HandleTypeDef *hspi,
								   	int16_t *axis)
{
	uint8_t dataR[3] = {0x00, 0x00, 0x00};

	if(LIS3DSH_Read_reg(hspi, LIS3DSH_REG_OUT_X_ADDR, dataR, 3) != LIS3DSH_OK)
	{
		return LIS3DSH_GET_AXIS_X_ERROR;
	}
	axis[0] = dataR[1] | dataR[2] << 8;

	if(LIS3DSH_Read_reg(hspi, LIS3DSH_REG_OUT_Y_ADDR, dataR, 3) != LIS3DSH_OK)
	{
		return LIS3DSH_GET_AXIS_Y_ERROR;
	}
	axis[1] = dataR[1] | dataR[2] << 8;

	if(LIS3DSH_Read_reg(hspi, LIS3DSH_REG_OUT_Z_ADDR, dataR, 3) != LIS3DSH_OK)
	{
		return LIS3DSH_GET_AXIS_Z_ERROR;
	}
	axis[2] = dataR[1] | dataR[2] << 8;

	return LIS3DSH_OK;
}


/**
 * @brief : This functions get the acceleration values on all of the 3 axis [X, Y, Z]
 *
 * @param : SPI_HandleTypeDef *hspi - SPI handler pointer used to communicate with the component
 * @param : float *accelerations - pointer to an array, containing the accelerations [x,y,z] values, in m.s-2
 *
 * @retval : t_e_lis3dsh_error - returns the error code if any, or a no error
 */
t_e_lis3dsh_error LIS3DSH_Get_accelerations(SPI_HandleTypeDef *hspi,
		   	   	   	   	   	   	   	   	    float *accelerations)
{
	int16_t 			l_a_axis[3] = {0x00, 0x00, 0x00};
	t_e_lis3dsh_error 	l_e_error;

	l_e_error = LIS3DSH_Get_axis(hspi, l_a_axis);

	if(l_e_error != LIS3DSH_OK)
	{
		return	l_e_error;
	}

	for(uint8_t i = 0; i < 3; i++)
	{
		accelerations[i] = LIS3DSH_EARTH_GRAVITY * ((float)l_a_axis[i] / (32768 / l_v_calibre));
	}

	return l_e_error;
}
