/*
 * LIS3DSH.C
 *
 *  Created on: 12 janv. 2021
 *      Author: matte
 */

#include <LIS3DSH.h>

struct {
	GPIO_TypeDef *GPIO_Port;
	uint16_t GPIO_Pin;
}l_s_gpio_config;


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

//	LIS3DSH_Read_reg(hspi, reg_addr, dataR, 2);
//	dataW[1] |= dataR[1];

	HAL_GPIO_WritePin(l_s_gpio_config.GPIO_Port, l_s_gpio_config.GPIO_Pin, GPIO_PIN_RESET);
	if(HAL_SPI_Transmit(hspi, dataW, size, 10) == HAL_OK)
	{
		HAL_GPIO_WritePin(l_s_gpio_config.GPIO_Port, l_s_gpio_config.GPIO_Pin, GPIO_PIN_SET);
		LIS3DSH_Read_reg(hspi, reg_addr, dataR, 2);

		return LIS3DSH_OK;
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

	/* Configuring CTRL_REG5 */
	l_reg_val[1] = (init_struct->full_scale & 0x38) \
				 | (init_struct->SPI_Mode & 0x01);

	if(LIS3DSH_Write_reg(hspi, LIS3DSH_REG_CTRL_REG5_ADDR, l_reg_val, 2) != LIS3DSH_OK)
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

	/*TODO : Configuring CTRL_REG3 (Interrupts) */

	return LIS3DSH_OK;
}


/**
 * @brief :
 *
 * @param : SPI_HandleTypeDef *hspi - SPI handler pointer used to communicate with the component
 *
 * @retval : t_e_lis3dsh_error - returns the error code if any, or a no error
 */
t_e_lis3dsh_error LIS3DSH_Get_axes(SPI_HandleTypeDef *hspi,
								   	uint16_t *axes)
{
	uint8_t dataR[3] = {0x00, 0x00, 0x00};

	if(LIS3DSH_Read_reg(hspi, LIS3DSH_REG_OUT_X_ADDR, dataR, 3) != LIS3DSH_OK)
	{
		return LIS3DSH_GET_AXE_X_ERROR;
	}
	axes[0] = dataR[1] | dataR[2] << 8;

	if(LIS3DSH_Read_reg(hspi, LIS3DSH_REG_OUT_Y_ADDR, dataR, 3) != LIS3DSH_OK)
	{
		return LIS3DSH_GET_AXE_Y_ERROR;
	}
	axes[1] = dataR[1] | dataR[2] << 8;

	if(LIS3DSH_Read_reg(hspi, LIS3DSH_REG_OUT_Z_ADDR, dataR, 3) != LIS3DSH_OK)
	{
		return LIS3DSH_GET_AXE_Z_ERROR;
	}
	axes[2] = dataR[1] | dataR[2] << 8;
}
