/*
 * LIS3DSH.h
 *
 *  Created on: Jan 11, 2021
 *      Author: matte
 */

#ifndef INC_LIS3DSH_H_
#define INC_LIS3DSH_H_

#include <stm32f4xx_hal.h>

/* Registers address */
#define LIS3DSH_REG_CTRL_REG4_ADDR	((uint8_t) 0x20)
#define LIS3DSH_REG_CTRL_REG3_ADDR	((uint8_t) 0x23)
#define LIS3DSH_REG_CTRL_REG5_ADDR	((uint8_t) 0x24)
#define LIS3DSH_REG_STATUS_ADDR		((uint8_t) 0x27)
#define LIS3DSH_REG_OUT_X_ADDR		((uint8_t) 0x28) /*Base address of OUT_X (H and L)*/
#define LIS3DSH_REG_OUT_Y_ADDR		((uint8_t) 0x2A) /*Base address of OUT_Y (H and L)*/
#define LIS3DSH_REG_OUT_Z_ADDR		((uint8_t) 0x2C) /*Base address of OUT_Z (H and L)*/

/* CTRL_REG4 register */
#define LIS3DSH_CTRL_REG4_ODR_OFF	((uint8_t) 0x00)
#define LIS3DSH_CTRL_REG4_ODR_3_125	((uint8_t) 0x10)
#define LIS3DSH_CTRL_REG4_ODR_6_25	((uint8_t) 0x20)
#define LIS3DSH_CTRL_REG4_ODR_12_5	((uint8_t) 0x30)
#define LIS3DSH_CTRL_REG4_ODR_25	((uint8_t) 0x40)
#define LIS3DSH_CTRL_REG4_ODR_50	((uint8_t) 0x50)
#define LIS3DSH_CTRL_REG4_ODR_100	((uint8_t) 0x60)
#define LIS3DSH_CTRL_REG4_ODR_400	((uint8_t) 0x70)
#define LIS3DSH_CTRL_REG4_ODR_800	((uint8_t) 0x80)
#define LIS3DSH_CTRL_REG4_ODR_1600	((uint8_t) 0x90)

#define LIS3DSH_CTRL_REG4_BDU_CONT	((uint8_t) 0x00)
#define LIS3DSH_CTRL_REG4_BDU_EN	((uint8_t) 0x08)

#define LIS3DSH_CTRL_REG4_ZEN_EN	((uint8_t) 0x04)
#define LIS3DSH_CTRL_REG4_YEN_EN	((uint8_t) 0x02)
#define LIS3DSH_CTRL_REG4_XEN_EN	((uint8_t) 0x01)

/* CTRL_REG3 register */
#define LIS3DSH_CTRL_REG3_DR_EN		((uint8_t) 0x80)

#define LIS3DSH_CTRL_REG3_IEA_LOW	((uint8_t) 0x00)
#define LIS3DSH_CTRL_REG3_IEA_HIGH	((uint8_t) 0x40)

#define LIS3DSH_CTRL_REG3_IEL_LATCH	((uint8_t) 0x00)
#define LIS3DSH_CTRL_REG3_IEL_PULSE	((uint8_t) 0x20)

#define LIS3DSH_CTRL_REG3_INT2_EN	((uint8_t) 0x10)
#define LIS3DSH_CTRL_REG3_INT1_EN	((uint8_t) 0x08)

/* CTRL_REG5 register */
#define LIS3DSH_CTRL_REG5_FSCALE_2	((uint8_t) 0x00)
#define LIS3DSH_CTRL_REG5_FSCALE_4	((uint8_t) 0x08)
#define LIS3DSH_CTRL_REG5_FSCALE_6	((uint8_t) 0x10)
#define LIS3DSH_CTRL_REG5_FSCALE_8	((uint8_t) 0x18)
#define LIS3DSH_CTRL_REG5_FSCALE_16	((uint8_t) 0x20)

#define LIS3DSH_CTRL_REG5_SIM_4WIRE	((uint8_t) 0x00)
#define LIS3DSH_CTRL_REG5_SIM_3WIRE	((uint8_t) 0x01)

#define LIS3DSH_EARTH_GRAVITY		((float) -9.81f)


#define SPI_CS_ENABLE(a_s_gpio_config) 	HAL_GPIO_WritePin(a_s_gpio_config.GPIO_Port, a_s_gpio_config.GPIO_Pin, GPIO_PIN_RESET)
#define SPI_CS_DISABLE(a_s_gpio_config) HAL_GPIO_WritePin(a_s_gpio_config.GPIO_Port, a_s_gpio_config.GPIO_Pin, GPIO_PIN_SET)


typedef enum {
	LIS3DSH_OK,

	LIS3DSH_INIT_ERROR,
	LIS3DSH_SCALE_ERROR,
	LIS3DSH_READ_ERROR,
	LIS3DSH_WRITE_ERROR,

	LIS3DSH_GET_AXIS_X_ERROR,
	LIS3DSH_GET_AXIS_Y_ERROR,
	LIS3DSH_GET_AXIS_Z_ERROR,
}t_e_lis3dsh_error;

typedef enum {
	FALSE,
	TRUE
}t_e_bool;

typedef struct{
	uint8_t dataReadyEnable;			/*CTRL_REG3 register (0x23) : DR_EN*/
	uint8_t polarity;					/*CTRL_REG3 register (0x23) : IEA*/
	uint8_t latching;					/*CTRL_REG3 register (0x23) : IEL*/

	t_e_bool int1_enable;				/*CTRL_REG3 register (0x23) : INT1_EN*/
	t_e_bool int2_enable;				/*CTRL_REG3 register (0x23) : INT2_EN*/
}t_s_lis3dsh_interrupt;


typedef struct{
	uint8_t SPI_Mode;					/*CTRL_5 register (0x24) : SIM*/
	uint8_t full_scale;					/*CTRL_5 register (0x24) : FSCALE*/
	uint8_t dataUpdate;					/*CTRL_4 register (0x20) : BDU*/
	uint8_t dataRate;					/*CTRL_4 register (0x20) : ODR*/
	uint8_t z_enable;					/*CTRL_4 register (0x20) : Zen*/
	uint8_t y_enable;					/*CTRL_4 register (0x20) : Yen*/
	uint8_t x_enable;					/*CTRL_4 register (0x20) : Xen*/
	t_s_lis3dsh_interrupt *int_struct;
}t_s_lis3dsh_init;

t_e_lis3dsh_error LIS3DSH_Read_reg(SPI_HandleTypeDef *hspi,
								   uint8_t reg_addr,
								   uint8_t *dataR,
								   uint8_t size);

t_e_lis3dsh_error LIS3DSH_Write_reg(SPI_HandleTypeDef *hspi,
									uint8_t reg_addr,
									uint8_t *dataW,
									uint8_t size);


t_e_lis3dsh_error LIS3DSH_Init(SPI_HandleTypeDef *hspi,
							   GPIO_TypeDef *GPIO_Port,
							   uint16_t GPIO_Pin,
							   t_s_lis3dsh_init *init_struct);

t_e_lis3dsh_error LIS3DSH_Get_axis(SPI_HandleTypeDef *hspi,
	   							   int16_t *axis);

t_e_lis3dsh_error LIS3DSH_Get_accelerations(SPI_HandleTypeDef *hspi,
		   	   	   	   	   	   	   	   	    float *accelerations);

#endif /* INC_LIS3DSH_H_ */
