/*
 * serial.c
 *
 *  Created on: 21 sept. 2022
 *      Author: Matt
 */

#include "serial.h"

uint16_t __sizeof_string(uint8_t* string)
{
	uint16_t result;

	for(result = 0; string[result] != '\0'; result++);

	result++;

	return result;
}

void serial_send_string(UART_HandleTypeDef* huart, uint8_t* string, uint16_t size)
{
	HAL_UART_Transmit(huart, string, size, 10000);
}
