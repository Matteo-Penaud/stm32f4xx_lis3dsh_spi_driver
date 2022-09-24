/*
 * serial.h
 *
 *  Created on: 21 sept. 2022
 *      Author: Matt
 */

#ifndef INC_SERIAL_H_
#define INC_SERIAL_H_

#include "stm32f4xx_ll_usart.h"

#define SERIAL_MAX_MSG_SIZE	256

uint8_t* __uint_to_string(uint32_t value);

void serial_send_string(UART_HandleTypeDef* huart, uint8_t* string, uint16_t size);

#endif /* INC_SERIAL_H_ */
