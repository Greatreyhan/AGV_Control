/*
 * Aktuator.c
 *
 *  Created on: Aug 21, 2023
 *      Author: Maulana Reyhan Savero
 */
 
#include "Aktuator.h"

void aktuator_up(aktuator_t drv){
	HAL_GPIO_WritePin(drv.PORT_IN1, drv.PIN_IN1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(drv.PORT_IN2, drv.PIN_IN2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(drv.PORT_IN3, drv.PIN_IN3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(drv.PORT_IN4, drv.PIN_IN4, GPIO_PIN_RESET);
}

void aktuator_down(aktuator_t drv){
	HAL_GPIO_WritePin(drv.PORT_IN1, drv.PIN_IN1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(drv.PORT_IN2, drv.PIN_IN2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(drv.PORT_IN3, drv.PIN_IN3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(drv.PORT_IN4, drv.PIN_IN4, GPIO_PIN_SET);
}