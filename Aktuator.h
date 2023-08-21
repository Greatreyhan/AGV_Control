/*
 * Aktuator.h
 *
 *  Created on: Aug 21, 2023
 *      Author: Maulana Reyhan Savero
 */
 
#ifndef AKTUATOR_LIBRARY_H_
#define AKTUATOR_LIBRARY_H_

#include "main.h"

typedef struct{
	GPIO_TypeDef*				PORT_IN1;
	GPIO_TypeDef*				PORT_IN2;
	GPIO_TypeDef*				PORT_IN3;
	GPIO_TypeDef*				PORT_IN4;
	uint16_t						PIN_IN1;
	uint16_t						PIN_IN2;
	uint16_t						PIN_IN3;
	uint16_t						PIN_IN4;
}aktuator_t;

void aktuator_up(aktuator_t drv);
void aktuator_down(aktuator_t drv);
#endif
