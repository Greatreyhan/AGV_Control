/*
 * Motor_Library.c
 *
 *  Created on: Aug 1, 2023
 *      Author: Maulana Reyhan Savero
 */
 
#include "Motor_Library.h"

void agv_run_motor(motor_t motor, int16_t speed){
	if(speed > 0){
		HAL_GPIO_WritePin(motor.EN_PORT_R, motor.EN_PIN_R, GPIO_PIN_SET);
		HAL_GPIO_WritePin(motor.EN_PORT_L, motor.EN_PIN_L, GPIO_PIN_RESET);
		if(motor.channel_R == 1){
			motor.tim_number_R->CCR1 = speed;
			HAL_TIM_PWM_Start(motor.tim_R, TIM_CHANNEL_1);
		}
		else if(motor.channel_R == 2){
			motor.tim_number_R->CCR2 = speed;
			HAL_TIM_PWM_Start(motor.tim_R, TIM_CHANNEL_2);
		}
		else if(motor.channel_R == 3){
			motor.tim_number_R->CCR3 = speed;
			HAL_TIM_PWM_Start(motor.tim_R, TIM_CHANNEL_3);
		}
		else if(motor.channel_R == 4){
			motor.tim_number_R->CCR4 = speed;
			HAL_TIM_PWM_Start(motor.tim_R, TIM_CHANNEL_4);
		}
	}
	else if(speed < 0){
		HAL_GPIO_WritePin(motor.EN_PORT_R, motor.EN_PIN_R, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor.EN_PORT_L, motor.EN_PIN_L, GPIO_PIN_SET);
		if(motor.channel_L == 1){
			motor.tim_number_L->CCR1 = speed;
			HAL_TIM_PWM_Start(motor.tim_L, TIM_CHANNEL_1);
		}
		else if(motor.channel_L == 2){
			motor.tim_number_L->CCR2 = speed;
			HAL_TIM_PWM_Start(motor.tim_L, TIM_CHANNEL_2);
		}
		else if(motor.channel_L == 3){
			motor.tim_number_L->CCR3 = speed;
			HAL_TIM_PWM_Start(motor.tim_L, TIM_CHANNEL_3);
		}
		else if(motor.channel_L == 4){
			motor.tim_number_L->CCR4 = speed;
			HAL_TIM_PWM_Start(motor.tim_L, TIM_CHANNEL_4);
		}
	}
}

void agv_encoder_start(encoder_t encoder, TIM_HandleTypeDef* tim,TIM_TypeDef* tim_number){
	encoder.tim = tim;
	encoder.tim_number = tim_number;
	HAL_TIM_Encoder_Start_IT(tim, TIM_CHANNEL_ALL);
}

 