/*
 * led.c
 *
 *  Created on: Sep 19, 2023
 *      Author: pansamic
 */
#include <led.h>

void Set_RGBLED(uint8_t Red, uint8_t Green, uint8_t Blue)
{
	if(Red)
		HAL_GPIO_WritePin(LEDR_GPIO_Port,LEDR_Pin,GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LEDR_GPIO_Port,LEDR_Pin,GPIO_PIN_SET);
	if(Green)
		HAL_GPIO_WritePin(LEDG_GPIO_Port,LEDG_Pin,GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LEDG_GPIO_Port,LEDG_Pin,GPIO_PIN_SET);
	if(Blue)
		HAL_GPIO_WritePin(LEDB_GPIO_Port,LEDB_Pin,GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LEDB_GPIO_Port,LEDB_Pin,GPIO_PIN_SET);
}
