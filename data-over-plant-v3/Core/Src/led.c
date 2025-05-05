/*
 * led.c
 *
 *  Created on: May 5, 2025
 *      Author: gilda
 */

#include "led.h"
#include "main.h"


void set_led3()
{
	HAL_GPIO_WritePin(GPIOG, LD3_Pin, GPIO_PIN_SET);
}

void reset_led3()
{
	HAL_GPIO_WritePin(GPIOG, LD3_Pin, GPIO_PIN_RESET);
}

void set_led4()
{
	HAL_GPIO_WritePin(GPIOG, LD4_Pin, GPIO_PIN_SET);
}

void reset_led4()
{
	HAL_GPIO_WritePin(GPIOG, LD4_Pin, GPIO_PIN_RESET);
}
