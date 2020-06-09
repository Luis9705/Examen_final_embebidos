/*
 * LED.c
 *
 *  Created on: May 9, 2020
 *      Author: FLL1GA
 */

/// @file i2c_adc.c
/// @brief Functions to use I2C_ADC

#include <buttons.h>		///<LED header
#include "gpio.h"		///<GPIO header


/**
 * @brief Sets up functions needed to manage the LED.
 * @retval None
 */
void buttons_setup(void){
	MX_GPIO_Init();
}


/**
 * @brief Turns on the Min Temperature LED.
 * @return min_led_status
 */
button_statusType getButtonUpThr_STATUS(void){
	return (HAL_GPIO_ReadPin(UP_THR_GPIO_Port,UP_THR_Pin)==GPIO_PIN_SET) ? BUTTON_ON: BUTTON_OFF;
}


/**
 * @brief Turns on the Min Temperature LED.
 * @return min_led_status
 */
button_statusType getButtonDownThr_STATUS(void){
	return (HAL_GPIO_ReadPin(DOWN_THR_GPIO_Port,DOWN_THR_Pin)==GPIO_PIN_SET) ? BUTTON_ON: BUTTON_OFF;
}
