/*
 * buttons.c
 *
 *  Created on: May 9, 2020
 *      Author: FLL1GA
 */

/// @file buttons.c
/// @brief Functions used by the buttons' system.

#include <buttons.h> ///<Buttons header
#include "gpio.h"	///<GPIO header


/**
 * @brief Sets up functions needed to manage the buttons.
 * @retval None
 */
void buttons_setup(void){
	MX_GPIO_Init();
}


/**
 * @brief Returns the UP_THR button status.
 * @return UP_THR_status
 */
button_statusType getButtonUpThr_STATUS(void){
	return (HAL_GPIO_ReadPin(UP_THR_GPIO_Port,UP_THR_Pin)==GPIO_PIN_SET) ? BUTTON_ON: BUTTON_OFF;
}


/**
 * @brief Returns the DOWN_THR button status.
 * @return DOWN_THR_status
 */
button_statusType getButtonDownThr_STATUS(void){
	return (HAL_GPIO_ReadPin(DOWN_THR_GPIO_Port,DOWN_THR_Pin)==GPIO_PIN_SET) ? BUTTON_ON: BUTTON_OFF;
}
