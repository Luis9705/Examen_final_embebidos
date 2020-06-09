/*
 * LED.h
 *
 *  Created on: May 9, 2020
 *      Author: FLL1GA
 */
/// @file LED.h
/// @brief LED Header file.
#ifndef BUTTONS_H_
#define BUTTONS_H_

typedef enum  {
  BUTTON_ON, //!< LED_ON
  BUTTON_OFF,//!< LED_OFF
} button_statusType;

void buttons_setup(void);

button_statusType getButtonUpThr_STATUS(void);
button_statusType getButtonDownThr_STATUS(void);

#endif /* BUTTONS_H_ */
