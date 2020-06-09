/*
 * temp_sensor.h
 *
 *  Created on: May 2, 2020
 *      Author: FLL1GA
 */
/// @file LIDAR.h
/// @brief LIDAR sensor header.
#ifndef LIDAR_H_
#define LIDAR_H_


#include <stdint.h>
#include "main.h"


#include "i2c_adc.h"



void LIDAR_setup(void);
uint16_t LIDAR_read(void);


#endif /* LIDAR_H_ */
