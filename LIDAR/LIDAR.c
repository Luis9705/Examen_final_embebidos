/*
 * LIDAR.c
 *
 *  Created on: May 2, 2020
 *      Author: FLL1GA
 */


/// @file LIDAR.c
/// @brief LIDAR sensor functions.
//  Copyright 2020 Copyright Equipo 2

#include <LIDAR.h>
#include "main.h"

/**
 * @brief Sets up the ADC peripherals needed for the LIDAR sensor.
 * @retval None
 */
void LIDAR_setup(void) {
    adc_pin_setup();
    adc_setup();
}

/**
 * @brief Reads the ADC value and returns the distance in centimeters.
 * @retval distance
 */
uint16_t LIDAR_read(void) {
    uint16_t adc_data = adc_read();
    return (uint16_t)(adc_convert_voltage(adc_data)*100.0);
}
