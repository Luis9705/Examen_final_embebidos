/*
 * i2c_adc.h
 *
 *  Created on: May 2, 2020
 *      Author: FLL1GA
 */
/// @file i2c_adc.h
/// @brief I2C ADC Header file.
#ifndef I2C_ADC_H_
#define I2C_ADC_H_

#include <stdint.h>

void adc_pin_setup(void);
void adc_setup( void);
uint16_t adc_read(void);
float adc_convert_voltage(uint16_t data);

#endif /* I2C_ADC_H_ */
