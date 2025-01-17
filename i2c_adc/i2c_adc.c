/*
 * i2c_adc.c
 *
 *  Created on: May 2, 2020
 *      Author: FLL1GA
 */

/// @file i2c_adc.c
/// @brief Functions to use I2C_ADC
//  Copyright 2020 Copyright Equipo 2
#include <stdint.h>		///<stdint header
#include <stdbool.h>	///<stdbool header
#include "i2c_adc.h"	///<i2c_adc header
#include "main.h"		///<main header
#include "i2c.h"		///<i2c header

I2C_HandleTypeDef * hi2c;				  ///<I2C Handle

uint8_t ads1115_addr = 0b01001000 <<1;            ///<I2C ADC Address.
uint8_t conversion_reg = 0x0;                     ///<Conversion register address.
uint8_t config_reg = 0x1;                         ///<Configuration register address.
uint8_t config_msb = 0b11000010;  // 0b11000100;     /// <Start a single
// conversion (when in power-down state). AINP = AIN0 and AINN = GND,
// FSR = ±4.096 V, Continuous-conversion mode.
uint8_t config_lsb = 0b10000011;                  ///<128 SPS, Disable
// comparator and set ALERT/RDY pin to high-impedance.

/**
 * @brief Sets up the ADC pins needed.
 * @retval None
 */
void adc_pin_setup(void) {}

/**
 * @brief Sets up the adc using I2C communication.
 * @retval None
 */
void adc_setup(void) {

	MX_I2C1_Init();

	hi2c = &hi2c1;

    unsigned char buff[3] = {config_reg,config_msb, config_lsb};

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit (hi2c, ads1115_addr,(uint8_t *) buff, 3, 10000);

    if(status != HAL_OK){
    	return;
    }
}

/**
 * @brief Reads the value of the ADC.
 * @retval data
 */
uint16_t adc_read(void) {
    uint8_t msb_data = 0x0;        /// MSB Read I2C byte
    uint8_t lsb_data = 0x0;        /// LSB Read I2C byte
    uint16_t data = 0;        /// Full data

    unsigned char aRxBuffer[2] = {0};

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, ads1115_addr, conversion_reg, I2C_MEMADD_SIZE_8BIT,(uint8_t *)aRxBuffer, 2, 10000);

    if(status != HAL_OK){
    	return 0;
    }

    msb_data = aRxBuffer[0];
    lsb_data = aRxBuffer[1];
    data = (msb_data <<8) + lsb_data;
    // int temp;
    // temp = data * 210 / (65536/2);
    return data;
}

/**
 * @brief Scales the adc value to a given voltage scale.
 * @param[in] data
 * @retval scaled_value
 */
float adc_convert_voltage(uint16_t data) {
    float scale = 125E-6;  // 62.5E-6;
    return ((float)(data))*scale;
}

