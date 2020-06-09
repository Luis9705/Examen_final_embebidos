/*
 * print.c
 *
 *  Created on: May 9, 2020
 *      Author: FLL1GA
 */
/// @file print.c
/// @brief Print functions.
#include "print.h"			///<print header file.
#include "miniprintf.h"		///<miniprintf header file.
#include "uc_uart.h"		///<uc_uart header file.

/**
 * @brief Sets up the device used to print info.
 * @retval None
 */
void print_setup(void){

	uart_setup();
}

/**
 * @brief Print function used.
 * @param[in] format - String to print
 * @retval None
 */
void print(const char *format, ...){
    va_list args;

    va_start(args, format);
    mini_vprintf_cooked(uart_putc, format, args);
    va_end(args);
}

