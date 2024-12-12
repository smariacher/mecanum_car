#ifndef HELPER_H
#define HELPER_H

#include "main.h"

/// @brief Very easy function to delay code execution
/// @param time amount of nop instructions being executed
/// @return 0 after time ran out
int delay(uint32_t time);

/// @brief Converts a char string to uint16
/// @param str 
/// @return 
uint16_t string_to_uint16(const char* str);

/// @brief Converts a char string to int16
/// @param str 
/// @return 
int16_t string_to_int16(const char* str);

/// @brief Function to set alternate functions for GPIO ports
/// @param port GPIOx port
/// @param pin 
/// @param alt_fn Can be looked up in stm32f401 manual
void gpio_alt(GPIO_TypeDef *port, uint32_t pin, uint8_t alt_fn);

/// @brief Calculates the value to be sent into the auto-reload register from a given frequency
/// @param freq 
/// @param timer The prescale registers timer
/// @return 
uint16_t arr_from_freq(uint16_t freq, TIM_TypeDef *timer);


#endif

