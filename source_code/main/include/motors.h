#ifndef MOTORS_H
#define MOTORS_H

#include "main.h"

/// @brief Struct for controlling the stepper motors
typedef struct {
    uint8_t en_pin_number;
    GPIO_TypeDef *en_pin_port;
    
    uint8_t dir_pin_number;
    GPIO_TypeDef *dir_pin_port;
    uint8_t set_direction;

    TIM_TypeDef *timer;

    uint16_t duty;

    int16_t req_speed;
    int16_t cur_speed;
    
} tmc_controller;

tmc_controller driver1;
tmc_controller driver2;
tmc_controller driver3;
tmc_controller driver4;

/// @brief Updates all motors speed and direction
/// @param acceleration how much the motor accelerates to the required speed
void update_motors(tmc_controller *drivers[], int16_t acceleration);

/// @brief Initializes all direction pins
void init_direction_pins(tmc_controller *drivers[]);

/// @brief Disables all motors - also callable from Interrupt
void disable_all_motors();

/// @brief Initializes all enable pins
/// @param drivers tmc_controller array
void init_enable_pins(tmc_controller *drivers[]);

/// @brief Initializes all timers
void init_timers();

#endif