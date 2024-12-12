#include "helper_func.h"

uint16_t string_to_uint16(const char* str) {
    uint16_t result = 0;
    for (size_t i = 0; i < strlen(str); i++) {
        if (str[i] >= '0' && str[i] <= '9') {
            result = result * 10 + (str[i] - '0');
        } else {
            // Handle non-digit characters if needed
            break;
        }
    }
    return result;
}

int delay(uint32_t time){
    for (uint32_t i = 0; i < time; i++){
        asm("nop");
    }
    return 0;
}

int16_t string_to_int16(const char* str) {
    int16_t result = 0;
    int16_t sign = 1;  // Variable to track sign

    size_t i = 0;
    if (str[0] == '-') {  // Check if the number is negative
        sign = -1;
        i = 1;  // Start parsing from the next character
    }

    for (; i < strlen(str); i++) {
        if (str[i] >= '0' && str[i] <= '9') {
            result = result * 10 + (str[i] - '0');
        } else {
            // Handle non-digit characters if needed
            break;
        }
    }

    return result * sign;
}

void gpio_alt(GPIO_TypeDef *port, uint32_t pin, uint8_t alt_fn){
    port->MODER &= ~(0b11 << (pin * 2));
    port->MODER |= (0b10 << (pin*2));

    int pos = pin * 4;
    uint8_t idx = 0;
    if (pin > 7){
        pos -= 32;
        idx = 1;
    }

    port->AFR[idx] &= ~(0b1111 << pos); 
    port->AFR[idx] |= (alt_fn << pos);
}

uint16_t arr_from_freq(uint16_t freq, TIM_TypeDef *timer){
    return (uint16_t)(F_CLK/(freq*(timer->PSC+1)) - 1);
}