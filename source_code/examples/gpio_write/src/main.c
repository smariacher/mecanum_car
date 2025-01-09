#include "main.h"

int delay(uint32_t time){
    for (uint32_t i = 0; i < time; i++){
        asm("nop");
    }
    return 0;
}

int main(){
    uint8_t gpio_pin = 2;

    // Enable clock for GPIOA
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;    

    // Set GPIOA pin x as output
    GPIOA->MODER &= ~GPIO_MODER_MODER0_Msk;
    GPIOA->MODER = GPIO_MODER_MODER0_0 << (2*gpio_pin);

    // Set GPIOA pin x as push-pull output
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_0 << gpio_pin);

    for (;;){
        GPIOA->BSRR |= GPIO_BSRR_BS0 << gpio_pin;
        delay(100);
        GPIOA->BSRR |= GPIO_BSRR_BR0 << gpio_pin;
        delay(100);
    }
    return 0;
}