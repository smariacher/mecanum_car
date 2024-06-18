#include "main.h"
#include "clock_config.h"

int delay(uint32_t time){
    for (uint32_t i = 0; i < time; i++){
        asm("nop");
    }
    return 0;
}

int main(){
    uint8_t gpio_pin = 10;

    // Enable clock for GPIOC
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;    

    // Set GPIOC pin x as output
    GPIOC->MODER |= GPIO_MODER_MODER0_0 << (2*gpio_pin);

    // Set GPIOC pin x as push-pull output
    GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_0 << gpio_pin);

    for (;;){
        GPIOC->BSRR |= GPIO_BSRR_BS0 << gpio_pin;
        delay(100000);
        GPIOC->BSRR |= GPIO_BSRR_BR0 << gpio_pin;
        delay(100000);
    }


    return 0;
}