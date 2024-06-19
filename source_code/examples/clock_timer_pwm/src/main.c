#include "main.h"
#include "clock_config.h"

int delay(uint32_t time){
    for (uint32_t i = 0; i < time; i++){
        asm("nop");
    }
    return 0;
}

uint16_t arr_from_freq(uint16_t freq){
    return F_CLK/(freq*(TIM3->PSC+1)) - 1;
}

void gpio_alt(GPIO_TypeDef *port, uint32_t pin, uint8_t alt_fn){
    port->MODER |= (0b10 << (pin*2));

    int pos = pin*4;
    uint8_t idx = 0;
    if (pin > 7){
        pos -= 32;
        idx = 1;
    }

    port->AFR[idx] |= (alt_fn << pos);
}

int main(){
    configure_clock();

    // Enable GPIOB port 5 and set to alt func 2
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    gpio_alt(GPIOB, 5, 2);

    // Enable TIM3 peripherals
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // Buffer the ARR register
    TIM3->CR1 |= TIM_CR1_ARPE;

    // Set the prescaler and overflow values
    TIM3->PSC = 0;
    TIM3->ARR = 64000;
    TIM3->CCR2 = 0;

    // Enable preload for channel 2
    TIM3->CCMR1 |= (TIM_CCMR1_OC2PE | (0b110 << TIM_CCMR1_OC2M_Pos));

    // Enable Capture Compare for channel 2
    TIM3->CCER |= TIM_CCER_CC2E;

    // Enable the timer and set to center-aligned mode
    TIM3->CR1 |= TIM_CR1_CMS_0 | TIM_CR1_CEN;
    
    uint8_t state = 0;
    uint16_t lower_limit = 1000; // One rotation per minute because of step/8 microstepping
    uint16_t upper_limit = 50;
    uint16_t pwm_frequency = lower_limit;
    uint16_t duty = arr_from_freq(lower_limit)/2;

    uint8_t level = 10;

    for(;;){
        // if (state == 0){
        //     if (pwm_frequency <= upper_limit){
        //         pwm_frequency += 1;
        //     } else {
        //         state = 1;
        //     }
        // } else if (state == 1){
        //     if (pwm_frequency >= lower_limit){
        //         pwm_frequency -= 1;
        //     } else {
        //         state = 0;
        //     }
        // }
        
        if (!state){
            level -= 1;
            if (level <= 1){state = 1;}
        }

        if (state){
            level += 1;
            if (level >= arr_from_freq(pwm_frequency)){state = 0;}
        }

        TIM3->ARR = arr_from_freq(pwm_frequency);
        duty = level;
        TIM3->CCR2 = duty*10;
        delay(500);
    }

    return 0;
}