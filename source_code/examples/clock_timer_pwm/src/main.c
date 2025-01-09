#include "main.h"
#include "clock_config.h"

int delay(uint32_t time){
    for (uint32_t i = 0; i < time; i++){
        asm("nop");
    }
    return 0;
}

uint16_t arr_from_freq(uint16_t freq){
    return F_CLK/(freq*(TIM2->PSC+1)) - 1;
}

void gpio_alt(GPIO_TypeDef *port, uint32_t pin, uint8_t alt_fn){
    // Clear the mode bits first
    port->MODER &= ~(0b11 << (pin*2));
    // Set to alternate function mode (0b10)
    port->MODER |= (0b10 << (pin*2));

    int pos = pin*4;
    uint8_t idx = 0;
    if (pin > 7){
        pos -= 32;
        idx = 1;
    }
    // Clear AFR bits first
    port->AFR[idx] &= ~(0xF << pos);
    // Set new alternate function
    port->AFR[idx] |= (alt_fn << pos);
}

int main(){
    configure_clock();

    // Enable GPIOB clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    
    // Configure PB3 as AF1 for TIM2_CH2
    gpio_alt(GPIOB, 3, 1);

    // Enable TIM2 peripheral
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Reset timer configuration
    TIM2->CR1 = 0;
    TIM2->CCER = 0;

    // Buffer the ARR register
    TIM2->CR1 |= TIM_CR1_ARPE;

    // Set initial frequency to 4kHz
    uint16_t pwm_frequency = 4000;
    TIM2->PSC = 0;
    TIM2->ARR = arr_from_freq(pwm_frequency);
    
    // Configure Channel 2 as PWM mode 1 (110)
    TIM2->CCMR1 &= ~(TIM_CCMR1_OC2M_Msk);  // Clear mode bits
    TIM2->CCMR1 |= (0b110 << TIM_CCMR1_OC2M_Pos);  // PWM mode 1
    TIM2->CCMR1 |= TIM_CCMR1_OC2PE;  // Enable preload
    
    // Set initial duty cycle to 1%
    TIM2->CCR2 = TIM2->ARR / 100;  // 1% of ARR

    // Enable output for channel 2
    TIM2->CCER |= TIM_CCER_CC2E;

    // Generate update event to load registers
    TIM2->EGR |= TIM_EGR_UG;
    
    // Enable counter in edge-aligned mode
    TIM2->CR1 |= TIM_CR1_CEN;

    uint8_t state = 0;
    uint16_t duty = TIM2->ARR / 100;  // Start at 1%

    for(;;){
        if (state == 0){
            duty += 100;
            if (duty >= TIM2->ARR){
                state = 1;
            }
        }

        if (state == 1){
            duty -= 100;
            if (duty <= 100){
                state = 0;
            }
        }
        
        TIM2->CCR2 = duty;
        delay(1000);
    }

    return 0;
}