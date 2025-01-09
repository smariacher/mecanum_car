#include "main.h"
#include "clock_config.h"

typedef struct {
    uint8_t en_pin_number;
    GPIO_TypeDef *en_pin_port;
    
    uint8_t dir_pin_number;
    GPIO_TypeDef *dir_pin_port;
    uint8_t set_direction;

    TIM_TypeDef *timer;

    uint16_t duty;
    
} tmc_controller;

int delay(uint32_t time){
    for (uint32_t i = 0; i < time; i++){
        asm("nop");
    }
    return 0;
}

uint16_t arr_from_freq(uint16_t freq, TIM_TypeDef * timer){
    return (uint16_t)(F_CLK/(freq*(timer->PSC+1)) - 1);
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

void init_enable_pin(GPIO_TypeDef en_port, uint8_t en_pin){
    GPIOD->MODER |= GPIO_MODER_MODER0_0 << (2 * en_pin);
    GPIOD->OTYPER &= ~(GPIO_OTYPER_OT_0 << en_pin);
    GPIOD->BSRR |= GPIO_BSRR_BS0 << en_pin;
}

tmc_controller driver1;
tmc_controller driver2;
tmc_controller driver3;
tmc_controller driver4;

int main(){
    configure_clock();

    driver1.en_pin_number = 0;
    driver1.en_pin_port = GPIOD;
    driver1.dir_pin_number = 8;
    driver1.dir_pin_port = GPIOD;

    driver2.en_pin_number = 1;
    driver2.en_pin_port = GPIOD;
    driver2.dir_pin_number = 9;
    driver2.dir_pin_port = GPIOD;

    driver3.en_pin_number = 2;
    driver3.en_pin_port = GPIOD;
    driver3.dir_pin_number = 10;
    driver3.dir_pin_port = GPIOD;

    driver4.en_pin_number = 3;
    driver4.en_pin_port = GPIOD;
    driver4.dir_pin_number = 11;
    driver4.dir_pin_port = GPIOD;

    tmc_controller drivers[4] = {driver1, driver2, driver3, driver4};

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

    for (int i = 0; i < 4; i++){
        init_enable_pin(*drivers[i].en_pin_port, drivers[i].en_pin_number);
    }

    // delay(1000000);
    // for (int i = 0; i < 4; i++){

    //     drivers[i].en_pin_port->BSRR |= GPIO_BSRR_BR0 << drivers[i].en_pin_number;
    // }

    // Set direction pins accordingly
    uint8_t dir_pin = 8;
    GPIOD->MODER |= GPIO_MODER_MODER0_0 << (dir_pin*2);
    GPIOD->OTYPER &= ~(GPIO_OTYPER_OT_0 << dir_pin);
    GPIOD->BSRR |= GPIO_BSRR_BR0 << dir_pin;
    // GPIOD->BSRR |= GPIO_BSRR_BS0 << dir_pin;

    dir_pin = 9;
    GPIOD->MODER |= GPIO_MODER_MODER0_0 << (dir_pin*2);
    GPIOD->OTYPER &= ~(GPIO_OTYPER_OT_0 << dir_pin);
    GPIOD->BSRR |= GPIO_BSRR_BS0 << dir_pin;
    // GPIOD->BSRR |= GPIO_BSRR_BS0 << dir_pin;

    dir_pin = 10;
    GPIOD->MODER |= GPIO_MODER_MODER0_0 << (dir_pin*2);
    GPIOD->OTYPER &= ~(GPIO_OTYPER_OT_0 << dir_pin);
    GPIOD->BSRR |= GPIO_BSRR_BR0 << dir_pin;
    // GPIOD->BSRR |= GPIO_BSRR_BS0 << dir_pin;

    dir_pin = 11;
    GPIOD->MODER |= GPIO_MODER_MODER0_0 << (dir_pin*2);
    GPIOD->OTYPER &= ~(GPIO_OTYPER_OT_0 << dir_pin);
    GPIOD->BSRR |= GPIO_BSRR_BS0 << dir_pin;
    // GPIOD->BSRR |= GPIO_BSRR_BS0 << dir_pin;


    // Enable GPIOB port 6 and set to alt func 2 (TIM4_CH1)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    gpio_alt(GPIOB, 6, 2);

    // Enable GPIOB port 4 and set to alt func 2
    gpio_alt(GPIOB, 4, 2);

    // Enable GPIOA port 0 and set to alt func 2
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    gpio_alt(GPIOA, 8, 1);
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8_1;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD8_Msk;

    // Enable GPIOE port 4 and set to alt func 2
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    gpio_alt(GPIOE, 5, 3);

    // Enable TIM4 peripheral
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    // Enable TIM3 peripherals
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // Enable TIM1 peripherals
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // Enable TIM9 peripherals
    RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;

    // Buffer the ARR register
    TIM4->CR1 |= TIM_CR1_ARPE;

    // Buffer the ARR register
    TIM3->CR1 |= TIM_CR1_ARPE;

    // Buffer the ARR register
    TIM1->CR1 |= TIM_CR1_ARPE;
    TIM1->CR2 = 0;

    // Buffer the ARR register
    TIM9->CR1 |= TIM_CR1_ARPE;

    // Set the prescaler and overflow values
    TIM4->PSC = 3;
    TIM4->ARR = 64000-1;
    TIM4->CCR1 = 0; // Set duty cycle initially to 0

    // Set the prescaler and overflow values
    TIM3->PSC = 3;
    TIM3->ARR = 64000-1;
    TIM3->CCR1 = 0; //CCRx for channel

    // Set the prescaler and overflow values
    TIM1->PSC = 3;
    TIM1->ARR = 64000-1;
    TIM1->CCR1 = 0; //CCRx for channel
    
    // Set the prescaler and overflow values
    TIM9->PSC = 3;
    TIM9->ARR = 64000-1;
    TIM9->CCR1 = 0; //CCRx for channel

    // Enable preload for channel 1 (both registers)
    TIM4->CCMR1 |= (TIM_CCMR1_OC1PE | (0b110 << TIM_CCMR1_OC1M_Pos));

    // Enable preload for channel 1 (both registers)
    TIM3->CCMR1 |= (TIM_CCMR1_OC1PE | (0b110 << TIM_CCMR1_OC1M_Pos));

    // Enable preload for channel 1 (both registers)
    TIM1->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
    TIM1->CCMR1 |= (TIM_CCMR1_OC1PE | (0b110 << TIM_CCMR1_OC1M_Pos));

    // Enable preload for channel 1 (both registers)
    TIM9->CCMR1 |= (TIM_CCMR1_OC1PE | (0b110 << TIM_CCMR1_OC1M_Pos));


    // Enable Capture Compare for channel 1
    TIM4->CCER |= TIM_CCER_CC1E;

    // Enable Capture Compare for channel 1
    TIM3->CCER |= TIM_CCER_CC1E;

    // Enable Capture Compare for channel 1
    TIM1->CCER |= TIM_CCER_CC1E;

    // Enable Capture Compare for channel 1
    TIM9->CCER |= TIM_CCER_CC1E;

    // Enable the timer and set to center-aligned mode
    TIM4->CR1 |= TIM_CR1_CMS_0 | TIM_CR1_CEN;

    // Enable the timer and set to center-aligned mode
    TIM3->CR1 |= TIM_CR1_CMS_0 | TIM_CR1_CEN;

    // Enable the timer and set to center-aligned mode
    TIM1->CR1 |= TIM_CR1_CMS_0 | TIM_CR1_CEN;

    // Enable the timer and set to center-aligned mode
    TIM9->CR1 |= TIM_CR1_CMS_0 | TIM_CR1_CEN;

    uint16_t lower_limit = 100;
    uint16_t upper_limit = 20000;
    uint16_t pwm_frequency_3 = lower_limit;
    uint16_t pwm_frequency_4 = lower_limit;
    uint16_t pwm_frequency_1 = lower_limit;
    uint16_t pwm_frequency_2 = lower_limit;
    uint16_t duty_3 = arr_from_freq(lower_limit, TIM4) / 2;
    uint16_t duty_4 = arr_from_freq(lower_limit, TIM3) / 2;
    uint16_t duty_1 = arr_from_freq(lower_limit, TIM1) / 2;
    uint16_t duty_2 = arr_from_freq(lower_limit, TIM9);
    uint8_t state = 0;

    //only for TIM1
    TIM1->BDTR |= TIM_BDTR_MOE;

    uint8_t acc = 8;

    for(;;){
        if (pwm_frequency_3 <= upper_limit && state == 0){
            pwm_frequency_3 += acc;  
        } else {
            state = 1;
        }

        if (pwm_frequency_3 >= lower_limit && state == 1){
            pwm_frequency_3 -= acc;
        } else {
            state = 0;
        }

        if (pwm_frequency_4 <= upper_limit && state == 0){
            pwm_frequency_4 += acc;  
        } else {
            state = 1;
        }

        if (pwm_frequency_4 >= lower_limit && state == 1){
            pwm_frequency_4 -= acc;
        } else {
            state = 0;
        }

        if (pwm_frequency_1 <= upper_limit && state == 0){
            pwm_frequency_1 += acc;  
        } else {
            state = 1;
        }

        if (pwm_frequency_1 >= lower_limit && state == 1){
            pwm_frequency_1 -= acc;
        } else {
            state = 0;
        }
        
        if (pwm_frequency_2 <= upper_limit && state == 0){
            pwm_frequency_2 += acc;  
        } else {
            state = 1;
        }

        if (pwm_frequency_2 >= lower_limit && state == 1){
            pwm_frequency_2 -= acc;
        } else {
            state = 0;
        }

        TIM4->ARR = arr_from_freq(pwm_frequency_3, TIM4);
        duty_3 = arr_from_freq(pwm_frequency_3, TIM4) / 2;
        TIM4->CCR1 = duty_3;

        TIM3->ARR = arr_from_freq(pwm_frequency_4, TIM3);
        duty_4 = arr_from_freq(pwm_frequency_4, TIM3)/2;
        TIM3->CCR1 = duty_4;

        TIM1->ARR = arr_from_freq(pwm_frequency_1, TIM1);
        duty_1 = arr_from_freq(pwm_frequency_1, TIM1)/2;
        TIM1->CCR1 = duty_1;

        TIM9->ARR = arr_from_freq(pwm_frequency_2, TIM9) * 2;
        duty_2 = arr_from_freq(pwm_frequency_2, TIM9);
        TIM9->CCR1 = duty_2;
        
        delay(500);
    }

    return 0;
}
