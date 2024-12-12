#include "motors.h"

void update_motors(tmc_controller *drivers[], int16_t acceleration){
    for (int i = 0; i < 4; i++){
        if (drivers[i]->cur_speed < drivers[i]->req_speed){
            drivers[i]->cur_speed += acceleration;
        }
        if (drivers[i]->cur_speed > drivers[i]->req_speed){
            drivers[i]->cur_speed -= acceleration;
        }
    }

    for (int i = 0; i < 4; i++){
        if (drivers[i]->cur_speed < 0){
            drivers[i]->dir_pin_port->BSRR |= GPIO_BSRR_BR0 << drivers[i]->dir_pin_number;
        } else {
            drivers[i]->dir_pin_port->BSRR |= GPIO_BSRR_BS0 << drivers[i]->dir_pin_number;
        }
    }

    TIM1->ARR = arr_from_freq(abs(driver1.cur_speed), TIM1) * 2;
    TIM1->CCR1 = arr_from_freq(abs(driver1.cur_speed), TIM1);

    // TODO: Reevaluate this calculation
    TIM9->ARR = arr_from_freq(abs(driver2.cur_speed), TIM9) * 4;
    TIM9->CCR1 = arr_from_freq(abs(driver2.cur_speed), TIM9);
    

    TIM4->ARR = arr_from_freq(abs(driver3.cur_speed), TIM4) * 2;
    TIM4->CCR1 = arr_from_freq(abs(driver3.cur_speed), TIM4);

    TIM3->ARR = arr_from_freq(abs(driver4.cur_speed), TIM3) * 2;
    TIM3->CCR1 = arr_from_freq(abs(driver4.cur_speed), TIM3);
}


void init_direction_pins(tmc_controller *drivers[]){
    // Set direction pins accordingly
    for (int i = 0; i < 4; i++){
        drivers[i]->dir_pin_port->MODER |= GPIO_MODER_MODER0_0 << (drivers[i]->dir_pin_number*2);
        drivers[i]->dir_pin_port->OTYPER &= ~(GPIO_OTYPER_OT_0 << drivers[i]->dir_pin_number);
        drivers[i]->dir_pin_port->BSRR |= GPIO_BSRR_BR0 << drivers[i]->dir_pin_number;
    }
}

void disable_all_motors(){
    tmc_controller *drivers[4] = {&driver1, &driver2, &driver3, &driver4};
    for (int i = 0; i < 4; i++){
        drivers[i]->en_pin_port->BSRR |= GPIO_BSRR_BS0 << drivers[i]->en_pin_number;
    }
}

void init_enable_pins(tmc_controller *drivers[]){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    for (int i = 0; i < 4; i++){
        drivers[i]->en_pin_port->MODER |= GPIO_MODER_MODER0_0 << (2 * drivers[i]->en_pin_number);
        drivers[i]->en_pin_port->OTYPER &= ~(GPIO_OTYPER_OT_0 << drivers[i]->en_pin_number);
        drivers[i]->en_pin_port->BSRR |= GPIO_BSRR_BS0 << drivers[i]->en_pin_number;
    }

    // Wait for motors to slow down after possible reset
    delay(1000000);

    for (int i = 0; i < 4; i++){
        drivers[i]->en_pin_port->BSRR |= GPIO_BSRR_BR0 << drivers[i]->en_pin_number;
    }
}

void init_timers(){
    // Enable GPIOB port 6 and set to alt func 2 (TIM4_CH1)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    gpio_alt(GPIOB, 6, 2);

    // Enable GPIOB port 4 and set to alt func 2
    gpio_alt(GPIOB, 4, 2);

    // Enable GPIOA port 8 and set to alt func 1
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    gpio_alt(GPIOA, 8, 1);
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8_1;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD8_Msk;

    // Enable GPIOE port 5 and set to alt func 3
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    gpio_alt(GPIOE, 5, 3);

    // Enable GPIOB port 3 and set to alt func 1 for back light
    gpio_alt(GPIOB, 3, 1);

    // Enable GPIOB port 10 and set to alt func 1 for front light
    gpio_alt(GPIOB, 10, 1);


    // Enable TIM4 peripheral
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    // Enable TIM3 peripherals
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // Enable TIM1 peripherals
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // Enable TIM9 peripherals
    RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;

    // Enable TIM2 peripherals
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Buffer the ARR register
    TIM4->CR1 |= TIM_CR1_ARPE;

    TIM3->CR1 |= TIM_CR1_ARPE;

    TIM1->CR1 |= TIM_CR1_ARPE;
    TIM1->CR2 = 0;

    TIM9->CR1 |= TIM_CR1_ARPE;

    TIM2->CR1 |= 0;
    TIM2->CCER = 0;
    TIM2->CR1 |= TIM_CR1_ARPE;

    GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED3_Msk;
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT3;

    // Set the prescaler and overflow values
    TIM4->PSC = 3;
    TIM4->ARR = 64000-1;
    TIM4->CCR1 = 0; // Set duty cycle initially to 0 (CCRx for channel)

    TIM3->PSC = 3;
    TIM3->ARR = 64000-1;
    TIM3->CCR1 = 0; 

    TIM1->PSC = 3;
    TIM1->ARR = 64000-1;
    TIM1->CCR1 = 0; 
    
    TIM9->PSC = 3;
    TIM9->ARR = 64000-1;
    TIM9->CCR1 = 0;

    TIM2->PSC = 0;
    TIM2->ARR = arr_from_freq(4000, TIM2);

    TIM4->CCMR1 |= (TIM_CCMR1_OC1PE | (0b110 << TIM_CCMR1_OC1M_Pos));

    TIM3->CCMR1 |= (TIM_CCMR1_OC1PE | (0b110 << TIM_CCMR1_OC1M_Pos));

    TIM1->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
    TIM1->CCMR1 |= (TIM_CCMR1_OC1PE | (0b110 << TIM_CCMR1_OC1M_Pos));

    TIM9->CCMR1 |= (TIM_CCMR1_OC1PE | (0b110 << TIM_CCMR1_OC1M_Pos));

    TIM2->CCMR1 &= ~(TIM_CCMR1_OC2M_Msk);  // Clear mode bits
    TIM2->CCMR1 |= (0b110 << TIM_CCMR1_OC2M_Pos);  // PWM mode 1
    TIM2->CCMR1 |= TIM_CCMR1_OC2PE;  // Enable preload
    TIM2->CCR2 = 0;

    TIM2->CCMR2 &= ~(TIM_CCMR2_OC3M);
    TIM2->CCMR2 |= (0b110 << TIM_CCMR2_OC3M_Pos);
    TIM2->CCMR2 |= TIM_CCMR2_OC3PE;
    TIM2->CCR3 = 0;

    // Enable Capture Compare for channel 1
    TIM4->CCER |= TIM_CCER_CC1E;
    TIM3->CCER |= TIM_CCER_CC1E;
    TIM1->CCER |= TIM_CCER_CC1E;
    TIM9->CCER |= TIM_CCER_CC1E;

    // Enable Capture Compare for channel 2,3
    TIM2->CCER |= TIM_CCER_CC2E;
    TIM2->CCER |= TIM_CCER_CC3E;
    TIM2->EGR |= TIM_EGR_UG;

    // Enable the timer and set to center-aligned mode
    TIM4->CR1 |= TIM_CR1_CMS_0 | TIM_CR1_CEN;
    TIM3->CR1 |= TIM_CR1_CMS_0 | TIM_CR1_CEN;
    TIM1->CR1 |= TIM_CR1_CMS_0 | TIM_CR1_CEN;
    TIM9->CR1 |= TIM_CR1_CMS_0 | TIM_CR1_CEN;
    TIM2->CR1 |= TIM_CR1_CMS_0 | TIM_CR1_CEN;

    //only for TIM1
    TIM1->BDTR |= TIM_BDTR_MOE;
}
