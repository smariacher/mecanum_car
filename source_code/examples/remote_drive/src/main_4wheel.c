#include "main.h"
#include "clock_config.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#define BAUDRATE 9600
#define APB_FREQ 16000000
#define BUFFER_SIZE 128

volatile char rxBuffer[BUFFER_SIZE];
volatile uint32_t rxIndex = 0;

void USART2_IRQHandler(void);
void USART2_Init(void);

#define LOG( msg... ) printf( msg );

typedef struct {
    uint8_t en_pin_number;
    GPIO_TypeDef *en_pin_port;
    
    uint8_t dir_pin_number;
    GPIO_TypeDef *dir_pin_port;
    uint8_t set_direction;

    TIM_TypeDef *timer;

    uint16_t duty;

    uint16_t req_speed;
    
} tmc_controller;

int delay(uint32_t time){
    for (uint32_t i = 0; i < time; i++){
        asm("nop");
    }
    return 0;
}

int _write(int handle, char* data, int size){
    int count = size;
    while (count--){
        while(!(USART2->SR & USART_SR_TXE)) {;}
        USART2->DR = *data++;
    }
    return size;
}

uint16_t arr_from_freq(uint16_t freq, TIM_TypeDef * timer){
    return (uint16_t)(F_CLK/(freq*(timer->PSC+1)) - 1);
}

uint16_t stringToUint16(const char* str) {
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

void USART2_Init(void){
    // Enable peripheral GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    // Enable peripheral USART2 clock
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // Configure PA2 as USART2_TX using alternate function 7
    GPIOA->MODER &= ~GPIO_MODER_MODER2_Msk;
    GPIOA->MODER |= GPIO_MODER_MODER2_1;
    GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL2_Msk;
    GPIOA->AFR[0] |= 7U << GPIO_AFRL_AFSEL2_Pos;

    // Configure PA3 as USART2_RX using alternate function 7
    GPIOA->MODER &= ~GPIO_MODER_MODER3_Msk;
    GPIOA->MODER |= GPIO_MODER_MODER3_1;
    GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL3_Msk;
    GPIOA->AFR[0] |= 7U << GPIO_AFRL_AFSEL3_Pos;

    // Clear CR1 register
    USART2->CR1 = 0;
    // Enable transmitter and receiver
    USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE);
    
    // Enable RXNE interrupt
    USART2->CR1 |= USART_CR1_RXNEIE;

    // Set baudrate 
    USART2->BRR = (APB_FREQ / BAUDRATE);

    // Enable USART2
    USART2->CR1 |= USART_CR1_UE;

    // Enable USART2 interrupt in NVIC
    NVIC_SetPriority(USART2_IRQn, 0);
    NVIC_EnableIRQ(USART2_IRQn);
}

tmc_controller driver1;
tmc_controller driver2;
tmc_controller driver3;
tmc_controller driver4;

void USART2_IRQHandler(void){
    if (USART2->SR & USART_SR_RXNE){
        char receivedChar = (char)(USART2->DR & 0xFF);

        if (rxIndex < BUFFER_SIZE - 1){
            rxBuffer[rxIndex++] = receivedChar;

            // Echo the received character
            while (!(USART2->SR & USART_SR_TXE));
            USART2->DR = receivedChar;

            if (receivedChar == '\r'){
                rxBuffer[rxIndex] = '\0';
                rxIndex = 0;
                
                // Process the received command here
                char string1[5];
                char string2[5];
                char string3[5];
                char string4[5];
                
                string1[5] = '\0';
                string2[5] = '\0';
                string3[5] = '\0';
                string4[5] = '\0';

                memcpy(string1, rxBuffer + 1, 4 * sizeof(char));
                memcpy(string2, rxBuffer + 6, 4 * sizeof(char));
                memcpy(string3, rxBuffer + 11, 4 * sizeof(char));
                memcpy(string4, rxBuffer + 16, 4 * sizeof(char));

                uint16_t speed1 = stringToUint16(string1);
                uint16_t speed2 = stringToUint16(string2);
                uint16_t speed3 = stringToUint16(string3);
                uint16_t speed4 = stringToUint16(string4);

                LOG("Speed1=%d, Speed2=%d, Speed3=%d, Speed4=%d\r\n", speed1, speed2, speed3, speed4);
            
                uint8_t dir_pin = 8;
                if (rxBuffer[0] == '-'){GPIOD->BSRR |= GPIO_BSRR_BR0 << dir_pin;} else {GPIOD->BSRR |= GPIO_BSRR_BS0 << dir_pin;}
                dir_pin = 9;
                if (rxBuffer[5] == '-'){GPIOD->BSRR |= GPIO_BSRR_BS0 << dir_pin;} else {GPIOD->BSRR |= GPIO_BSRR_BR0 << dir_pin;}
                dir_pin = 10;
                if (rxBuffer[10] == '-'){GPIOD->BSRR |= GPIO_BSRR_BR0 << dir_pin;} else {GPIOD->BSRR |= GPIO_BSRR_BS0 << dir_pin;}
                dir_pin = 11;
                if (rxBuffer[15] == '-'){GPIOD->BSRR |= GPIO_BSRR_BS0 << dir_pin;} else {GPIOD->BSRR |= GPIO_BSRR_BR0 << dir_pin;}

                driver1.req_speed = speed1;
                driver2.req_speed = speed2;
                driver3.req_speed = speed3;
                driver4.req_speed = speed4;

            }
        }
        else {
            rxIndex = 0;
        }
    }
}

int main(){
    configure_clock();
    USART2_Init();

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

    delay(1000000);
    for (int i = 0; i < 4; i++){

        drivers[i].en_pin_port->BSRR |= GPIO_BSRR_BR0 << drivers[i].en_pin_number;
    }

    

    // Set direction pins accordingly
    uint8_t dir_pin = 8;
    GPIOD->MODER |= GPIO_MODER_MODER0_0 << (dir_pin*2);
    GPIOD->OTYPER &= ~(GPIO_OTYPER_OT_0 << dir_pin);
    GPIOD->BSRR |= GPIO_BSRR_BR0 << dir_pin;

    dir_pin = 9;
    GPIOD->MODER |= GPIO_MODER_MODER0_0 << (dir_pin*2);
    GPIOD->OTYPER &= ~(GPIO_OTYPER_OT_0 << dir_pin);
    GPIOD->BSRR |= GPIO_BSRR_BR0 << dir_pin;

    dir_pin = 10;
    GPIOD->MODER |= GPIO_MODER_MODER0_0 << (dir_pin*2);
    GPIOD->OTYPER &= ~(GPIO_OTYPER_OT_0 << dir_pin);
    GPIOD->BSRR |= GPIO_BSRR_BS0 << dir_pin;

    dir_pin = 11;
    GPIOD->MODER |= GPIO_MODER_MODER0_0 << (dir_pin*2);
    GPIOD->OTYPER &= ~(GPIO_OTYPER_OT_0 << dir_pin);
    GPIOD->BSRR |= GPIO_BSRR_BS0 << dir_pin;
    


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

    for (int i = 0; i < 4; i++){drivers[i].req_speed = 0;}

    uint16_t pwm_frequency_3 = driver3.req_speed;
    uint16_t pwm_frequency_4 = driver4.req_speed;
    uint16_t pwm_frequency_1 = driver1.req_speed;
    uint16_t pwm_frequency_2 = driver2.req_speed;
    uint16_t duty_3 = arr_from_freq(driver3.req_speed, TIM4) / 2;
    uint16_t duty_4 = arr_from_freq(driver4.req_speed, TIM3) / 2;
    uint16_t duty_1 = arr_from_freq(driver1.req_speed, TIM1) / 2;
    uint16_t duty_2 = arr_from_freq(driver2.req_speed, TIM9);
    uint8_t state = 0;

    //only for TIM1
    TIM1->BDTR |= TIM_BDTR_MOE;

    uint8_t acc = 8;

    LOG("Online\r\n");

    for(;;){
        if (pwm_frequency_3 <= driver3.req_speed){
            pwm_frequency_3 += acc;  
        }

        if (pwm_frequency_3 >= driver3.req_speed){
            pwm_frequency_3 -= acc;
        }

        if (pwm_frequency_4 <= driver4.req_speed){
            pwm_frequency_4 += acc;  
        }

        if (pwm_frequency_4 >= driver4.req_speed){
            pwm_frequency_4 -= acc;
        }

        if (pwm_frequency_1 <= driver1.req_speed){
            pwm_frequency_1 += acc;  
        }

        if (pwm_frequency_1 >= driver1.req_speed){
            pwm_frequency_1 -= acc;
        }
        
        if (pwm_frequency_2 <= driver2.req_speed){
            pwm_frequency_2 += acc;  
        }

        if (pwm_frequency_2 >= driver1.req_speed){
            pwm_frequency_2 -= acc;
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
