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

// Function to send debug info via USART2
#define LOG( msg... ) printf( msg );

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

/// @brief Very easy function to delay code execution
/// @param time amount of nop instructions being executed
/// @return 0 after time ran out
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

/// @brief Calculates the value to be sent into the auto-reload register from a given frequency
/// @param freq 
/// @param timer The prescale registers timer
/// @return 
uint16_t arr_from_freq(uint16_t freq, TIM_TypeDef *timer){
    return (uint16_t)(F_CLK/(freq*(timer->PSC+1)) - 1);
}

/// @brief Converts a char string to uint16
/// @param str 
/// @return 
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

/// @brief Converts a char string to int16
/// @param str 
/// @return 
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

/// @brief Function to set alternate functions for GPIO ports
/// @param port GPIOx port
/// @param pin 
/// @param alt_fn Can be looked up in stm32f401 manual
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

/// @brief Initializes the enable pin for a given gpio port and pin number
/// @param en_port 
/// @param en_pin 
void init_enable_pin(GPIO_TypeDef en_port, uint8_t en_pin){
    GPIOD->MODER |= GPIO_MODER_MODER0_0 << (2 * en_pin);
    GPIOD->OTYPER &= ~(GPIO_OTYPER_OT_0 << en_pin);
    GPIOD->BSRR |= GPIO_BSRR_BS0 << en_pin;
}

/// @brief Initializes USART2
/// @param  
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

/// @brief Function that gets called upon interrupt from USART2
/// @param  
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
                if (rxIndex == 21){
                    // Motor controll code
                    // TODO: error handling
                    char string1[6];
                    char string2[6];
                    char string3[6];
                    char string4[6];
                    
                    string1[6] = '\0';
                    string2[6] = '\0';
                    string3[6] = '\0';
                    string4[6] = '\0';

                    memcpy(string1, rxBuffer, 5 * sizeof(char));
                    memcpy(string2, rxBuffer + 5, 5 * sizeof(char));
                    memcpy(string3, rxBuffer + 10, 5 * sizeof(char));
                    memcpy(string4, rxBuffer + 15, 5 * sizeof(char));

                    int16_t speed1 = string_to_int16(string1);
                    int16_t speed2 = string_to_int16(string2);
                    int16_t speed3 = string_to_int16(string3);
                    int16_t speed4 = string_to_int16(string4);

                    LOG("Speed1=%d, Speed2=%d, Speed3=%d, Speed4=%d\r\n", speed1, speed2, speed3, speed4);

                    driver1.req_speed = speed1;
                    driver2.req_speed = speed2;
                    driver3.req_speed = speed3;
                    driver4.req_speed = speed4;
                }
                if (rxBuffer[0] == 'e'){
                    
                }
                rxIndex = 0;

            }
        }
        else {
            rxIndex = 0;
        }
    }
}

/// @brief Updates all motors speed and direction
/// @param acceleration how much the motor accelerates to the required speed
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

    TIM1->ARR = arr_from_freq(abs(driver1.cur_speed), TIM1);
    TIM1->CCR1 = arr_from_freq(abs(driver1.cur_speed), TIM1) / 2;

    // TODO: Reevaluate this calculation
    TIM9->ARR = arr_from_freq(abs(driver2.cur_speed), TIM9) * 2;
    TIM9->CCR1 = arr_from_freq(abs(driver2.cur_speed), TIM9);
    

    TIM4->ARR = arr_from_freq(abs(driver3.cur_speed), TIM4);
    TIM4->CCR1 = arr_from_freq(abs(driver3.cur_speed), TIM4) / 2;

    TIM3->ARR = arr_from_freq(abs(driver4.cur_speed), TIM3);
    TIM3->CCR1 = arr_from_freq(abs(driver4.cur_speed), TIM3) / 2;
}

/// @brief Initializes all direction pins
void init_direction_pins(){
    // TODO: use tmc_controller
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
}

/// @brief Disables all motors - also callable from Interrupt
void disable_all_motors(){
    tmc_controller *drivers[4] = {&driver1, &driver2, &driver3, &driver4};
    for (int i = 0; i < 4; i++){
        drivers[i]->en_pin_port->BSRR |= GPIO_BSRR_BS0 << drivers[i]->en_pin_number;
    }
}

void init_enable_pins(tmc_controller *drivers[]){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    for (int i = 0; i < 4; i++){
        init_enable_pin(*drivers[i]->en_pin_port, drivers[i]->en_pin_number);
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

    TIM3->CR1 |= TIM_CR1_ARPE;

    TIM1->CR1 |= TIM_CR1_ARPE;
    TIM1->CR2 = 0;

    TIM9->CR1 |= TIM_CR1_ARPE;

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

    TIM4->CCMR1 |= (TIM_CCMR1_OC1PE | (0b110 << TIM_CCMR1_OC1M_Pos));

    TIM3->CCMR1 |= (TIM_CCMR1_OC1PE | (0b110 << TIM_CCMR1_OC1M_Pos));

    TIM1->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
    TIM1->CCMR1 |= (TIM_CCMR1_OC1PE | (0b110 << TIM_CCMR1_OC1M_Pos));

    TIM9->CCMR1 |= (TIM_CCMR1_OC1PE | (0b110 << TIM_CCMR1_OC1M_Pos));

    // Enable Capture Compare for channel 1
    TIM4->CCER |= TIM_CCER_CC1E;

    TIM3->CCER |= TIM_CCER_CC1E;

    TIM1->CCER |= TIM_CCER_CC1E;

    TIM9->CCER |= TIM_CCER_CC1E;

    // Enable the timer and set to center-aligned mode
    TIM4->CR1 |= TIM_CR1_CMS_0 | TIM_CR1_CEN;

    TIM3->CR1 |= TIM_CR1_CMS_0 | TIM_CR1_CEN;

    TIM1->CR1 |= TIM_CR1_CMS_0 | TIM_CR1_CEN;

    TIM9->CR1 |= TIM_CR1_CMS_0 | TIM_CR1_CEN;
}

int main(){
    configure_clock();
    USART2_Init();
    LOG("USART2 initialized!\r\n");

    tmc_controller *drivers[4] = {&driver1, &driver2, &driver3, &driver4};

    // Define all driver values
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

    init_enable_pins(&drivers);
    LOG("Enable pins initialized!\r\n");
    init_direction_pins();
    LOG("Direction pins initialized!\r\n");
    init_timers();
    LOG("Timer initialized!\r\n");
    
    for (int i = 0; i < 4; i++){
        drivers[i]->req_speed = 0;
        drivers[i]->cur_speed = 0;
    }

    LOG("Speed set to zero!\r\n");

    //only for TIM1
    TIM1->BDTR |= TIM_BDTR_MOE;

    LOG("Initilization phase done!\r\n");
    for(;;){
        update_motors(&drivers, 8);
        
        // TODO: Integrate Timer-based clock speed
        delay(500);
    }
    return 0;
}
