#include "remote.h"

volatile uint32_t rxIndex = 0;

uint8_t light_state_front = 0;
uint8_t light_state_back = 0;

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

                // if motor controll string is sent -> process it
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

                    //LOG("Speed1=%d, Speed2=%d, Speed3=%d, Speed4=%d\r\n", speed1, speed2, speed3, speed4);

                    driver1.req_speed = speed1;
                    driver2.req_speed = speed2;
                    driver3.req_speed = speed3;
                    driver4.req_speed = speed4;
                }

                // if emergency stop is sent
                if (rxBuffer[0] == 'e'){
                    disable_all_motors();
                    driver1.cur_speed = 0;
                    driver2.cur_speed = 0;
                    driver3.cur_speed = 0;
                    driver4.cur_speed = 0;
                    //LOG("Emergency stop called! Waiting for reset!\r\n")
                }

                // if light command sent
                if (rxBuffer[0] == 'f'){
                    if (light_state_front){
                        TIM2->CCR3 = 0;
                        light_state_front = 0;
                    } else {
                        TIM2->CCR3 = TIM2->ARR;
                        light_state_front = 1;
                    }
                }

                if (rxBuffer[0] == 'b'){
                    if (light_state_back){
                        TIM2->CCR2 = 0;
                        light_state_back = 0;
                    } else {
                        TIM2->CCR2 = TIM2->ARR;
                        light_state_back = 1;
                    }
                }
                rxIndex = 0;

            }
        }
        else {
            rxIndex = 0;
        }
    }
}