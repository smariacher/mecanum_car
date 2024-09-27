#include "main.h"
#include "clock_config.h"
#include "stm32f401xe.h"

#define BAUDRATE 9600
#define APB_FREQ 16000000
#define BUFFER_SIZE 128

volatile char rxBuffer[BUFFER_SIZE];
volatile uint32_t rxIndex = 0;

void USART2_IRQHandler(void);
void USART2_Init(void);

#define LOG( msg... ) printf( msg );

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
                rxIndex = 0;
                
                // Process the received command here
                LOG("Received: %s\r\n", rxBuffer);
            }
        }
        else {
            rxIndex = 0;
        }
    }
}

int main(void){
    // uart2 --> PA2:TX, PA3:RX
    configure_clock();
    USART2_Init();
    LOG("ONLINE\r\n");
    
    while (1){
        // Main program loop
    }
}