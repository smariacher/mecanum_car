#include "main.h"
#include "clock_config.h"

#define BAUDRATE 9600
#define APB_FREQ 16000000

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

int main(){
    // uart2 --> PA2:TX, PA3:RX
    configure_clock();

    // Enable peripheral GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    // Enable peripheral USART2 clock
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // Configure PA2 as USART2_TX using alternate function 7
    GPIOA->MODER |= GPIO_MODER_MODER2_1;
    GPIOA->AFR[0] |= 0b0111 << (4*2);

    // Configure PA3 as USART2_RX using alternate function 7
    GPIOA->MODER |= GPIO_MODER_MODER3_1;
    GPIOA->AFR[0] |= 0b0111 << (4*3);

    // Clear CR1 register
    USART2->CR1 = 0;
    // Enable transmitter and receiver
    USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE);

    // Set baudrate 
    USART2->BRR = (APB_FREQ / BAUDRATE);

    // Enable USART2
    USART2->CR1 |= USART_CR1_UE;
    int cnt = 0;

    uint8_t rxb = '\0';

    for (;;){
        while (! (USART2->SR & USART_SR_RXNE)) {
            ;
        }
        rxb = USART2->DR;
        
        LOG("Hello World! %d\n", rxb);
        delay(100000);
        cnt++;
    }

    return 0;
}
