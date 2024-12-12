#ifndef REMOTE_H
#define REMOTE_H

#include "main.h"

#define BAUDRATE 9600
#define APB_FREQ 16000000
#define BUFFER_SIZE 128 // Size for UART rx buffer

// Function to send debug info via USART2
#define LOG( msg... ) printf( msg );

volatile char rxBuffer[BUFFER_SIZE];
extern volatile uint32_t rxIndex;

extern uint8_t light_state_front;
extern uint8_t light_state_back;

void USART2_IQRHandler(void);

/// @brief Initializes USART2
void USART2_Init(void);

int _write(int handle, char* data, int size);



#endif