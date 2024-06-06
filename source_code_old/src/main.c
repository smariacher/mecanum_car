#include "main.h"
#include "mci_clock.h"

#define C

#define DEBUG

#ifdef DEBUG
  #define LOG( msg... ) printf( msg );
#else
  #define LOG( msg... ) ;
#endif

#define F_CLK 24000000
#define BAUDRATE 9600

int _write( int handle, char* data, int size ) {
    int count = size;
    while( count-- ) {
        while( !( USART2->ISR & USART_ISR_TXE ) ) {};
        USART2->TDR = *data++;
    }
    return size;
}

/**
 * @brief Delays the program execution for a specified amount of time.
 * @param time The amount of time to delay in number of cycles.
 * @return 0 when the delay is completed.
 */
int delay(uint32_t time){
    for(uint32_t i = 0; i < time; i++ ){
        asm("nop"); // No operation, used for delaying
    }
    return 0;
}

/**
 * @brief Returns the right ARR value for a certain frequency
 * 
 * @param freq 
 * @return ARR value
 */
uint16_t arr_from_freq(uint16_t freq){
    return F_CLK/(freq*(TIM2->PSC+1)) - 1;
}

void swuart_calcCRC(unsigned char* datagram, unsigned char datagramLength)
{
    int i,j;
    unsigned char* crc = datagram + (datagramLength-1); // CRC located in last byte of message
    unsigned char currentByte;
    *crc = 0;
    for (i=0; i<(datagramLength-1); i++) { // Execute for all bytes of a message
        currentByte = datagram[i]; // Retrieve a byte to be sent from Array
        for (j=0; j<8; j++) {
            if ((*crc >> 7) ^ (currentByte&0x01)){ // update CRC based result of XOR operation
                *crc = (*crc << 1) ^ 0x07;
            }
            else{
                *crc = (*crc << 1);
            }
            currentByte = currentByte >> 1;
        } // for CRC bit
    } // for message byte
}

void write_to_tmc(unsigned char* msg_buffer){
    swuart_calcCRC(msg_buffer, 8);
    for (int b = 0; b<8; b++){
        while( !( USART2->ISR & USART_ISR_TXE ) ) {};
        USART2->TDR = msg_buffer[b];    
    }
}

void read_from_tmc(unsigned char* msg_buffer){
    swuart_calcCRC(msg_buffer, 4);
    for (int b = 0; b<4; b++){
        while( !( USART2->ISR & USART_ISR_TXE ) ) {};
        USART2->TDR = msg_buffer[b];    
    }
}

void tmc_init(){
    unsigned char msg_buffer[] = {0x55, 0x00, 0x80, 0x20, 0x00, 0x00, 0x00, 0x00};
    write_to_tmc(msg_buffer);
}

void tmc_inverse_dir(){
    unsigned char msg_buffer[] = {0x55, 0x00, 0x80, 0x04, 0x00, 0x00, 0x00, 0x00};
    write_to_tmc(msg_buffer);
}

void read_tmc_init(){
    unsigned char msg_buffer[] = {0x05, 0x00, 0x00, 0x00};
    read_from_tmc(msg_buffer);
}

/**
 * @brief Main function
 * @return 0
 */
int main(void){

    EPL_SystemClock_Config();

    // Enable the GPIOA and TIM2 peripherals
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Enable peripheral  USART2 clock
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN; 

    // Configure PA10 for EN pin and set high immediately
    GPIOA->MODER |= 0b01 << (2*10);
    GPIOA->BSRR |= GPIO_BSRR_BS_10;

    // Configure PA2 as USART2_TX using alternate function 1
    GPIOA->MODER |= GPIO_MODER_MODER2_1;
    GPIOA->AFR[0] |= 0b0001 << (4*2);

    // Configure PA3 as USART2_RX using alternate function 1
    GPIOA->MODER |= GPIO_MODER_MODER3_1;
    GPIOA->AFR[0] |= 0b0001 << (4*3);

    // Configure the UART Baude rate Register 
    USART2->BRR = (APB_FREQ / BAUDRATE);
    
    // Enable the UART using the CR1 register
    USART2->CR1 |= ( USART_CR1_RE | USART_CR1_TE | USART_CR1_UE );


    // Set the mode of the GPIOA pin 5 to alternate function mode 2
    GPIOA->MODER |= GPIO_MODER_MODER5_1;
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR5;
    GPIOA->AFR[0] |= 0b0010 << 20;
    
    // Disable the timer before setting the prescaler and overflow values
    TIM2->CR1 &= ~TIM_CR1_CEN;
    
    // Set the prescaler and overflow values
    TIM2->PSC = 3; // 10 seems to be optimal for now
    TIM2->ARR = 64000;
    TIM2->CCR1 = 0;

    // Set the PWM mode 1
    TIM2->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
    TIM2->CCMR1 |= TIM_CCMR1_OC1PE;
    // Set the output polarity to active high
    TIM2->CCER |= TIM_CCER_CC1E;

    // Enable the main output
    // TIM2->BDTR |= TIM_BDTR_MOE; // does not exist on TIM2
    
    // Enable auto reload preload -> configures a buffer register for ARR and CCR1 so that those values do not get changed while the timer is still counting
    TIM2->CR1 |= TIM_CR1_ARPE;

    // Enbable the timer
    // Set the center-aligned mode 
    TIM2->CR1 |= TIM_CR1_CMS_0 | TIM_CR1_CEN;

    // One rotation per minute because of step/8 microstepping
    uint16_t lower_limit = 100; // Around 100 is the minimum of steps - to be lower needs a different PSC value
    uint16_t upper_limit = 400; // Around 8000 seems to be current maximum
    uint16_t pwm_frequency = lower_limit;
    uint16_t duty = arr_from_freq(lower_limit)/2;
    uint8_t state = 0;

    uint32_t counter = 0;
    

    tmc_init();
    delay(100000);
    // tmc_inverse_dir();

    read_tmc_init();

    // Wait for motor to ramp down
    delay(10000000);
    GPIOA->BSRR |= GPIO_BSRR_BR_10;

    for(;;){
        if (pwm_frequency <= upper_limit && state == 0){
            pwm_frequency += 1;
            // LOG("ARR: %d,%d\r\n", TIM2->ARR, arr_from_freq(pwm_frequency));
            // LOG("Freq: %d\r\n", pwm_frequency);   
        } else {state = 0;}

        if (pwm_frequency >= lower_limit && state == 1){
            pwm_frequency -= 1;
        } else {state = 0;}
        
        TIM2->ARR = arr_from_freq(pwm_frequency);
        duty = arr_from_freq(pwm_frequency)/2;
        TIM2->CCR1 = duty;
        // LOG("ARR: %d, duty: %d\r\n", TIM2->ARR, duty);
        
        // if (counter >= 1000){
        //     LOG("%02X%02X%02X%02X\r\n", 0xBF, 0x60, 0x00, 0x55);
        //     counter = 0;
        // }
        counter += 1;

        delay(2000);
    }
}