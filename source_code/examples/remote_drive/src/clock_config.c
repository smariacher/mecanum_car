#include "clock_config.h"


/**
 * Configures the stm32 to use the internal 16Mhz clock
 */
void configure_clock(){
    // Reset the flash 'Access Control Register'
    // set wait-state to 1 
    // enable prefetch buffer
    FLASH->ACR &= ~(FLASH_ACR_LATENCY_Msk | FLASH_ACR_PRFTEN_Msk);
    FLASH->ACR |= (FLASH_ACR_LATENCY | FLASH_ACR_PRFTEN);
    
    // activate the internal 16 MHz clock
    RCC->CR |= RCC_CR_HSION;

    // wait for the clock to become stable
    while (!(RCC->CR & RCC_CR_HSIRDY)){}

    // AHB and APB clocks are not configured but should be just not divided so run on 16Mhz as well

}

