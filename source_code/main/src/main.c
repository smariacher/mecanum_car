#include "main.h"
#include "clock_config.h"

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

    init_direction_pins(&drivers);
    LOG("Direction pins initialized!\r\n");

    init_timers();
    LOG("Timer initialized!\r\n");
    
    // Reset speed
    for (int i = 0; i < 4; i++){
        drivers[i]->req_speed = 0;
        drivers[i]->cur_speed = 0;
    }
    LOG("Speed set to zero!\r\n");

    // INIT PHASE DONE
    LOG("Initilization phase done!\r\n");

    // MAIN LOOP
    for(;;){
        update_motors(&drivers, 8);
        
        // TODO: Integrate Timer-based clock speed
        delay(500);
    }
    return 0;
}