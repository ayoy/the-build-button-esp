#ifndef BUTTON_H
#define BUTTON_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void start_button_task(uint8_t gpio_pin, void (*press_handler)(void), 
    void (*long_press_handler)(void), 
    TickType_t long_press_interval);

#endif //BUTTON_H