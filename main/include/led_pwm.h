#ifndef LED_PWM_H
#define LED_PWM_H

#include "esp_system.h"

void set_led_pwm_gpio(uint8_t gpio_pin);

uint8_t is_led_pwm_enabled();
void set_led_pwm_enabled(uint8_t enabled);

#endif //LED_PWM_H
