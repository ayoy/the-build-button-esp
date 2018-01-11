#ifndef LED_PWM_H
#define LED_PWM_H

#include "esp_system.h"

uint8_t is_led_pwm_enabled();
void set_led_pwm_enabled(uint8_t enabled);

#endif //LED_PWM_H
