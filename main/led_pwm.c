#include "led_pwm.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"

static const char * kLEDPWMTag = "LED_PWM";

static const uint8_t kLEDGPIO = 26;
static const uint16_t kLEDFadeTime = 1500;
static const uint16_t kLEDDutyMax = 8191;
static const uint16_t kLEDDutyMin = 0;

static uint8_t is_stop_requested = 0;
TaskHandle_t pwm_task_handle = NULL;

void tear_down_pwm_task(ledc_channel_config_t *config)
{
    ledc_set_duty(config->speed_mode, config->channel, 0);
    ledc_update_duty(config->speed_mode, config->channel);
    ledc_fade_func_uninstall();
    pwm_task_handle = NULL;
    is_stop_requested = 0;
    vTaskDelete(NULL);
}

void pwm_task(void *pvParameter)
{
    ESP_LOGI(kLEDPWMTag, "Starting LED PWM task");

    gpio_pad_select_gpio(kLEDGPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(kLEDGPIO, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(kLEDGPIO, GPIO_PULLDOWN_ONLY);

    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = 5000,                      // frequency of PWM signal
        .speed_mode = LEDC_HIGH_SPEED_MODE,    // timer mode
        .timer_num = LEDC_TIMER_0             // timer index
    };

    ledc_channel_config_t ledc_config = {
        .gpio_num = kLEDGPIO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty = 200,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
    };

    ledc_timer_config(&ledc_timer);
    ledc_channel_config(&ledc_config);
    ledc_fade_func_install(0);

    ledc_set_duty(ledc_config.speed_mode, ledc_config.channel, kLEDDutyMax);
    ledc_update_duty(ledc_config.speed_mode, ledc_config.channel);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    while (1) {
        ESP_LOGI(kLEDPWMTag, "1. LEDC fade down to duty = %d", kLEDDutyMin);
        ledc_set_fade_with_time(ledc_config.speed_mode, ledc_config.channel, kLEDDutyMin, kLEDFadeTime);
        ledc_fade_start(ledc_config.speed_mode, ledc_config.channel, LEDC_FADE_WAIT_DONE);

        if (is_stop_requested) {
            tear_down_pwm_task(&ledc_config);
        }

        ESP_LOGI(kLEDPWMTag, "2. LEDC fade up to duty = %d", kLEDDutyMax);
        ledc_set_fade_with_time(ledc_config.speed_mode, ledc_config.channel, kLEDDutyMax, kLEDFadeTime);
        ledc_fade_start(ledc_config.speed_mode, ledc_config.channel, LEDC_FADE_WAIT_DONE);
    }
}

void start_led_pwm_task() 
{
    xTaskCreate(&pwm_task, "LED_PWM_task", 3072, NULL, 5, &pwm_task_handle);
}

uint8_t is_led_pwm_enabled()
{
    return pwm_task_handle != NULL;
}

void set_led_pwm_enabled(uint8_t enabled)
{
    uint8_t is_enabled = is_led_pwm_enabled();

    ESP_LOGE(kLEDPWMTag, "%s %d (%d)", __func__, enabled, is_enabled);
    if (enabled != is_enabled) {
        ESP_LOGE(kLEDPWMTag, "%s %d", __func__, enabled);
        if (enabled) {
            start_led_pwm_task();
        } else {
            is_stop_requested = 1;
        }
    }
}
