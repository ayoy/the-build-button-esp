#include "button.h"
#include "esp_log.h"
#include "driver/timer.h"

static const char * kButtonTag = "BUTTON";

static uint8_t button_pin = 0;

static void (*button_press_handler)(void) = NULL;
static void (*button_long_press_handler)(void) = NULL;
static TickType_t button_long_press_interval = 0;
static void (*button_touch_down_handler)(void) = NULL;

#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER) * 1000  // convert counter value to milliseconds
static bool is_timer_running = 0;

/*
 * A simple helper function to print the raw timer counter value
 * and the counter value converted to seconds
 */
static void inline print_timer_counter(uint64_t counter_value)
{
    printf("Counter: 0x%08x%08x\n", (uint32_t) (counter_value >> 32),
                                    (uint32_t) (counter_value));
    printf("Time   : %.8fms\n", (double) counter_value / TIMER_SCALE);
}

void start_timer()
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_DIS;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = 0;
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);

    timer_start(TIMER_GROUP_0, TIMER_0);
    is_timer_running = 1;
}

void stop_timer()
{
    is_timer_running = 0;
    timer_pause(TIMER_GROUP_0, TIMER_0);
}

void button_handler_task(void *pvParameter)
{
    ESP_LOGE(kButtonTag, "Starting button handler task");

    gpio_pad_select_gpio(button_pin);
    gpio_set_direction(button_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(button_pin, GPIO_PULLUP_ONLY);

    int level = gpio_get_level(button_pin);
    bool triggered_at_startup = 0;
    if (level == 0) {
        ESP_LOGE(kButtonTag, "Pin %i low at handler task startup", button_pin);
        triggered_at_startup = 1;
    }
    while(1) {
        if (gpio_get_level(button_pin) != level || triggered_at_startup) {
            triggered_at_startup = 0;
            level = gpio_get_level(button_pin);
            if (level == 0) {
                ESP_LOGI(kButtonTag, "Pin %i low!", button_pin);
                start_timer();
                if (button_touch_down_handler) {
                    button_touch_down_handler();
                }
            } else {
                if (is_timer_running) {
                    ESP_LOGI(kButtonTag, "Pin %i high again!", button_pin);
                    stop_timer();
                    double timer_time_sec;
                    timer_get_counter_time_sec(TIMER_GROUP_0, TIMER_0, &timer_time_sec);

                    if (timer_time_sec * 1000/portTICK_PERIOD_MS > button_long_press_interval) {
                        ESP_LOGI(kButtonTag, "Button long press! (%.3fs)", timer_time_sec);
                        if (button_long_press_handler) {
                            button_long_press_handler();
                        }
                    } else {
                        ESP_LOGI(kButtonTag, "Button press! (%.3fs)", timer_time_sec);
                        if (button_press_handler) {
                            button_press_handler();
                        }
                    }
                }
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void start_button_task(uint8_t gpio_pin, void (*press_handler)(void), 
    void (*long_press_handler)(void), 
    TickType_t long_press_interval,
    void (*touch_down_handler)(void))
{
    button_pin = gpio_pin;
    button_press_handler = press_handler;
    button_long_press_handler = long_press_handler;
    button_long_press_interval = long_press_interval;
    button_touch_down_handler = touch_down_handler;
    xTaskCreate(&button_handler_task, "button_handler_task", 3072, NULL, 5, NULL);
}

