#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

#define BLINK_GPIO 2

static uint8_t s_led_state = 0;

void app_main(void)
{
    ESP_LOGI("Blink", "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    while (1) 
    {
        gpio_set_level(BLINK_GPIO, s_led_state);
        ESP_LOGI("Blink", "Blinking!");
        /* Toggle the LED state */
        s_led_state = !s_led_state;
        vTaskDelay(2000);
    }
}
