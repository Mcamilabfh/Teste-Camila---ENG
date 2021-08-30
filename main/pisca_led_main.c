
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define GPIO_LED 2
#define GPIO_OUTPUT_PIN_SEL (1ULL<<GPIO_LED) 

void app_main(void)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    while (1) {
         vTaskDelay(1000 / portTICK_RATE_MS);
         gpio_set_level(GPIO_LED, 1);
         vTaskDelay(1000 / portTICK_RATE_MS);
         gpio_set_level(GPIO_LED, 0);
    }
}
