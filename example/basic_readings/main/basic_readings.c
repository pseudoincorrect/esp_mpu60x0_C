// STD and SDK
#include <stdio.h>
#include "sdkconfig.h"
// ESP-IDF
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
// USER
#include "mpu60x0.h"

static const char *TAG = "MAIN";

static void i2c_slave_init(void)
{
    ESP_LOGI(TAG, "i2c_slave_init");
}

static void i2c_test_task(void *arg)
{
    int counter = 0;
    while(1)
    {
        mpu_print_something();
        ESP_LOGI(TAG, "counter : %d", counter++);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }

}

void app_main(void)
{
    i2c_slave_init();
    xTaskCreate(i2c_test_task, "i2c_test_task_1", 1024 * 2, (void *)1, 10, NULL);
}
