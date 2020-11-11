// =========================================================================
// This library is placed under the MIT License
//
// C port
// Copyright 2020 Maxime Clement
//
// Original Author esp32-MPU-driver C++ libraries
// Copyright 2017-2018 Natanael Josue Rabello
//
// For the license information refer to LICENSE file in root directory
// =========================================================================

// STD and SDK
#include <stdio.h>
#include "sdkconfig.h"

// ESP-IDF
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"

// USER
#include "mpu60x0.h"

static const char *TAG = "MAIN";

mpu_handle_t mpu;

/*******************************************************************************
 * @brief Init the I2C connection with MPU
 */
static void i2c_slave_init(void)
{
    esp_err_t err;
    mpu.addr = 104;
    mpu.bus.num = 1;
    mpu.bus.timeout = 50;
    mpu.init.clk_speed = 400000;
    mpu.init.sda_io_num = GPIO_NUM_32;
    mpu.init.scl_io_num = GPIO_NUM_33;
    mpu.init.sda_pullup_en = false;
    mpu.init.scl_pullup_en = false;
    err = mpu_initialize_peripheral(&mpu);

    // Great! Let's verify the communication
    // (this also check if the connected MPU supports the
    // implementation of chip selected in the component menu)
    err = mpu_test_connection(&mpu);
    while (err)
    {
        ESP_LOGE(TAG, "Failed to connect to the MPU, error=%#X", err);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        err = mpu_test_connection(&mpu);
    }
    ESP_LOGI(TAG, "MPU connection successful!");

    err = mpu_initialize_chip(&mpu);
    ESP_LOGI(TAG, "i2c_slave_init");
}

/*******************************************************************************
 * @brief Main task
 */
static void i2c_test_task(void *arg)
{
    esp_err_t err;


    // Reading sensor data
    printf("Reading sensor data:\n");
    raw_axes_t accel_raw;   // x, y, z axes as int16
    raw_axes_t gyro_raw;    // x, y, z axes as int16
    float_axes_t accel_g;   // accel axes in (g) gravity format
    float_axes_t gyro_dps;  // gyro axes in (DPS) º/s format

    while (true)
    {
        // Read
        err = mpu_acceleration(&mpu, &accel_raw);  // fetch raw data from the registers
        ESP_ERROR_CHECK(err);
        err = mpu_rotation(&mpu, &gyro_raw);       // fetch raw data from the registers
        ESP_ERROR_CHECK(err);
        // printf("accel: %d\t %d\t %d\n", accel_raw.x, accel_raw.y, accel_raw.z);
        // printf("gyro:  %d\t %d\t %d\n", gyro_raw.x, gyro_raw.y, gyro_raw.z);

        mpu_motion(&mpu, &accel_raw, &gyro_raw);  // read both in one shot
        // Convert
        accel_g = mpu_math_accel_gravity(&accel_raw, ACCEL_FS_4G);
        gyro_dps = mpu_math_gyro_deg_per_sec(&gyro_raw, GYRO_FS_500DPS);
        // Debug
        printf("accel: [%+6.2f %+6.2f %+6.2f ] (G) \t", accel_g.x, accel_g.y,
               accel_g.z);
        printf("gyro: [%+7.2f %+7.2f %+7.2f ] (º/s)\n", gyro_dps.x, gyro_dps.y,
               gyro_dps.z);

        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

/*******************************************************************************
 * @brief Program main entry point
 */
void app_main(void)
{
    i2c_slave_init();
    xTaskCreate(i2c_test_task, "i2c_test_task_1", 1024 * 2, (void *)1, 10, NULL);
}
