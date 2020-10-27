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

#include <stdint.h>
#include <stdbool.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mpu60x0.h"
#include "mpu60x0_i2c.h"
#include "mpu60x0_registers.h"

static const char *TAG = "MPU";

#include "../include/mpu60x0_log.h"


// Static declarations
static esp_err_t set_clock_src(mpu_handle_t * mpu, mpu_clock_src_t clockSrc);
static mpu_clock_src_t get_clock_src(mpu_handle_t * mpu);
static esp_err_t set_gyro_full_scale(mpu_handle_t * mpu, gyro_fs_t fsr);
static gyro_fs_t get_gyro_full_scale(mpu_handle_t * mpu);
static esp_err_t set_accel_full_scale(mpu_handle_t * mpu, accel_fs_t fsr);
static accel_fs_t get_accel_full_scale(mpu_handle_t * mpu);
static esp_err_t set_digital_low_pass_filter(mpu_handle_t * mpu, dlpf_t dlpf);
static dlpf_t get_digital_low_pass_filter(mpu_handle_t * mpu);
static esp_err_t set_sample_rate(mpu_handle_t * mpu, uint16_t rate);
static uint16_t get_sample_rate(mpu_handle_t * mpu);

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Purely for debug
 */
void mpu_print_something(void)
{
    ESP_LOGI(TAG, "Log from mpu6050");
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief initialize the I2C/SPI peripheral
 * @details wrapper arount mpu_i2c_begin/mpu_spi_begin
 */
esp_err_t mpu_initialize_peripheral(mpu_handle_t * mpu)
{
    esp_err_t err = mpu_i2c_begin(mpu);
    return MPU_ERR_CHECK(err);
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Initialize MPU device and set basic configurations.
 * @details
 *  Init configuration:
 *  - Accel FSR: 4G
 *  - Gyro FSR: 500DPS
 *  - Sample rate: 100Hz
 *  - DLPF: 42Hz
 *  - INT pin: disabled
 *  - FIFO: disabled
 *  - Clock source: gyro PLL \n
 *
 * @note
 *  - A soft reset is performed first, which takes 100-200ms.
 *  - When using SPI, the primary I2C Slave module is disabled right away.
 */
esp_err_t mpu_initialize_chip(mpu_handle_t * mpu)
{
    esp_err_t err;

    // reset device (wait a little to clear all registers)
    err = mpu_reset(mpu);
    if(MPU_ERR_CHECK(err)) return err;

    // wake-up the device (power on-reset state is asleep for some models)
    err = mpu_set_sleep(mpu, false);
    if(MPU_ERR_CHECK(err)) return err;

#ifdef CONFIG_MPU_SPI
    err = mpu_i2c_write_bit(mpu, MPU_USER_CTRL, MPU_USERCTRL_I2C_IF_DIS_BIT, 1);
    if(MPU_ERR_CHECK(err)) return err;
#endif

    // disable MPU's I2C slave module when using SPI
    err = set_clock_src(mpu, (uint8_t) CLOCK_PLL);
    if(MPU_ERR_CHECK(err)) return err;

    // set Full Scale range
    err = set_gyro_full_scale(mpu, GYRO_FS_500DPS);
    if(MPU_ERR_CHECK(err)) return err;

    err = set_accel_full_scale(mpu, ACCEL_FS_4G);
    if(MPU_ERR_CHECK(err)) return err;

    // set Digital Low Pass Filter to get smoother data
    err = set_digital_low_pass_filter(mpu, DLPF_42HZ);
    if(MPU_ERR_CHECK(err)) return err;

    // set sample rate to 100Hz
    err = set_sample_rate(mpu, 100);
    if(MPU_ERR_CHECK(err)) return err;

    MPU_LOGI("Initialization complete");
    return err;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Reset internal registers and restore to default start-up state.
 * @note
 *  - This function delays 100ms when using I2C and 200ms when using SPI.
 *  - It does not initialize the MPU again, just call initialize() instead.
 */
esp_err_t mpu_reset(mpu_handle_t * mpu)
{
    esp_err_t err = mpu_i2c_write_bit(mpu, MPU_PWR_MGMT1, MPU_PWR1_DEVICE_RESET_BIT,
                                      1);
    if(MPU_ERR_CHECK(err)) return err;
    vTaskDelay(100 / portTICK_PERIOD_MS);

#ifdef CONFIG_MPU_SPI
    err = reset_signal_path(mpu);
    if(MPU_ERR_CHECK(err)) return err;
#endif

    MPU_LOGI("Reset!");
    return err;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Enable / disable sleep mode
 * @param enable enable value
 */
esp_err_t mpu_set_sleep(mpu_handle_t * mpu, bool enable)
{
    esp_err_t err;
    err =  mpu_i2c_write_bit(mpu, MPU_PWR_MGMT1, MPU_PWR1_SLEEP_BIT,
                             (uint8_t) enable);
    return MPU_ERR_CHECK(err);
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Get current sleep state.
 * @return
 *  - `true`: sleep enabled.
 *  - `false`: sleep disabled.
 */
bool mpu_get_sleep(mpu_handle_t * mpu)
{
    esp_err_t err = mpu_i2c_read_bit(mpu, MPU_PWR_MGMT1, MPU_PWR1_SLEEP_BIT,
                                     mpu->buffer);
    if (MPU_ERR_CHECK(err)) mpu->last_err = err;
    return mpu->buffer[0];
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Returns the value from WHO_AM_I register.
 */
static esp_err_t who_am_i(mpu_handle_t * mpu, uint8_t * buff)
{
    esp_err_t err = mpu_i2c_read_byte(mpu, MPU_WHO_AM_I, buff);
    return MPU_ERR_CHECK(err);
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Test connection with MPU.
 * @details It reads the WHO_AM_IM register and check its value against
 *          the correct chip model.
 * @return
 *  - `ESP_OK`: The MPU is connected and matchs the model.
 *  - `ESP_ERR_NOT_FOUND`: A device is connect, but does not match the
 *                                                chip selected in _menuconfig_.
 *  - May return other communication bus errors. e.g:
 *                                      `ESP_FAIL`, `ESP_ERR_TIMEOUT`.
 */
esp_err_t mpu_test_connection(mpu_handle_t * mpu)
{
    uint8_t wai;
    esp_err_t err = who_am_i(mpu, &wai);
    if(MPU_ERR_CHECK(err)) return err;
    return (wai == 0x68) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Select clock source.
 * @note The gyro PLL is better than internal clock.
 * @param clockSrc clock source
 */
static esp_err_t set_clock_src(mpu_handle_t * mpu, mpu_clock_src_t clockSrc)
{
    esp_err_t err = mpu_i2c_write_bits(mpu, MPU_PWR_MGMT1, MPU_PWR1_CLKSEL_BIT,
                                       MPU_PWR1_CLKSEL_LENGTH, clockSrc);
    return MPU_ERR_CHECK(err);
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Return clock source.
 */
static mpu_clock_src_t get_clock_src(mpu_handle_t * mpu)
{
    esp_err_t err = mpu_i2c_read_bits(mpu, MPU_PWR_MGMT1, MPU_PWR1_CLKSEL_BIT,
                                      MPU_PWR1_CLKSEL_LENGTH, mpu->buffer);
    if (MPU_ERR_CHECK(err)) mpu->last_err = err;
    return (mpu_clock_src_t) mpu->buffer[0];
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Select Gyroscope Full-scale range.
 */
static esp_err_t set_gyro_full_scale(mpu_handle_t * mpu, gyro_fs_t fsr)
{
    esp_err_t err = mpu_i2c_write_bits(mpu, MPU_GYRO_CONFIG, MPU_GCONFIG_FS_SEL_BIT,
                                       MPU_GCONFIG_FS_SEL_LENGTH, fsr);
    return MPU_ERR_CHECK(err);
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Return Gyroscope Full-scale range.
 */
static gyro_fs_t get_gyro_full_scale(mpu_handle_t * mpu)
{
    esp_err_t err = mpu_i2c_read_bits(mpu, MPU_GYRO_CONFIG, MPU_GCONFIG_FS_SEL_BIT,
                                      MPU_GCONFIG_FS_SEL_LENGTH, mpu->buffer);
    if (MPU_ERR_CHECK(err)) mpu->last_err = err;
    return (gyro_fs_t) mpu->buffer[0];
}


////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Select Accelerometer Full-scale range.
 * */
static esp_err_t set_accel_full_scale(mpu_handle_t * mpu, accel_fs_t fsr)
{
    esp_err_t err = mpu_i2c_write_bits(mpu, MPU_ACCEL_CONFIG,
                                       MPU_ACONFIG_FS_SEL_BIT,
                                       MPU_ACONFIG_FS_SEL_LENGTH, fsr);
    return MPU_ERR_CHECK(err);
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Return Accelerometer Full-scale range.
 */
static accel_fs_t get_accel_full_scale(mpu_handle_t * mpu)
{
    esp_err_t err = mpu_i2c_read_bits(mpu, MPU_ACCEL_CONFIG, MPU_ACONFIG_FS_SEL_BIT,
                                      MPU_ACONFIG_FS_SEL_LENGTH, mpu->buffer);
    if (MPU_ERR_CHECK(err)) mpu->last_err = err;
    return (accel_fs_t) mpu->buffer[0];
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Configures Digital Low Pass Filter (DLPF) setting for both the
 *          gyroscope and accelerometer.
 * @param dlpf digital low-pass filter value
 */
static esp_err_t set_digital_low_pass_filter(mpu_handle_t * mpu, dlpf_t dlpf)
{
    esp_err_t err = mpu_i2c_write_bits(mpu, MPU_CONFIG, MPU_CONFIG_DLPF_CFG_BIT,
                                       MPU_CONFIG_DLPF_CFG_LENGTH, dlpf);
    if(MPU_ERR_CHECK(err)) return err;

#ifdef CONFIG_MPU6500
    err = mpu_i2c_write_bits(mpu, MPU_ACCEL_CONFIG2, MPU_ACONFIG2_A_DLPF_CFG_BIT,
                             MPU_ACONFIG2_A_DLPF_CFG_LENGTH, dlpf);
#endif
    return MPU_ERR_CHECK(err);
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Return Digital Low Pass Filter configuration
 */
static dlpf_t get_digital_low_pass_filter(mpu_handle_t * mpu)
{
    esp_err_t err = mpu_i2c_read_bits(mpu, MPU_CONFIG, MPU_CONFIG_DLPF_CFG_BIT,
                                      MPU_CONFIG_DLPF_CFG_LENGTH, mpu->buffer);
    if (MPU_ERR_CHECK(err)) mpu->last_err = err;
    return (dlpf_t) mpu->buffer[0];
}


////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Set sample rate of data output.
 *
 * Sample rate controls sensor data output rate and FIFO sample rate.
 * This is the update rate of sensor register. \n
 * Formula: Sample Rate = Internal Output Rate / (1 + SMPLRT_DIV)
 *
 * @param rate 4Hz ~ 1KHz
 *  - For sample rate 8KHz: set digital low pass filter to DLPF_256HZ_NOLPF.
 *  - For sample rate 32KHZ [MPU6500 / MPU9250]: set fchoice to FCHOICE_0, see setFchoice().
 */
static esp_err_t set_sample_rate(mpu_handle_t * mpu, uint16_t rate)
{
    // Check value range
    if (rate < 4)
    {
        MPU_LOGWMSG(MPU_MSG_INVALID_SAMPLE_RATE, " %d, minimum rate is 4", rate);
        rate = 4;
    }
    else if (rate > 1000)
    {
        MPU_LOGWMSG(MPU_MSG_INVALID_SAMPLE_RATE, " %d, maximum rate is 1000", rate);
        rate = 1000;
    }

#if CONFIG_MPU_LOG_LEVEL >= ESP_LOG_WARN
    // Check dlpf configuration
    dlpf_t dlpf = get_digital_low_pass_filter(mpu);
    esp_err_t err = mpu->last_err;
    if (MPU_ERR_CHECK(err)) return err;
    if (dlpf == 0 || dlpf == 7)
        MPU_LOGWMSG(MPU_MSG_INVALID_STATE,
                    ", sample rate divider is not effective when DLPF is (0 or 7)");
#endif

    uint16_t internal_sample_rate = 1000;
    uint16_t divider              = internal_sample_rate / rate - 1;
    // Check for rate match
    uint16_t finalRate = (internal_sample_rate / (1 + divider));
    if (finalRate != rate)
    {
        MPU_LOGW("Sample rate constrained to %d Hz", finalRate);
    }
    else
    {
        MPU_LOGI("Sample rate set to %d Hz", finalRate);
    }
    // Write divider to register
    err = mpu_i2c_write_byte(mpu, MPU_SMPLRT_DIV, (uint8_t) divider);
    if(MPU_ERR_CHECK(err)) return err;

    return err;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Retrieve sample rate divider and calculate the actual rate.
 */
static uint16_t get_sample_rate(mpu_handle_t * mpu)
{
    uint16_t sample_rate_max_nolpf = 8000;

    dlpf_t dlpf                    = get_digital_low_pass_filter(mpu);
    esp_err_t err = mpu->last_err;
    if (MPU_ERR_CHECK(err)) return err;

    if (dlpf == 0 || dlpf == 7) return sample_rate_max_nolpf;

    uint16_t internal_sample_rate = 1000;
    err = mpu_i2c_read_byte(mpu, MPU_SMPLRT_DIV, mpu->buffer);
    if(MPU_ERR_CHECK(err)) return err;

    uint16_t rate = internal_sample_rate / (1 + mpu->buffer[0]);
    return rate;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Read accelerometer raw data.
 * */
esp_err_t mpu_acceleration(mpu_handle_t * mpu, raw_axes_t* accel)
{
    esp_err_t err = mpu_i2c_read_bytes(mpu, MPU_ACCEL_XOUT_H, 6, mpu->buffer);
    if (MPU_ERR_CHECK(err)) return err;

    uint8_t* buffer = mpu->buffer;
    accel->x = buffer[0] << 8 | buffer[1];
    accel->y = buffer[2] << 8 | buffer[3];
    accel->z = buffer[4] << 8 | buffer[5];

    return err;
}


////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Read gyroscope raw data.
 * */
esp_err_t mpu_rotation(mpu_handle_t * mpu, raw_axes_t* gyro)
{
    esp_err_t err = mpu_i2c_read_bytes(mpu, MPU_GYRO_XOUT_H, 6, mpu->buffer);
    if (MPU_ERR_CHECK(err)) return err;

    uint8_t* buffer = mpu->buffer;
    gyro->x = buffer[0] << 8 | buffer[1];
    gyro->y = buffer[2] << 8 | buffer[3];
    gyro->z = buffer[4] << 8 | buffer[5];

    return err;
}