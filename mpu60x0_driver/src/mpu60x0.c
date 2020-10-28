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

static const char *TAG = "MPU"; // needs to be included before mpu60x0_log.h
#include "../include/mpu60x0_log.h"


// Static declarations
static esp_err_t mpu_set_clock_src(mpu_handle_t * mpu,
                                   mpu_clock_src_t clockSrc);
static mpu_clock_src_t mpu_get_clock_src(mpu_handle_t * mpu);
static esp_err_t mpu_set_gyro_full_scale(mpu_handle_t * mpu, gyro_fs_t fsr);
static gyro_fs_t mpu_get_gyro_full_scale(mpu_handle_t * mpu);
static esp_err_t mpu_set_accel_full_scale(mpu_handle_t * mpu, accel_fs_t fsr);
static accel_fs_t mpu_get_accel_full_scale(mpu_handle_t * mpu);
static esp_err_t mpu_set_dlp_filter(mpu_handle_t * mpu, dlpf_t dlpf);
static dlpf_t mpu_get_dlp_filter(mpu_handle_t * mpu);
static esp_err_t mpu_set_sample_rate(mpu_handle_t * mpu, uint16_t rate);
static uint16_t mpu_get_sample_rate(mpu_handle_t * mpu);
static esp_err_t mpu_set_aux_i2c_enabled(mpu_handle_t * mpu, bool enable);
static bool mpu_get_aux_i2c_enabled(mpu_handle_t * mpu);

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Purely for debug
 */
void mpu_print_something(void)
{
    ESP_LOGI(TAG, "Log from mpu6050");
}

// #### ##    ## #### ########
//  ##  ###   ##  ##     ##
//  ##  ####  ##  ##     ##
//  ##  ## ## ##  ##     ##
//  ##  ##  ####  ##     ##
//  ##  ##   ###  ##     ##
// #### ##    ## ####    ##

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
    err = mpu_set_clock_src(mpu, (uint8_t) CLOCK_PLL);
    if(MPU_ERR_CHECK(err)) return err;

    // set Full Scale range
    err = mpu_set_gyro_full_scale(mpu, GYRO_FS_500DPS);
    if(MPU_ERR_CHECK(err)) return err;

    err = mpu_set_accel_full_scale(mpu, ACCEL_FS_4G);
    if(MPU_ERR_CHECK(err)) return err;

    // set Digital Low Pass Filter to get smoother data
    err = mpu_set_dlp_filter(mpu, DLPF_42HZ);
    if(MPU_ERR_CHECK(err)) return err;

    // set sample rate to 100Hz
    err = mpu_set_sample_rate(mpu, 100);
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

//  ######  ##       ######## ######## ########
// ##    ## ##       ##       ##       ##     ##
// ##       ##       ##       ##       ##     ##
//  ######  ##       ######   ######   ########
//       ## ##       ##       ##       ##
// ##    ## ##       ##       ##       ##
//  ######  ######## ######## ######## ##

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
    uint8_t* buffer = mpu->buffer;
    esp_err_t err = mpu_i2c_read_bit(mpu, MPU_PWR_MGMT1, MPU_PWR1_SLEEP_BIT,
                                     buffer);
    if (MPU_ERR_CHECK(err)) mpu->last_err = err;
    return buffer[0];
}

// ######## ########  ######  ########
//    ##    ##       ##    ##    ##
//    ##    ##       ##          ##
//    ##    ######    ######     ##
//    ##    ##             ##    ##
//    ##    ##       ##    ##    ##
//    ##    ########  ######     ##

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Returns the value from WHO_AM_I register.
 */
static esp_err_t mpu_who_am_i(mpu_handle_t * mpu, uint8_t * buff)
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
    esp_err_t err = mpu_who_am_i(mpu, &wai);
    if(MPU_ERR_CHECK(err)) return err;
    return (wai == 0x68) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

//  ######  ##        #######   ######  ##    ##
// ##    ## ##       ##     ## ##    ## ##   ##
// ##       ##       ##     ## ##       ##  ##
// ##       ##       ##     ## ##       #####
// ##       ##       ##     ## ##       ##  ##
// ##    ## ##       ##     ## ##    ## ##   ##
//  ######  ########  #######   ######  ##    ##

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Select clock source.
 * @note The gyro PLL is better than internal clock.
 * @param clockSrc clock source
 */
static esp_err_t mpu_set_clock_src(mpu_handle_t * mpu, mpu_clock_src_t clockSrc)
{
    esp_err_t err = mpu_i2c_write_bits(mpu, MPU_PWR_MGMT1, MPU_PWR1_CLKSEL_BIT,
                                       MPU_PWR1_CLKSEL_LENGTH, clockSrc);
    return MPU_ERR_CHECK(err);
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Return clock source.
 */
static mpu_clock_src_t mpu_get_clock_src(mpu_handle_t * mpu)
{
    uint8_t* buffer = mpu->buffer;
    esp_err_t err = mpu_i2c_read_bits(mpu, MPU_PWR_MGMT1, MPU_PWR1_CLKSEL_BIT,
                                      MPU_PWR1_CLKSEL_LENGTH, buffer);
    if (MPU_ERR_CHECK(err)) mpu->last_err = err;
    return (mpu_clock_src_t) buffer[0];
}

//  ######   ######     ###    ##       ########
// ##    ## ##    ##   ## ##   ##       ##
// ##       ##        ##   ##  ##       ##
//  ######  ##       ##     ## ##       ######
//       ## ##       ######### ##       ##
// ##    ## ##    ## ##     ## ##       ##
//  ######   ######  ##     ## ######## ########

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Select Gyroscope Full-scale range.
 */
static esp_err_t mpu_set_gyro_full_scale(mpu_handle_t * mpu, gyro_fs_t fsr)
{
    esp_err_t err = mpu_i2c_write_bits(mpu, MPU_GYRO_CONFIG, MPU_GCONFIG_FS_SEL_BIT,
                                       MPU_GCONFIG_FS_SEL_LENGTH, fsr);
    return MPU_ERR_CHECK(err);
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Return Gyroscope Full-scale range.
 */
static gyro_fs_t mpu_get_gyro_full_scale(mpu_handle_t * mpu)
{
    uint8_t* buffer = mpu->buffer;
    esp_err_t err = mpu_i2c_read_bits(mpu, MPU_GYRO_CONFIG, MPU_GCONFIG_FS_SEL_BIT,
                                      MPU_GCONFIG_FS_SEL_LENGTH, buffer);
    if (MPU_ERR_CHECK(err)) mpu->last_err = err;
    return (gyro_fs_t) buffer[0];
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Select Accelerometer Full-scale range.
 * */
static esp_err_t mpu_set_accel_full_scale(mpu_handle_t * mpu, accel_fs_t fsr)
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
static accel_fs_t mpu_get_accel_full_scale(mpu_handle_t * mpu)
{
    uint8_t* buffer = mpu->buffer;
    esp_err_t err = mpu_i2c_read_bits(mpu, MPU_ACCEL_CONFIG, MPU_ACONFIG_FS_SEL_BIT,
                                      MPU_ACONFIG_FS_SEL_LENGTH, buffer);
    if (MPU_ERR_CHECK(err)) mpu->last_err = err;
    return (accel_fs_t) buffer[0];
}

// ########  ##       ########  ########
// ##     ## ##       ##     ## ##
// ##     ## ##       ##     ## ##
// ##     ## ##       ########  ######
// ##     ## ##       ##        ##
// ##     ## ##       ##        ##
// ########  ######## ##        ##

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Configures Digital Low Pass Filter (DLPF) setting for both the
 *          gyroscope and accelerometer.
 * @param dlpf digital low-pass filter value
 */
static esp_err_t mpu_set_dlp_filter(mpu_handle_t * mpu,
                                    dlpf_t dlpf)
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
static dlpf_t mpu_get_dlp_filter(mpu_handle_t * mpu)
{
    uint8_t* buffer = mpu->buffer;
    esp_err_t err = mpu_i2c_read_bits(mpu, MPU_CONFIG, MPU_CONFIG_DLPF_CFG_BIT,
                                      MPU_CONFIG_DLPF_CFG_LENGTH, buffer);
    if (MPU_ERR_CHECK(err)) mpu->last_err = err;
    return (dlpf_t) buffer[0];
}

//  ######     ###    ##     ## ########  ##       ########       ########
// ##    ##   ## ##   ###   ### ##     ## ##       ##             ##     ##
// ##        ##   ##  #### #### ##     ## ##       ##             ##     ##
//  ######  ##     ## ## ### ## ########  ##       ######         ########
//       ## ######### ##     ## ##        ##       ##             ##   ##
// ##    ## ##     ## ##     ## ##        ##       ##             ##    ##  ###
//  ######  ##     ## ##     ## ##        ######## ########       ##     ## ###

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
static esp_err_t mpu_set_sample_rate(mpu_handle_t * mpu, uint16_t rate)
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
    dlpf_t dlpf = mpu_get_dlp_filter(mpu);
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
static uint16_t mpu_get_sample_rate(mpu_handle_t * mpu)
{
    uint16_t sample_rate_max_nolpf = 8000;

    dlpf_t dlpf                    = mpu_get_dlp_filter(mpu);
    esp_err_t err = mpu->last_err;
    if (MPU_ERR_CHECK(err)) return err;

    if (dlpf == 0 || dlpf == 7) return sample_rate_max_nolpf;

    uint16_t internal_sample_rate = 1000;
    uint8_t* buffer = mpu->buffer;
    err = mpu_i2c_read_byte(mpu, MPU_SMPLRT_DIV, buffer);
    if(MPU_ERR_CHECK(err)) return err;

    uint16_t rate = internal_sample_rate / (1 + buffer[0]);
    return rate;
}

// ########  ########    ###    ########  #### ##    ##  ######    ######
// ##     ## ##         ## ##   ##     ##  ##  ###   ## ##    ##  ##    ##
// ##     ## ##        ##   ##  ##     ##  ##  ####  ## ##        ##
// ########  ######   ##     ## ##     ##  ##  ## ## ## ##   ####  ######
// ##   ##   ##       ######### ##     ##  ##  ##  #### ##    ##        ##
// ##    ##  ##       ##     ## ##     ##  ##  ##   ### ##    ##  ##    ##
// ##     ## ######## ##     ## ########  #### ##    ##  ######    ######

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Read accelerometer raw data.
 * */
esp_err_t mpu_acceleration(mpu_handle_t * mpu, raw_axes_t* accel)
{
    uint8_t* buffer = mpu->buffer;
    esp_err_t err = mpu_i2c_read_bytes(mpu, MPU_ACCEL_XOUT_H, 6, buffer);
    if (MPU_ERR_CHECK(err)) return err;

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
    uint8_t* buffer = mpu->buffer;
    esp_err_t err = mpu_i2c_read_bytes(mpu, MPU_GYRO_XOUT_H, 6, buffer);
    if (MPU_ERR_CHECK(err)) return err;

    gyro->x = buffer[0] << 8 | buffer[1];
    gyro->y = buffer[2] << 8 | buffer[3];
    gyro->z = buffer[4] << 8 | buffer[5];

    return err;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Reset sensors signal path.
 *
 * Reset all gyro digital signal path, accel digital signal path, and temp
 * digital signal path. This also clears all the sensor registers.
 *
 * @note This function delays 100 ms, needed for reset to complete.
 * */
esp_err_t mpu_reset_signal_path(mpu_handle_t * mpu)
{
    esp_err_t err = mpu_i2c_write_bit(mpu, MPU_USER_CTRL,
                                      MPU_USERCTRL_SIG_COND_RESET_BIT, 1);
    if (MPU_ERR_CHECK(err)) return err;
    vTaskDelay(100 / portTICK_PERIOD_MS);
    return err;
}

// ##        #######  ##      ##    ########   #######  ##      ##
// ##       ##     ## ##  ##  ##    ##     ## ##     ## ##  ##  ##
// ##       ##     ## ##  ##  ##    ##     ## ##     ## ##  ##  ##
// ##       ##     ## ##  ##  ##    ########  ##     ## ##  ##  ##
// ##       ##     ## ##  ##  ##    ##        ##     ## ##  ##  ##
// ##       ##     ## ##  ##  ##    ##        ##     ## ##  ##  ##
// ########  #######   ###  ###     ##         #######   ###  ###

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Enter Low Power Accelerometer mode.
 *
 * In low-power accel mode, the chip goes to sleep and only wakes up to sample
 * the accelerometer at a certain frequency.
 * See setLowPowerAccelRate() to set the frequency.
 *
 * @param enable value
 *  + This function does the following when enable == true :
 *   - Set CYCLE bit to 1
 *   - Set SLEEP bit to 0
 *   - Set TEMP_DIS bit to 1
 *   - Set STBY_XG, STBY_YG, STBY_ZG bits to 1
 *   - Set STBY_XA, STBY_YA, STBY_ZA bits to 0
 *   - Set FCHOICE to 0 (ACCEL_FCHOICE_B bit to 1) [MPU6500 / MPU9250 only]
 *   - Disable Auxiliary I2C Master I/F
 *
 *  + This function does the following when enable == false :
 *   - Set CYCLE bit to 0
 *   - Set TEMP_DIS bit to 0
 *   - Set STBY_XG, STBY_YG, STBY_ZG bits to 0
 *   - Set STBY_XA, STBY_YA, STBY_ZA bits to 0
 *   - Set FCHOICE to 3 (ACCEL_FCHOICE_B bit to 0) [MPU6500 / MPU9250 only]
 *   - Enable Auxiliary I2C Master I/F
 * */
esp_err_t mpu_set_low_power_accel_mode(mpu_handle_t * mpu, bool enable)
{
    // read PWR_MGMT1 and PWR_MGMT2 at once
    uint8_t* buffer = mpu->buffer;
    esp_err_t err = mpu_i2c_read_bytes(mpu, MPU_PWR_MGMT1, 2, buffer);
    if (MPU_ERR_CHECK(err)) return err;

    if (enable)
    {
        // set CYCLE bit to 1 and SLEEP bit to 0 and TEMP_DIS bit to 1
        buffer[0] |= 1 << MPU_PWR1_CYCLE_BIT;
        buffer[0] &= ~(1 << MPU_PWR1_SLEEP_BIT);
        buffer[0] |= 1 << MPU_PWR1_TEMP_DIS_BIT;
        // set STBY_XG, STBY_YG, STBY_ZG bits to 1
        buffer[1] |= MPU_PWR2_STBY_XYZG_BITS;
    }
    else    // disable
    {
        // set CYCLE bit to 0 and TEMP_DIS bit to 0
        buffer[0] &= ~(1 << MPU_PWR1_CYCLE_BIT);
        buffer[0] &= ~(1 << MPU_PWR1_TEMP_DIS_BIT);
        // set STBY_XG, STBY_YG, STBY_ZG bits to 0
        buffer[1] &= ~(MPU_PWR2_STBY_XYZG_BITS);
    }
    // set STBY_XA, STBY_YA, STBY_ZA bits to 0
    buffer[1] &= ~(MPU_PWR2_STBY_XYZA_BITS);

    // write back PWR_MGMT1 and PWR_MGMT2 at once
    err = mpu_i2c_write_bytes(mpu, MPU_PWR_MGMT1, 2, buffer);
    if (MPU_ERR_CHECK(err)) return err;

    // disable Auxiliary I2C Master I/F in case it was active
    err = mpu_set_aux_i2c_enabled(mpu, !enable);
    if (MPU_ERR_CHECK(err)) return err;

    return err;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Return Low Power Accelerometer state.
 *
 * Condition to return true:
 *  - CYCLE bit is 1
 *  - SLEEP bit is 0
 *  - TEMP_DIS bit is 1
 *  - STBY_XG, STBY_YG, STBY_ZG bits are 1
 *  - STBY_XA, STBY_YA, STBY_ZA bits are 0
 *  - FCHOICE is 0 (ACCEL_FCHOICE_B bit is 1) [MPU6500 / MPU9250 only]
 *
 * */
bool mpu_get_low_power_accel_mode(mpu_handle_t * mpu)
{
    // read PWR_MGMT1 and PWR_MGMT2 at once
    uint8_t* buffer = mpu->buffer;
    esp_err_t err = mpu_i2c_read_bytes(mpu, MPU_PWR_MGMT1, 2, buffer);
    if (MPU_ERR_CHECK(err)) mpu->last_err = err;

    // define configuration bits
    uint8_t LPACCEL_CONFIG_BITMASK[2] =
    {
        (1 << MPU_PWR1_SLEEP_BIT) | (1 << MPU_PWR1_CYCLE_BIT) | (1 << MPU_PWR1_TEMP_DIS_BIT),
        MPU_PWR2_STBY_XYZA_BITS | MPU_PWR2_STBY_XYZG_BITS
    };
    uint8_t LPACCEL_ENABLED_VALUE[2] =
    {
        (1 << MPU_PWR1_CYCLE_BIT) | (1 << MPU_PWR1_TEMP_DIS_BIT),
        MPU_PWR2_STBY_XYZG_BITS
    };
    // get just the configuration bits
    buffer[0] &= LPACCEL_CONFIG_BITMASK[0];
    buffer[1] &= LPACCEL_CONFIG_BITMASK[1];
    // check pattern
    return buffer[0] == LPACCEL_ENABLED_VALUE[0]
           && buffer[1] == LPACCEL_ENABLED_VALUE[1];
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Set Low Power Accelerometer frequency of wake-up.
 * */
esp_err_t mpu_set_low_power_accel_rate(mpu_handle_t * mpu, lp_accel_rate_t rate)
{
    esp_err_t err = mpu_i2c_write_bits(mpu, MPU_PWR_MGMT2,
                                       MPU_PWR2_LP_WAKE_CTRL_BIT,
                                       MPU_PWR2_LP_WAKE_CTRL_LENGTH, rate);
    return MPU_ERR_CHECK(err);
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Get Low Power Accelerometer frequency of wake-up.
 */
lp_accel_rate_t mpu_get_low_power_accel_rate(mpu_handle_t * mpu)
{
    uint8_t* buffer = mpu->buffer;
    esp_err_t err = mpu_i2c_read_bits(mpu, MPU_PWR_MGMT2, MPU_PWR2_LP_WAKE_CTRL_BIT,
                                      MPU_PWR2_LP_WAKE_CTRL_LENGTH, buffer);
    if (MPU_ERR_CHECK(err)) mpu->last_err = err;

    return (lp_accel_rate_t) buffer[0];
}

//    ###    ##     ## ##     ##    ####  #######   ######
//   ## ##   ##     ##  ##   ##      ##  ##     ## ##    ##
//  ##   ##  ##     ##   ## ##       ##         ## ##
// ##     ## ##     ##    ###        ##   #######  ##
// ######### ##     ##   ## ##       ##  ##        ##
// ##     ## ##     ##  ##   ##      ##  ##        ##    ##
// ##     ##  #######  ##     ##    #### #########  ######

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Enable / disable Auxiliary I2C Master module.
 * */
static esp_err_t mpu_set_aux_i2c_enabled(mpu_handle_t * mpu, bool enable)
{
    esp_err_t err = mpu_i2c_write_bit(mpu, MPU_USER_CTRL,
                                      MPU_USERCTRL_I2C_MST_EN_BIT, (uint8_t) enable);
    if (MPU_ERR_CHECK(err)) return err;

    if (enable)
    {
        err = mpu_i2c_write_bit(mpu, MPU_INT_PIN_CONFIG,
                                MPU_INT_CFG_I2C_BYPASS_EN_BIT, 0);
        return (MPU_ERR_CHECK(err));
    }
    return err;
}

// ////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Return Auxiliary I2C Master state.
 */
static bool mpu_get_aux_i2c_enabled(mpu_handle_t * mpu)
{
    uint8_t* buffer = mpu->buffer;

    esp_err_t err = mpu_i2c_read_bit(mpu, MPU_USER_CTRL,
                                     MPU_USERCTRL_I2C_MST_EN_BIT, buffer);
    if (MPU_ERR_CHECK(err)) return err;

    err = mpu_i2c_read_bit(mpu, MPU_INT_PIN_CONFIG,
                           MPU_INT_CFG_I2C_BYPASS_EN_BIT, buffer + 1);
    if (MPU_ERR_CHECK(err)) return err;

    return buffer[0] && (!buffer[1]);
}

// ##     ##  #######  ######## ####  #######  ##    ##
// ###   ### ##     ##    ##     ##  ##     ## ###   ##
// #### #### ##     ##    ##     ##  ##     ## ####  ##
// ## ### ## ##     ##    ##     ##  ##     ## ## ## ##
// ##     ## ##     ##    ##     ##  ##     ## ##  ####
// ##     ## ##     ##    ##     ##  ##     ## ##   ###
// ##     ##  #######     ##    ####  #######  ##    ##

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Enable/disable Motion modules (Motion detect, Zero-motion, Free-Fall).
 *
 * @attention
 *  The configurations must've already been set with setMotionDetectConfig() before
 *  enabling the module!
 * @note
 *  - Call getMotionDetectStatus() to find out which axis generated motion interrupt.
 *    [MPU6000, MPU6050, MPU9150].
 *  - It is recommended to set the Motion Interrupt to propagate to the INT pin.
 *    To do that, use setInterruptEnabled().
 * @param enable
 *  - On _true_, this function modifies the DLPF, put gyro and temperature in standby,
 *    and disable Auxiliary I2C Master I/F.
 *  - On _false_, this function sets DLPF to 42Hz and enables Auxiliary I2C master I/F.
 * */
esp_err_t mpu_set_motion_feature_enabled(mpu_handle_t * mpu, bool enable)
{
    esp_err_t err;
    if (enable)
    {
        dlpf_t kDLPF = DLPF_256HZ_NOLPF;
        err = mpu_set_dlp_filter(mpu, kDLPF);
        if (MPU_ERR_CHECK(err)) return err;

        // give a time for accumulation of samples
        vTaskDelay(10 / portTICK_PERIOD_MS);
        err = mpu_i2c_write_bits(mpu, MPU_ACCEL_CONFIG, MPU_ACONFIG_HPF_BIT,
                                 MPU_ACONFIG_HPF_LENGTH, ACCEL_DHPF_HOLD);
        if (MPU_ERR_CHECK(err)) return err;
    }
    else
    {
        dlpf_t kDLPF = DLPF_42HZ;
        err = mpu_set_dlp_filter(mpu, kDLPF);
        if (MPU_ERR_CHECK(err)) return err;
    }
    // disable Auxiliary I2C Master I/F in case it was active
    err = mpu_set_aux_i2c_enabled(mpu, !enable);
    if (MPU_ERR_CHECK(err)) return err;
    return err;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Return true if a Motion Dectection module is enabled.
 */
bool mpu_get_motion_feature_enabled(mpu_handle_t * mpu)
{
    uint8_t data;
    esp_err_t err = mpu_i2c_read_bits(mpu, MPU_ACCEL_CONFIG, MPU_ACONFIG_HPF_BIT,
                                      MPU_ACONFIG_HPF_LENGTH, &data);
    if (MPU_ERR_CHECK(err)) mpu->last_err = err;

    if (data != ACCEL_DHPF_HOLD) return false;
    dlpf_t kDLPF = DLPF_256HZ_NOLPF;
    dlpf_t dlpf = mpu_get_dlp_filter(mpu);
    if (MPU_ERR_CHECK(mpu->last_err)) (void) 0;

    if (dlpf != kDLPF) return false;
    return true;
}

/**
 * @brief Configure Motion-Detect or Wake-on-motion feature.
 *
 * The behaviour of this feature is very different between the MPU6050 (MPU9150) and the
 * MPU6500 (MPU9250). Each chip's version of this feature is explained below.
 *
 * [MPU6050, MPU6000, MPU9150]:
 * Accelerometer measurements are passed through a configurable digital high pass filter (DHPF)
 * in order to eliminate bias due to gravity. A qualifying motion sample is one where the high
 * passed sample from any axis has an absolute value exceeding a user-programmable threshold. A
 * counter increments for each qualifying sample, and decrements for each non-qualifying sample.
 * Once the counter reaches a user-programmable counter threshold, a motion interrupt is triggered.
 * The axis and polarity which caused the interrupt to be triggered is flagged in the
 * MOT_DETECT_STATUS register.
 *
 * [MPU6500, MPU9250]:
 * Unlike the MPU6050 version, the hardware does not "lock in" a reference sample.
 * The hardware monitors the accel data and detects any large change over a short period of time.
 * A qualifying motion sample is one where the high passed sample from any axis has
 * an absolute value exceeding the threshold.
 * The hardware motion threshold can be between 4mg and 1020mg in 4mg increments.
 *
 * @note
 * It is possible to enable **wake-on-motion** mode by doing the following:
 *  1. Enter Low Power Accelerometer mode with setLowPowerAccelMode();
 *  2. Select the wake-up rate with setLowPowerAccelRate();
 *  3. Configure motion-detect interrupt with setMotionDetectConfig();
 *  4. Enable the motion detection module with setMotionFeatureEnabled();
 * */
esp_err_t mpu_set_motion_detect_config(mpu_handle_t * mpu, mot_config_t* config)
{
    esp_err_t err;
#if defined CONFIG_MPU_6050
    err = mpu_i2c_write_byte(mpu, MPU_MOTION_DUR, config->time);
    if (MPU_ERR_CHECK(err)) return err;

    err = mpu_i2c_write_bits(mpu, MPU_MOTION_DETECT_CTRL,
                             MPU_MOTCTRL_ACCEL_ON_DELAY_BIT,
                             MPU_MOTCTRL_ACCEL_ON_DELAY_LENGTH, config->accel_on_delay);
    if (MPU_ERR_CHECK(err)) return err;

    err = mpu_i2c_write_bits(mpu, MPU_MOTION_DETECT_CTRL,
                             MPU_MOTCTRL_MOT_COUNT_BIT, MPU_MOTCTRL_MOT_COUNT_LENGTH,
                             config->counter);
    if (MPU_ERR_CHECK(err)) return err;
#endif
    err = mpu_i2c_write_byte(mpu, MPU_MOTION_THR, config->threshold);
    if (MPU_ERR_CHECK(err)) return err;

    return err;
}

// /**
//  * @brief Return Motion Detection Configuration.
//  */
// mot_config_t MPU::getMotionDetectConfig()
// {
//     mot_config_t config{};
// #if defined CONFIG_MPU_6050
//     MPU_ERR_CHECK(readByte(MPU_MOTION_DUR, &config.time));
//     MPU_ERR_CHECK(readByte(MPU_MOTION_DETECT_CTRL, buffer));
//     config.accel_on_delay =
//         (buffer[0] >> (MPU_MOTCTRL_ACCEL_ON_DELAY_BIT - MPU_MOTCTRL_ACCEL_ON_DELAY_LENGTH + 1)) & 0x3;
//     config.counter =
//         (mot_counter_t)((buffer[0] >> (MPU_MOTCTRL_MOT_COUNT_BIT - MPU_MOTCTRL_MOT_COUNT_LENGTH + 1)) & 0x3);
// #endif
//     MPU_ERR_CHECK(readByte(MPU_MOTION_THR, &config.threshold));
//     return config;
// }

// #if defined CONFIG_MPU_6050
// /**
//  * @brief Configure Zero-Motion.
//  *
//  * The Zero Motion detection capability uses the digital high pass filter (DHPF) and a similar
//  * threshold scheme to that of Free Fall detection. Each axis of the high passed accelerometer
//  * measurement must have an absolute value less than a threshold specified in the ZRMOT_THR
//  * register, which can be increased in 1 mg increments. Each time a motion sample meets this
//  * condition, a counter increments. When this counter reaches a threshold specified in ZRMOT_DUR, an
//  * interrupt is generated.
//  *
//  * Unlike Free Fall or Motion detection, Zero Motion detection triggers an interrupt both when Zero
//  * Motion is first detected and when Zero Motion is no longer detected. While Free Fall and Motion
//  * are indicated with a flag which clears after being read, reading the state of the Zero Motion
//  * detected from the MOT_DETECT_STATUS register does not clear its status.
//  *
//  * @note Enable by calling setMotionFeatureEnabled();
//  * */
// esp_err_t MPU::setZeroMotionConfig(zrmot_config_t& config)
// {
//     buffer[0] = config.threshold;
//     buffer[1] = config.time;
//     return MPU_ERR_CHECK(writeBytes(MPU_ZRMOTION_THR, 2, buffer));
// }

// /**
//  * @brief Return Zero-Motion configuration.
//  */
// zrmot_config_t MPU::getZeroMotionConfig()
// {
//     MPU_ERR_CHECK(readBytes(MPU_ZRMOTION_THR, 2, buffer));
//     zrmot_config_t config{};
//     config.threshold = buffer[0];
//     config.time      = buffer[1];
//     return config;
// }

// /**
//  * @brief Configure Free-Fall.
//  *
//  * Free fall is detected by checking if the accelerometer measurements from all 3 axes have an
//  * absolute value below a user-programmable threshold (acceleration threshold). For each sample
//  * where this condition is true (a qualifying sample), a counter is incremented. For each sample
//  * where this condition is false (a non- qualifying sample), the counter is decremented. Once the
//  * counter reaches a user-programmable threshold (the counter threshold), the Free Fall interrupt is
//  * triggered and a flag is set. The flag is cleared once the counter has decremented to zero. The
//  * counter does not increment above the counter threshold or decrement below zero.
//  *
//  * @note Enable by calling setMotionFeatureEnabled().
//  * */
// esp_err_t MPU::setFreeFallConfig(ff_config_t& config)
// {
//     buffer[0] = config.threshold;
//     buffer[1] = config.time;
//     if (MPU_ERR_CHECK(writeBytes(MPU_FF_THR, 2, buffer))) return err;
//     if (MPU_ERR_CHECK(writeBits(MPU_MOTION_DETECT_CTRL, MPU_MOTCTRL_ACCEL_ON_DELAY_BIT,
//                                 MPU_MOTCTRL_ACCEL_ON_DELAY_LENGTH, config.accel_on_delay))) {
//         return err;
//     }
//     if (MPU_ERR_CHECK(writeBits(MPU_MOTION_DETECT_CTRL, MPU_MOTCTRL_MOT_COUNT_BIT, MPU_MOTCTRL_MOT_COUNT_LENGTH,
//                                 config.counter))) {
//         return err;
//     }
//     return err;
// }

// /**
//  * @brief Return Free-Fall Configuration.
//  */
// ff_config_t MPU::getFreeFallConfig()
// {
//     ff_config_t config{};
//     MPU_ERR_CHECK(readBytes(MPU_FF_THR, 2, buffer));
//     config.threshold = buffer[0];
//     config.time      = buffer[1];
//     MPU_ERR_CHECK(readByte(MPU_MOTION_DETECT_CTRL, buffer));
//     config.accel_on_delay =
//         (buffer[0] >> (MPU_MOTCTRL_ACCEL_ON_DELAY_BIT - MPU_MOTCTRL_ACCEL_ON_DELAY_LENGTH + 1)) & 0x3;
//     config.counter =
//         (mot_counter_t)((buffer[0] >> (MPU_MOTCTRL_MOT_COUNT_BIT - MPU_MOTCTRL_MOT_COUNT_LENGTH + 1)) & 0x3);
//     return config;
// }

// /**
//  * @brief Return Motion Detection Status.
//  * @note Reading this register clears all motion detection status bits.
//  * */
// mot_stat_t MPU::getMotionDetectStatus()
// {
//     MPU_ERR_CHECK(readByte(MPU_MOTION_DETECT_STATUS, buffer));
//     return (mot_stat_t) buffer[0];
// }
// #endif  // MPU6050's stuff

// /**
//  * @brief Configure sensors' standby mode.
//  * */
// esp_err_t MPU::setStandbyMode(stby_en_t mask)
// {
//     const uint8_t kPwr1StbyBits = mask >> 6;
//     if (MPU_ERR_CHECK(writeBits(MPU_PWR_MGMT1, MPU_PWR1_GYRO_STANDBY_BIT, 2, kPwr1StbyBits))) {
//         return err;
//     }
//     return MPU_ERR_CHECK(writeBits(MPU_PWR_MGMT2, MPU_PWR2_STBY_XA_BIT, 6, mask));
// }

// /**
//  * @brief Return Standby configuration.
//  * */
// stby_en_t MPU::getStandbyMode()
// {
//     MPU_ERR_CHECK(readBytes(MPU_PWR_MGMT1, 2, buffer));
//     constexpr uint8_t kStbyTempAndGyroPLLBits = STBY_EN_TEMP | STBY_EN_LOWPWR_GYRO_PLL_ON;
//     stby_en_t mask                            = buffer[0] << 3 & kStbyTempAndGyroPLLBits;
//     constexpr uint8_t kStbyAccelAndGyroBits   = STBY_EN_ACCEL | STBY_EN_GYRO;
//     mask |= buffer[1] & kStbyAccelAndGyroBits;
//     return mask;
// }