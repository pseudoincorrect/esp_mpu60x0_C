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

#ifndef __mpu60x0_types_h__
#define __mpu60x0_types_h__

#include "driver/gpio.h"

#ifdef CONFIG_MPU_I2C

// ######## ##    ## ########  ######## ########  ######## ########
//    ##     ##  ##  ##     ## ##       ##     ## ##       ##
//    ##      ####   ##     ## ##       ##     ## ##       ##
//    ##       ##    ########  ######   ##     ## ######   ######
//    ##       ##    ##        ##       ##     ## ##       ##
//    ##       ##    ##        ##       ##     ## ##       ##
//    ##       ##    ##        ######## ########  ######## ##

////////////////////////////////////////////////////////////////////////////////
/** @brief Port number of the I2C peripheral  */
typedef int mpu_periph_num_t;

////////////////////////////////////////////////////////////////////////////////
/** @brief Variables for initialization of the I2C */
typedef struct mpu_initializer_struct
{
    gpio_num_t sda_io_num;
    gpio_num_t scl_io_num;
    gpio_pullup_t sda_pullup_en;
    gpio_pullup_t scl_pullup_en;
    uint32_t clk_speed;
} mpu_initializer_t;

#elif

////////////////////////////////////////////////////////////////////////////////
/** @brief Port number of the SPI peripheral  */
typedef int mpu_periph_num_t;

////////////////////////////////////////////////////////////////////////////////
/** @brief Variables for initialization of the SPI */
typedef struct mpu_initializer_struct
{
    int init;
} mpu_initializer_t;

#endif

////////////////////////////////////////////////////////////////////////////////
/** @brief I2C bus address */
typedef uint8_t mpu_addr_t;

////////////////////////////////////////////////////////////////////////////////
/** @brief I2C/SPI bus metadata handler */
typedef struct mpu_bus_struct
{
    mpu_periph_num_t num;
    int timeout; // timeout for read/write operations
} mpu_bus_t;

////////////////////////////////////////////////////////////////////////////////
/** @brief main MPU data/metadata handler */
typedef struct mpu_handle_struct
{
    mpu_initializer_t init;
    mpu_bus_t bus;      // Communication bus pointer, I2C / SPI
    mpu_addr_t addr;    // I2C address / SPI device handle
    uint8_t buffer[16]; // Commom buffer for temporary data
    esp_err_t last_err; // Holds last error code
} mpu_handle_t;

////////////////////////////////////////////////////////////////////////////////
/** @brief data structure (INT) for Acceleration/Gyro measurements */
typedef struct raw_axes_struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} raw_axes_t;

////////////////////////////////////////////////////////////////////////////////
/** @brief data structure (FLOAT) for Acceleration/Gyro measurements */
typedef struct float_axes_struct
{
    float x;
    float y;
    float z;
} float_axes_t;

////////////////////////////////////////////////////////////////////////////////
/** @brief Motion Detection counter decrement rate (Motion and FreeFall) */
typedef enum mot_counter_struct{
    MOT_COUNTER_RESET = 0,  //!< When set, any non-qualifying sample will reset the corresponding counter to 0
    MOT_COUNTER_DEC_1 = 1,  //!< Decrement counter in 1
    MOT_COUNTER_DEC_2 = 2,  //!< Decrement counter in 2
    MOT_COUNTER_DEC_4 = 3   //!< Decrement counter in 4
} mot_counter_t;

////////////////////////////////////////////////////////////////////////////////
/** @brief Motion Detection configuration */
typedef struct mot_config_struct
{
    /**< Motion threshold in LSB.
    * For MPU6000 / MPU6050 / MPU9150: 1LSB = 32mg, 255LSB = 8160mg.
    * For MPU6500 / MPU9250: 1LSB = 4mg, 255LSB = 1020mg. */
    uint8_t threshold;

    /**< Duration in milliseconds that the accel data must exceed
    * the threshold before motion is reported. MAX = 255ms. */
    uint8_t time;

    /**< Specifies in milliseconds the additional power-on delay applied to accelerometer
    * data path modules. MAX = 3ms.
    * More: The signal path contains filters which must be flushed on wake-up with new
    * samples before the detection modules begin operations.
    * There is already a default built-in 4ms delay. */
    uint8_t accel_on_delay : 2;

    //!< Configures the detection counter decrement rate.
    mot_counter_t counter : 2;
} mot_config_t;

// ######## ##    ## ##     ## ##     ##
// ##       ###   ## ##     ## ###   ###
// ##       ####  ## ##     ## #### ####
// ######   ## ## ## ##     ## ## ### ##
// ##       ##  #### ##     ## ##     ##
// ##       ##   ### ##     ## ##     ##
// ######## ##    ##  #######  ##     ##

////////////////////////////////////////////////////////////////////////////////
/** @brief Clock Source */
typedef enum mpu_clock_src_enum
{
    CLOCK_INTERNAL = 0,  // Internal oscillator: 20MHz for MPU6500 and 8MHz for MPU6050
    CLOCK_PLL      = 3,  // Selects automatically best pll source (recommended)
#if defined CONFIG_MPU_6050
    CLOCK_EXT32KHZ = 4,  // PLL with external 32.768kHz reference
    CLOCK_EXT19MHZ = 5,  // PLL with external 19.2MHz reference
#endif
    CLOCK_KEEP_RESET = 7  // Stops the clock and keeps timing generator in reset
} mpu_clock_src_t;

////////////////////////////////////////////////////////////////////////////////
/** @brief Gyroscope full-scale range */
typedef enum gyro_fs_enum
{
    GYRO_FS_250DPS  = 0,  //!< +/- 250 º/s  -> 131 LSB/(º/s)
    GYRO_FS_500DPS  = 1,  //!< +/- 500 º/s  -> 65.5 LSB/(º/s)
    GYRO_FS_1000DPS = 2,  //!< +/- 1000 º/s -> 32.8 LSB/(º/s)
    GYRO_FS_2000DPS = 3   //!< +/- 2000 º/s -> 16.4 LSB/(º/s)
} gyro_fs_t;

////////////////////////////////////////////////////////////////////////////////
/** @brief Accel full-scale range */
typedef enum accel_fs_enum
{
    ACCEL_FS_2G  = 0,  //!< +/- 2 g  -> 16.384 LSB/g
    ACCEL_FS_4G  = 1,  //!< +/- 4 g  -> 8.192 LSB/g
    ACCEL_FS_8G  = 2,  //!< +/- 8 g  -> 4.096 LSB/g
    ACCEL_FS_16G = 3   //!< +/- 16 g -> 2.048 LSB/g
} accel_fs_t;

////////////////////////////////////////////////////////////////////////////////
/** @brief Digital low-pass filter (based on gyro bandwidth) */
typedef enum dlpf_enum
{
    DLPF_256HZ_NOLPF = 0,
    DLPF_188HZ       = 1,
    DLPF_98HZ        = 2,
    DLPF_42HZ        = 3,
    DLPF_20HZ        = 4,
    DLPF_10HZ        = 5,
    DLPF_5HZ         = 6,
#ifdef CONFIG_MPU_6050
    DLPF_2100HZ_NOLPF = 7
#endif
} dlpf_t;


////////////////////////////////////////////////////////////////////////////////
/** @brief Low-Power Accelerometer wake-up rates */
typedef enum lp_accel_rate_enum
{
    LP_ACCEL_RATE_1_25HZ = 0,
    LP_ACCEL_RATE_5HZ    = 1,
    LP_ACCEL_RATE_20HZ   = 2,
    LP_ACCEL_RATE_40HZ   = 3
} lp_accel_rate_t;

////////////////////////////////////////////////////////////////////////////////
/** @brief Accelerometer Digital High Pass Filter (only for motion detection modules) */
typedef enum accel_dhpf_enum
{
    ACCEL_DHPF_RESET = 0,   /**< This effectively disables the high pass filter. This mode may be toggled to quickly
                             * settle the filter. */
    ACCEL_DHPF_5HZ    = 1,  //!< ON state, the high pass filter will pass signals above the cut off frequency.
    ACCEL_DHPF_2_5HZ  = 2,  //!< ON state, the high pass filter will pass signals above the cut off frequency.
    ACCEL_DHPF_1_25HZ = 3,  //!< ON state, the high pass filter will pass signals above the cut off frequency.
    ACCEL_DHPF_0_63HZ = 4,  //!< ON state, the high pass filter will pass signals above the cut off frequency.
    ACCEL_DHPF_HOLD   = 7,  /**< The filter holds the present sample. The output will be the difference between the
                             * input sample and the held sample. */
} accel_dhpf_t;

#endif // ifndef __mpu60x0_types_h__