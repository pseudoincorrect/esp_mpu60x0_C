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

#ifndef __mpu60x0_math_h__
#define __mpu60x0_math_h__

#include <math.h>
#include <stdint.h>
#include "mpu/types.hpp"
#include "sdkconfig.h"

////////////////////////////////////////////////////////////////////////////////
inline uint8_t accel_fsr_value(const accel_fs_t fs)
{
    return 2 << fs;
}

////////////////////////////////////////////////////////////////////////////////
inline uint16_t gyro_fsr_value(const gyro_fs_t fs)
{
    return 250 << fs;
}

////////////////////////////////////////////////////////////////////////////////
inline uint16_t accel_sensitivity(const accel_fs_t fs)
{
    return 16384 >> fs;
}

////////////////////////////////////////////////////////////////////////////////
inline float gyro_sensitivity(const gyro_fs_t fs)
{
    return 131.f / (1 << fs);
}

////////////////////////////////////////////////////////////////////////////////
inline float accel_resolution(const accel_fs_t fs)
{
    return (float) (accel_fsr_value(fs)) / INT16_MAX;
}

////////////////////////////////////////////////////////////////////////////////
inline float gyro_resolution(const gyro_fs_t fs)
{
    return (float) (gyro_fsr_value(fs)) / INT16_MAX;
}

////////////////////////////////////////////////////////////////////////////////
inline float accel_gravity(const int16_t axis, const accel_fs_t fs)
{
    return axis * accel_resolution(fs);
}

////////////////////////////////////////////////////////////////////////////////
inline float_axes_t accel_gravity(const raw_axes_t& raw_axes, const accel_fs_t fs)
{
    float_axes_t axes;
    axes.x = raw_axes.x * accel_resolution(fs);
    axes.y = raw_axes.y * accel_resolution(fs);
    axes.z = raw_axes.z * accel_resolution(fs);
    return axes;
}

////////////////////////////////////////////////////////////////////////////////
inline float gyro_deg_per_sec(const int16_t axis, const gyro_fs_t fs)
{
    return axis * gyro_resolution(fs);
}

////////////////////////////////////////////////////////////////////////////////
inline float_axes_t gyro_deg_per_sec(const raw_axes_t& raw_axes, const gyro_fs_t fs)
{
    float_axes_t axes;
    axes.x = raw_axes.x * gyro_resolution(fs);
    axes.y = raw_axes.y * gyro_resolution(fs);
    axes.z = raw_axes.z * gyro_resolution(fs);
    return axes;
}

////////////////////////////////////////////////////////////////////////////////
inline float gyro_rad_per_sec(const int16_t axis, const gyro_fs_t fs)
{
    return (M_PI / 180) * gyro_deg_per_sec(axis, fs);
}

////////////////////////////////////////////////////////////////////////////////
inline float_axes_t gyro_rad_per_sec(const raw_axes_t& raw_axes, const gyro_fs_t fs)
{
    float_axes_t axes;
    axes.x = (M_PI / 180) * gyro_deg_per_sec(raw_axes.x, fs);
    axes.y = (M_PI / 180) * gyro_deg_per_sec(raw_axes.y, fs);
    axes.z = (M_PI / 180) * gyro_deg_per_sec(raw_axes.z, fs);
    return axes;
}


constexpr int16_t k_room_temp_offset = -521;   // LSB
constexpr float k_celsius_offset    = 35.f;   // ºC
constexpr float k_temp_sensitivity  = 340.f;  // LSB/ºC
constexpr float k_temp_resolution   = 98.67f / INT16_MAX;
constexpr float k_fahrenheit_offset = k_celsius_offset * 1.8f + 32;  // ºF

////////////////////////////////////////////////////////////////////////////////
inline float temp_celsius(const int16_t temp)
{
    // temp_deg_c = ( ( temp_out – room_temp_offset ) / temp_sensitivity ) + degrees_celsius_offset
    return (temp - k_room_temp_offset) * k_temp_resolution + k_celsius_offset;
}

////////////////////////////////////////////////////////////////////////////////
inline float temp_fahrenheit(const int16_t temp)
{
    return (temp - k_room_temp_offset) * k_temp_resolution * 1.8f + k_fahrenheit_offset;
}

#endif // ifndef __mpu60x0_math_h__
