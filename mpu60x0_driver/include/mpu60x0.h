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

#ifndef __mpu60x0_h__
#define __mpu60x0_h__

#include "mpu60x0_types.h"
#include "esp_err.h"

void mpu_print_something(void);

esp_err_t mpu_initialize_peripheral(mpu_handle_t * mpu);

esp_err_t mpu_initialize_chip(mpu_handle_t * mpu);

esp_err_t mpu_reset(mpu_handle_t * mpu);

esp_err_t mpu_set_sleep(mpu_handle_t * mpu, bool enable);

bool mpu_get_sleep(mpu_handle_t * mpu);

esp_err_t mpu_test_connection(mpu_handle_t * mpu);

esp_err_t mpu_acceleration(mpu_handle_t * mpu, raw_axes_t* accel);

esp_err_t mpu_rotation(mpu_handle_t * mpu, raw_axes_t* gyro);

#endif // ifndef __mpu60x0_h__