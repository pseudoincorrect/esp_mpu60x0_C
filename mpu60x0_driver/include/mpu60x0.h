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

static esp_err_t who_am_i(mpu_handle_t * mpu, uint8_t * buff);

esp_err_t mpu_test_connection(mpu_handle_t * mpu);

static esp_err_t set_clock_src(mpu_handle_t * mpu, mpu_clock_src_t clockSrc);

static mpu_clock_src_t get_clock_src(mpu_handle_t * mpu);

static esp_err_t set_gyro_full_scale(mpu_handle_t * mpu, gyro_fs_t fsr);

static gyro_fs_t get_gyro_full_scale(mpu_handle_t * mpu);

static esp_err_t set_digital_low_pass_filter(mpu_handle_t * mpu, dlpf_t dlpf);

static dlpf_t get_digital_low_pass_filter(mpu_handle_t * mpu);

#endif