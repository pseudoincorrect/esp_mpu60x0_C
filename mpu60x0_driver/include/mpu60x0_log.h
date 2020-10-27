#ifndef _MPU_LOG_HPP_
#define _MPU_LOG_HPP_

#include "esp_err.h"
#include "esp_log.h"
#include "sdkconfig.h"

// Note: declare TAG before include this header
// Note: include only in .cpp files from this library

#define MPU_LOGE(format, ...) if (CONFIG_MPU_LOG_LEVEL >= ESP_LOG_ERROR)   { ESP_LOGE(TAG, format, ##__VA_ARGS__); }
#define MPU_LOGW(format, ...) if (CONFIG_MPU_LOG_LEVEL >= ESP_LOG_WARN)    { ESP_LOGW(TAG, format, ##__VA_ARGS__); }
#define MPU_LOGI(format, ...) if (CONFIG_MPU_LOG_LEVEL >= ESP_LOG_INFO)    { ESP_LOGI(TAG, format, ##__VA_ARGS__); }
#define MPU_LOGD(format, ...) if (CONFIG_MPU_LOG_LEVEL >= ESP_LOG_DEBUG)   { ESP_LOGD(TAG, format, ##__VA_ARGS__); }

#define MPU_LOGEMSG(msg, format, ...) MPU_LOGE("%s()-> %s" format, __FUNCTION__, msg, ##__VA_ARGS__)
#define MPU_LOGWMSG(msg, format, ...) MPU_LOGW("%s()-> %s" format, __FUNCTION__, msg, ##__VA_ARGS__)
#define MPU_LOGIMSG(msg, format, ...) MPU_LOGI("%s()-> %s" format, __FUNCTION__, msg, ##__VA_ARGS__)
#define MPU_LOGDMSG(msg, format, ...) MPU_LOGD("%s()-> %s" format, __FUNCTION__, msg, ##__VA_ARGS__)

#ifndef MPU_LOG_ON_ERROR
#define MPU_ERR_CHECK(x) (x)
#define MPU_ERR_CHECK_STATEMENT(x) { if (x) (void)0; }
#else
#define MPU_ERR_CHECK(x) errorCheckLogger(x, __ASSERT_FUNC, __LINE__, #x)
#define MPU_ERR_CHECK_STATEMENT(x)  MPU_ERR_CHECK(x)
#endif

static inline esp_err_t errorCheckLogger(esp_err_t x, const char* func, const int line, const char* expr)
{
    if (x) MPU_LOGE("func:%s @ line:%d, expr:\"%s\", error:0x%X ", func, line, expr, x);
    return x;
}

#endif