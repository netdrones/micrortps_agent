#ifndef MICRORTPS_AGENT_LOGGING_ANDROID_H
#define MICRORTPS_AGENT_LOGGING_ANDROID_H

#include <android/log.h>

#define MICRORTPS_AGENT_LOG_TAG "MicroRtpsAgent"

#define  LOGV(...)  __android_log_print(ANDROID_LOG_VERBOSE,    MICRORTPS_AGENT_LOG_TAG, __VA_ARGS__)
#define  LOGW(...)  __android_log_print(ANDROID_LOG_WARN,       MICRORTPS_AGENT_LOG_TAG, __VA_ARGS__)
#define  LOGD(...)  __android_log_print(ANDROID_LOG_DEBUG,      MICRORTPS_AGENT_LOG_TAG, __VA_ARGS__)
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,       MICRORTPS_AGENT_LOG_TAG, __VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,      MICRORTPS_AGENT_LOG_TAG, __VA_ARGS__)


#endif //AXON_LOGGING_ANDROID_H
