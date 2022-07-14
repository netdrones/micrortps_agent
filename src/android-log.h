#ifndef COPILOT_ANDROID_LOG_H
#define COPILOT_ANDROID_LOG_H

#ifdef ANDROID

#include <android/log.h>

#define ANDROID_LOG_TAG "micrortps_agent"

//#ifndef NDEBUG
//#define  LOGV(...)
//#define  LOGD(...)
//#define  LOGI(...)
//#define  LOGW(...)
//#define  LOGE(...)
//#else
#define  PX4_DEBUG(format, ...)  __android_log_print(ANDROID_LOG_DEBUG, ANDROID_LOG_TAG , "[%s:%d/%s] " format, basename(__FILE__), __LINE__, __FUNCTION__, ##__VA_ARGS__)
#define  PX4_ERR(format, ...)  __android_log_print(ANDROID_LOG_ERROR, ANDROID_LOG_TAG , "[%s:%d/%s] " format, basename(__FILE__), __LINE__, __FUNCTION__, ##__VA_ARGS__)
//#endif // NDEBUG

#endif // ANDROID

#endif //COPILOT_ANDROID_LOG_H
