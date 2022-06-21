#ifndef AXON_ANDROID_LOG_H
#define AXON_ANDROID_LOG_H

#ifdef ANDROID

#include <android/log.h>

#define MICRORTPS_AGENT_LOGTAG "MicroRTPSAgent"

#define PX4_WARN(...)  __android_log_print(ANDROID_LOG_WARN,  MICRORTPS_AGENT_LOGTAG, __VA_ARGS__)
#define PX4_DEBUG(...) __android_log_print(ANDROID_LOG_DEBUG, MICRORTPS_AGENT_LOGTAG, __VA_ARGS__)
#define PX4_INFO(...)  __android_log_print(ANDROID_LOG_INFO,  MICRORTPS_AGENT_LOGTAG, __VA_ARGS__)
#define PX4_ERR(...)   __android_log_print(ANDROID_LOG_ERROR, MICRORTPS_AGENT_LOGTAG, __VA_ARGS__)

#endif // ANDROID

#endif //AXON_ANDROID_LOG_H
