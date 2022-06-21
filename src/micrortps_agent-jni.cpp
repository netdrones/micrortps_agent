#include <jni.h>
#include <memory>
#include <string>
#include <jni.h>
#include "MicroRTPSAgent.h"
#include <jni.h>

static std::unique_ptr<netdrones::axon::MicroRTPSAgent> g_agent(nullptr);

extern "C"
void Java_es_netdron_axon_MicroRtpsAgent_nativeInitUART(
    JNIEnv*,
    jclass,
    jint fd,
    jint baudrate,
    jint poll_interval,
    jboolean sw_flow_control,
    jboolean hw_flow_control,
    jboolean verbose
) {
    if (!g_agent) {
        g_agent = std::make_unique<netdrones::axon::MicroRTPSAgent>(
            fd,
            baudrate,
            poll_interval,
            sw_flow_control,
            hw_flow_control,
            verbose
        );
    }
}

extern "C"
jboolean Java_es_netdron_axon_MicroRtpsAgent_start(JNIEnv*, jclass) {
    if (!g_agent) return false;
    return g_agent->Start();
}

extern "C"
jboolean Java_es_netdron_axon_MicroRtpsAgent_stop(JNIEnv*, jclass) {
    if (!g_agent) return false;
    return g_agent->Stop();
}

JNIEXPORT jint JNI_OnLoad(JavaVM* vm, void*) {
    JNIEnv* env;

    if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK) {
        return JNI_ERR;
    }

    auto klass = env->FindClass("es/netdron/axon/MicroRtpsAgent");
    if (!klass) return JNI_ERR;

    static const JNINativeMethod methods[] = {
        {"nativeInitUART", "(IIIZZZ)V", reinterpret_cast<void*>(Java_es_netdron_axon_MicroRtpsAgent_nativeInitUART)},
        {"start", "()Z", reinterpret_cast<void*>(Java_es_netdron_axon_MicroRtpsAgent_start)},
        {"stop", "()Z", reinterpret_cast<void*>(Java_es_netdron_axon_MicroRtpsAgent_stop)},
    };
    auto rc = env->RegisterNatives(klass, methods, sizeof(methods) / sizeof(JNINativeMethod));
    if (rc != JNI_OK) return rc;

    return JNI_VERSION_1_6;
}
