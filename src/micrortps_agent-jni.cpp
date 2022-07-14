// Copyright 2022 NetDrones Inc. All rights reserved.
#include <jni.h>
#include <memory>
#include <string>
#include "MicroRTPSAgent.h"
#include "JNIUtils.h"
#include "logger.h"

using namespace netdrones::jni;
using netdrones::pilot::MicroRTPSAgent;

extern "C"
void Java_es_netdron_copilot_service_MicroRtpsAgent_nativeInitUART(
    JNIEnv* env,
    jobject thiz,
    jint fd,
    jint baudrate,
    jint poll_interval,
    jboolean sw_flow_control,
    jboolean hw_flow_control,
    jboolean verbose
) {
    auto agent = new MicroRTPSAgent(
        fd,
        baudrate,
        poll_interval,
        sw_flow_control,
        hw_flow_control,
        verbose
    );
    env->SetLongField(thiz, get_native_handle_fieldID(env, thiz),
                      reinterpret_cast<jlong>(agent));
}

extern "C"
void Java_es_netdrone_copilot_service_MicroRtpsAgent_setRosLocalhostOnly(
    JNIEnv* env,
    jobject thiz,
    jboolean only
) {
    native_handle<MicroRTPSAgent>(env, thiz)->set_ros_localhost_only(only);
}

extern "C"
void Java_es_netdrone_copilot_service_MicroRtpsAgent_setRosNamespace(
    JNIEnv* env,
    jobject thiz,
    jstring ns_
) {
    JString ns(env, ns_);

    native_handle<MicroRTPSAgent>(env, thiz)->set_ros_namespace(ns.String());
}

extern "C"
jboolean Java_es_netdron_copilot_service_MicroRtpsAgent_start(
    JNIEnv* env,
    jobject thiz
) {
    return native_handle<MicroRTPSAgent>(env, thiz)->Start();
}

extern "C"
jboolean Java_es_netdron_copilot_service_MicroRtpsAgent_stop(
    JNIEnv* env,
    jobject thiz
) {
    return native_handle<MicroRTPSAgent>(env, thiz)->Stop();
}

extern "C"
void Java_es_netdron_copilot_service_MicroRtpsAgent_nativeRelease(
    JNIEnv* env,
    jobject thiz
) {
    auto agent = native_handle<MicroRTPSAgent>(env, thiz);
    if (agent) {
        delete agent;
    }
}

JNIEXPORT jint JNI_OnLoad(JavaVM* vm, void*) {
    JNIEnv* env;

    if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK) {
        return JNI_ERR;
    }

    auto klass = env->FindClass("es/netdron/copilot/service/MicroRtpsAgent");
    if (!klass) return JNI_ERR;

    static const JNINativeMethod methods[] = {
        {"nativeInitUART", "(IIIZZZ)V", reinterpret_cast<void*>(Java_es_netdron_copilot_service_MicroRtpsAgent_nativeInitUART)},
        {"setRosLocalhostOnly", "(Z)V", reinterpret_cast<void*>(Java_es_netdrone_copilot_service_MicroRtpsAgent_setRosLocalhostOnly)},
        {"setRosNamespace", "(Ljava/lang/String;)V", reinterpret_cast<void*>(Java_es_netdrone_copilot_service_MicroRtpsAgent_setRosNamespace)},
        {"nativeRelease", "()V", reinterpret_cast<void*>(Java_es_netdron_copilot_service_MicroRtpsAgent_nativeRelease)},
        {"start", "()Z", reinterpret_cast<void*>(Java_es_netdron_copilot_service_MicroRtpsAgent_start)},
        {"stop", "()Z", reinterpret_cast<void*>(Java_es_netdron_copilot_service_MicroRtpsAgent_stop)},
    };
    auto rc = env->RegisterNatives(klass, methods, sizeof(methods) / sizeof(JNINativeMethod));
    if (rc != JNI_OK) return rc;

    return JNI_VERSION_1_6;
}
