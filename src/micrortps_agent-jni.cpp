// Copyright 2022 NetDrones Inc. All rights reserved.

#include <jni.h>
#include <memory>
#include <string>
#include "MicroRTPSAgent.h"
#include "../utils/include/JNIUtils.h"

using netdrones::micrortps_agent::MicroRTPSAgent;

extern "C"
void Java_es_netdron_micrortps_1agent_MicroRtpsAgent_nativeInitUART(
    JNIEnv* env,
    jobject thiz,
    jint fd,
    jint baudrate,
    jint flow_ctrl,
    jboolean debug
) {
    auto agent = new MicroRTPSAgent(
        fd,
        baudrate,
        flow_ctrl,
        debug
    );
    SetNativeHandle(env, thiz, agent);
}

extern "C"
void Java_es_netdron_micrortps_1agent_MicroRtpsAgent_nativeInitUdp(
    JNIEnv* env,
    jobject thiz,
    jint recv_port,
    jint send_port
) {
    auto agent = new MicroRTPSAgent(recv_port, send_port);
    SetNativeHandle(env, thiz, agent);
}

extern "C"
void Java_es_netdron_micrortps_1agent_MicroRtpsAgent_setRosLocalhostOnly(
    JNIEnv* env,
    jobject thiz,
    jboolean only
) {
    GetNativeHandle<MicroRTPSAgent>(env, thiz)->set_ros_localhost_only(only);
}

extern "C"
void Java_es_netdron_micrortps_1agent_MicroRtpsAgent_setRosNamespace(
    JNIEnv* env,
    jobject thiz,
    jstring ns_
) {
    JString ns(env, ns_);
    GetNativeHandle<MicroRTPSAgent>(env, thiz)->set_ros_namespace(ns.String());
}

extern "C"
jboolean Java_es_netdron_micrortps_1agent_MicroRtpsAgent_start(
    JNIEnv* env,
    jobject thiz
) {
    return GetNativeHandle<MicroRTPSAgent>(env, thiz)->Start();
}

extern "C"
jboolean Java_es_netdron_micrortps_1agent_MicroRtpsAgent_stop(
    JNIEnv* env,
    jobject thiz
) {
    return GetNativeHandle<MicroRTPSAgent>(env, thiz)->Stop();
}

extern "C"
void Java_es_netdron_micrortps_1agent_MicroRtpsAgent_nativeRelease(
    JNIEnv* env,
    jobject thiz
) {
    delete GetNativeHandle<MicroRTPSAgent>(env, thiz);
}

JNIEXPORT jint JNI_OnLoad(JavaVM* vm, void*) {
    JNIEnv* env;

    if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK) {
        return JNI_ERR;
    }

    auto klass = env->FindClass("es/netdron/micrortps_agent/MicroRtpsAgent");
    if (!klass) return JNI_ERR;

    static const JNINativeMethod methods[] = {
        {"nativeInitUART", "(IIIZ)V", reinterpret_cast<void*>(Java_es_netdron_micrortps_1agent_MicroRtpsAgent_nativeInitUART)},
        {"nativeRelease", "()V", reinterpret_cast<void*>(Java_es_netdron_micrortps_1agent_MicroRtpsAgent_nativeRelease)},
        {"setRosLocalhostOnly", "(Z)V", reinterpret_cast<void*>(Java_es_netdron_micrortps_1agent_MicroRtpsAgent_setRosLocalhostOnly)},
        {"setRosNamespace", "(Ljava/lang/String;)V", reinterpret_cast<void*>(Java_es_netdron_micrortps_1agent_MicroRtpsAgent_setRosNamespace)},
        {"start", "()Z", reinterpret_cast<void*>(Java_es_netdron_micrortps_1agent_MicroRtpsAgent_start)},
        {"stop", "()Z", reinterpret_cast<void*>(Java_es_netdron_micrortps_1agent_MicroRtpsAgent_stop)},
    };
    auto rc = env->RegisterNatives(klass, methods, sizeof(methods) / sizeof(JNINativeMethod));
    if (rc != JNI_OK) return rc;

    return JNI_VERSION_1_6;
}
