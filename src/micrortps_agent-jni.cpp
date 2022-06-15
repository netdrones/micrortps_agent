#include <jni.h>
#include <memory>
#include <string>
#include "MicroRTPSAgent.h"

static std::unique_ptr<netdrones::axon::MicroRTPSAgent> agent(nullptr);

extern "C"
void Java_es_netdron_axon_MicroRtpsAgent_nativeInitUDP(
    JNIEnv*,
    jobject
) {
    if (!agent) {
        agent = std::make_unique<netdrones::axon::MicroRTPSAgent>();
    }
}

extern "C"
void Java_es_netdron_axon_MicroRtpsAgent_nativeInitUART(
    JNIEnv* env,
    jobject ,
    jstring device
) {
    if (!agent) {
        auto pDevice = env->GetStringUTFChars(device, nullptr);
        auto dev = std::string(pDevice);
        agent = std::make_unique<netdrones::axon::MicroRTPSAgent>(dev);
        env->ReleaseStringUTFChars(device, pDevice);
    }
}

extern "C"
void Java_es_netdron_axon_MicroRtpsAgent_setUARTDevice(
    JNIEnv* env,
    jobject,
    jstring device
) {
    if (!agent) return;

    auto pDevice = env->GetStringUTFChars(device, nullptr);
    auto dev = std::string(pDevice);
    agent->setUARTDevice(dev);
    env->ReleaseStringUTFChars(device, pDevice);
}

extern "C"
void Java_es_netdron_axon_MicroRtpsAgent_setBaudrate(
    JNIEnv* ,
    jobject ,
    jint baudrate
) {
    if (!agent) return;
    agent->setBaudrate(static_cast<int>(baudrate));
}

extern "C"
void Java_es_netdron_axon_MicroRtpsAgent_setNamespace(
    JNIEnv* env,
    jobject,
    jstring namespace_
) {
    if (!agent) return;

    auto pNamespace = env->GetStringUTFChars(namespace_, nullptr);
    auto ns = std::string(pNamespace);
    agent->setNamespace(ns);
    env->ReleaseStringUTFChars(namespace_, pNamespace);
}

extern "C"
void Java_es_netdron_axon_MicroRtpsAgent_setPollInterval(
    JNIEnv*,
    jobject,
    jlong interval
) {
    if (!agent) return;
    agent->setPollInterval(static_cast<int>(interval));
}

extern "C"
void Java_es_netdron_axon_MicroRtpsAgent_setUDPRecvPort(
    JNIEnv* ,
    jobject ,
    jint port
) {
    if (!agent) return;
    agent->setRecvPort(port);
}

extern "C"
void Java_es_netdron_axon_MicroRtpsAgent_setUDPSendPort(
    JNIEnv* ,
    jobject ,
    jint port
) {
    if (!agent) return;
    agent->setSendPort(port);
}

extern "C"
jboolean Java_es_netdron_axon_MicroRtpsAgent_start(
    JNIEnv* ,
    jobject
) {
    if (!agent) return false;
    return agent->start();
}

extern "C"
jboolean Java_es_netdron_axon_MicroRtpsAgent_stop(
    JNIEnv* ,
    jobject
) {
    if (!agent) return false;
    return agent->stop();
}