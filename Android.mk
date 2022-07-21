LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES := \
    src/RtpsTopics.cpp \
    src/collision_constraints.cpp \
    src/collision_constraintsPubSubTypes.cpp \
    src/collision_constraints_Publisher.cpp \
    src/debug_array.cpp \
    src/debug_arrayPubSubTypes.cpp \
    src/debug_array_Subscriber.cpp \
    src/debug_key_value.cpp \
    src/debug_key_valuePubSubTypes.cpp \
    src/debug_key_value_Subscriber.cpp \
    src/debug_value.cpp \
    src/debug_valuePubSubTypes.cpp \
    src/debug_value_Subscriber.cpp \
    src/debug_vect.cpp \
    src/debug_vectPubSubTypes.cpp \
    src/debug_vect_Subscriber.cpp \
    src/microRTPS_timesync.cpp \
    src/microRTPS_transport.cpp \
    src/offboard_control_mode.cpp \
    src/offboard_control_modePubSubTypes.cpp \
    src/offboard_control_mode_Subscriber.cpp \
    src/onboard_computer_status.cpp \
    src/onboard_computer_statusPubSubTypes.cpp \
    src/onboard_computer_status_Subscriber.cpp \
    src/optical_flow.cpp \
    src/optical_flowPubSubTypes.cpp \
    src/optical_flow_Subscriber.cpp \
    src/position_setpoint.cpp \
    src/position_setpointPubSubTypes.cpp \
    src/position_setpoint_Subscriber.cpp \
    src/position_setpoint_triplet.cpp \
    src/position_setpoint_tripletPubSubTypes.cpp \
    src/position_setpoint_triplet_Subscriber.cpp \
    src/sensor_combined.cpp \
    src/sensor_combinedPubSubTypes.cpp \
    src/sensor_combined_Publisher.cpp \
    src/telemetry_status.cpp \
    src/telemetry_statusPubSubTypes.cpp \
    src/telemetry_status_Subscriber.cpp \
    src/timesync.cpp \
    src/timesyncPubSubTypes.cpp \
    src/timesync_Publisher.cpp \
    src/timesync_Subscriber.cpp \
    src/timesync_status.cpp \
    src/timesync_statusPubSubTypes.cpp \
    src/timesync_status_Publisher.cpp \
    src/trajectory_bezier.cpp \
    src/trajectory_bezierPubSubTypes.cpp \
    src/trajectory_bezier_Subscriber.cpp \
    src/trajectory_setpoint.cpp \
    src/trajectory_setpointPubSubTypes.cpp \
    src/trajectory_setpoint_Subscriber.cpp \
    src/trajectory_waypoint.cpp \
    src/trajectory_waypointPubSubTypes.cpp \
    src/trajectory_waypoint_Publisher.cpp \
    src/vehicle_command.cpp \
    src/vehicle_commandPubSubTypes.cpp \
    src/vehicle_command_Subscriber.cpp \
    src/vehicle_control_mode.cpp \
    src/vehicle_control_modePubSubTypes.cpp \
    src/vehicle_control_mode_Publisher.cpp \
    src/vehicle_local_position_setpoint.cpp \
    src/vehicle_local_position_setpointPubSubTypes.cpp \
    src/vehicle_local_position_setpoint_Subscriber.cpp \
    src/vehicle_mocap_odometry.cpp \
    src/vehicle_mocap_odometryPubSubTypes.cpp \
    src/vehicle_mocap_odometry_Subscriber.cpp \
    src/vehicle_odometry.cpp \
    src/vehicle_odometryPubSubTypes.cpp \
    src/vehicle_odometry_Publisher.cpp \
    src/vehicle_status.cpp \
    src/vehicle_statusPubSubTypes.cpp \
    src/vehicle_status_Publisher.cpp \
    src/vehicle_trajectory_bezier.cpp \
    src/vehicle_trajectory_bezierPubSubTypes.cpp \
    src/vehicle_trajectory_bezier_Subscriber.cpp \
    src/vehicle_trajectory_waypoint.cpp \
    src/vehicle_trajectory_waypointPubSubTypes.cpp \
    src/vehicle_trajectory_waypoint_Subscriber.cpp \
    src/vehicle_trajectory_waypoint_desired.cpp \
    src/vehicle_trajectory_waypoint_desiredPubSubTypes.cpp \
    src/vehicle_trajectory_waypoint_desired_Publisher.cpp \
    src/vehicle_visual_odometry.cpp \
    src/vehicle_visual_odometryPubSubTypes.cpp \
    src/vehicle_visual_odometry_Subscriber.cpp \

LOCAL_SRC_FILES += \
    src/USBSerial_node.cpp \
    src/MicroRTPSAgent.cpp \
    src/micrortps_agent-jni.cpp \

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/../utils/include

LOCAL_CFLAGS := \
    -fvisibility=hidden

LOCAL_CPPFLAGS := \
    -fvisibility=hidden \
    -DANDROID \
    -Wno-unused-but-set-variable

LOCAL_CPP_FEATURES := rtti exceptions

LOCAL_LDLIBS := -llog

LOCAL_SHARED_LIBRARIES := \
	usb_serial \
    fastcdr \
    fastrtps \

LOCAL_ARM_MODE := arm

LOCAL_MODULE := micrortps_agent

include $(BUILD_SHARED_LIBRARY)