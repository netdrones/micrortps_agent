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

# FIXME(rakuto): ROS_BRIDGE flag workarounds a problem that Fast-DDS discovery not working on Android
# that causes a Fast-DDS message cannot be received on a ROS node. With this flag, micrortps_agent
# works as ROS node and publish and subscribe a message as ROS node.
LOCAL_CPPFLAGS := \
    -fvisibility=hidden \
    -Wno-unused-but-set-variable \
    -DROS_BRIDGE \

LOCAL_CPP_FEATURES := rtti exceptions

LOCAL_LDLIBS := -llog

LOCAL_SHARED_LIBRARIES := \
	usb_serial \
    fastcdr \
    fastrtps \

# ROS2 dependencies
LOCAL_SHARED_LIBRARIES += \
    ament_index_cpp \
    builtin_interfaces__rosidl_generator_c \
    builtin_interfaces__rosidl_typesupport_c \
    builtin_interfaces__rosidl_typesupport_cpp \
    builtin_interfaces__rosidl_typesupport_fastrtps_c \
    builtin_interfaces__rosidl_typesupport_fastrtps_cpp \
    builtin_interfaces__rosidl_typesupport_introspection_c \
    builtin_interfaces__rosidl_typesupport_introspection_cpp \
    fastcdr \
    fastrtps \
    geometry_msgs__rosidl_generator_c \
    geometry_msgs__rosidl_typesupport_c \
    geometry_msgs__rosidl_typesupport_cpp \
    geometry_msgs__rosidl_typesupport_fastrtps_c \
    geometry_msgs__rosidl_typesupport_fastrtps_cpp \
    geometry_msgs__rosidl_typesupport_introspection_c \
    geometry_msgs__rosidl_typesupport_introspection_cpp \
    libstatistics_collector \
    rcl \
    rcl_interfaces__rosidl_generator_c \
    rcl_interfaces__rosidl_typesupport_c \
    rcl_interfaces__rosidl_typesupport_cpp \
    rcl_interfaces__rosidl_typesupport_fastrtps_c \
    rcl_interfaces__rosidl_typesupport_fastrtps_cpp \
    rcl_interfaces__rosidl_typesupport_introspection_c \
    rcl_interfaces__rosidl_typesupport_introspection_cpp \
    rcl_logging_interface \
    rcl_logging_spdlog \
    rcl_yaml_param_parser \
    rclcpp \
    rcutils \
    rcpputils \
    rmw \
    rmw_fastrtps_cpp \
    rmw_fastrtps_shared_cpp \
    rmw_implementation \
    rmw_dds_common \
    rmw_dds_common__rosidl_generator_c \
    rmw_dds_common__rosidl_typesupport_c \
    rmw_dds_common__rosidl_typesupport_cpp \
    rmw_dds_common__rosidl_typesupport_fastrtps_c \
    rmw_dds_common__rosidl_typesupport_fastrtps_cpp \
    rmw_dds_common__rosidl_typesupport_introspection_c \
    rmw_dds_common__rosidl_typesupport_introspection_cpp \
    rosgraph_msgs__rosidl_generator_c \
    rosgraph_msgs__rosidl_typesupport_c \
    rosgraph_msgs__rosidl_typesupport_cpp \
    rosgraph_msgs__rosidl_typesupport_introspection_c \
    rosgraph_msgs__rosidl_typesupport_introspection_cpp \
    rosidl_runtime_c \
    rosidl_typesupport_c \
    rosidl_typesupport_cpp \
    rosidl_typesupport_fastrtps_c \
    rosidl_typesupport_fastrtps_cpp \
    rosidl_typesupport_introspection_c \
    rosidl_typesupport_introspection_cpp \
    spdlog \
    statistics_msgs__rosidl_generator_c \
    statistics_msgs__rosidl_typesupport_c \
    statistics_msgs__rosidl_typesupport_cpp \
    statistics_msgs__rosidl_typesupport_introspection_c \
    statistics_msgs__rosidl_typesupport_introspection_cpp \
    std_msgs__rosidl_generator_c \
    std_msgs__rosidl_typesupport_c \
    std_msgs__rosidl_typesupport_cpp \
    std_msgs__rosidl_typesupport_fastrtps_c \
    std_msgs__rosidl_typesupport_fastrtps_cpp \
    std_msgs__rosidl_typesupport_introspection_c \
    std_msgs__rosidl_typesupport_introspection_cpp \
    sensor_msgs__rosidl_generator_c \
    sensor_msgs__rosidl_typesupport_c \
    sensor_msgs__rosidl_typesupport_cpp \
    sensor_msgs__rosidl_typesupport_fastrtps_c \
    sensor_msgs__rosidl_typesupport_fastrtps_cpp \
    sensor_msgs__rosidl_typesupport_introspection_c \
    sensor_msgs__rosidl_typesupport_introspection_cpp \
    tracetools \
    yaml \

# px4_msgs
LOCAL_SHARED_LIBRARIES += \
	px4_msgs__rosidl_generator_c \
	px4_msgs__rosidl_typesupport_c \
	px4_msgs__rosidl_typesupport_cpp \
	px4_msgs__rosidl_typesupport_fastrtps_c \
	px4_msgs__rosidl_typesupport_fastrtps_cpp \
	px4_msgs__rosidl_typesupport_introspection_c \
	px4_msgs__rosidl_typesupport_introspection_cpp \
	px4_msgs__rosidl_typesupport_introspection_cpp \

LOCAL_ARM_MODE := arm

LOCAL_MODULE := micrortps_agent

include $(BUILD_SHARED_LIBRARY)