/****************************************************************************
 *
 * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 * Copyright (c) 2018-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "RtpsTopics.h"
#include "logging-android.h"

#ifdef ROS_BRIDGE

#include <chrono>
using namespace std::chrono_literals;

//using DebugArray = px4_msgs::msg::DebugArray;
//using DebugKeyValue = px4_msgs::msg::DebugKeyValue;
//using DebugValue = px4_msgs::msg::DebugValue;
//using DebugVect = px4_msgs::msg::DebugVect;
using OffboardControlMode = px4_msgs::msg::OffboardControlMode;
//using OpticalFlow = px4_msgs::msg::OpticalFlow;
using PositionSetpoint = px4_msgs::msg::PositionSetpoint;
using PositionSetpointTriplet = px4_msgs::msg::PositionSetpointTriplet;
using TelemetryStatus = px4_msgs::msg::TelemetryStatus;
using VehicleCommand = px4_msgs::msg::VehicleCommand;
using VehicleLocalPositionSetpoint = px4_msgs::msg::VehicleLocalPositionSetpoint;
using TrajectorySetpoint = px4_msgs::msg::TrajectorySetpoint;
using VehicleTrajectoryWaypoint = px4_msgs::msg::VehicleTrajectoryWaypoint;
using OnboardComputerStatus = px4_msgs::msg::OnboardComputerStatus;
using TrajectoryBezier = px4_msgs::msg::TrajectoryBezier;
using VehicleTrajectoryBezier = px4_msgs::msg::VehicleTrajectoryBezier;
using VehicleMocapOdometry = px4_msgs::msg::VehicleMocapOdometry;
using VehicleOdometry = px4_msgs::msg::VehicleOdometry;
using VehicleVisualOdometry = px4_msgs::msg::VehicleVisualOdometry;
using Timesync = px4_msgs::msg::Timesync;
using TrajectoryWaypoint = px4_msgs::msg::TrajectoryWaypoint;
using VehicleControlMode = px4_msgs::msg::VehicleControlMode;
using VehicleOdometry = px4_msgs::msg::VehicleOdometry;
using VehicleStatus = px4_msgs::msg::VehicleStatus;
using CollisionConstraints = px4_msgs::msg::CollisionConstraints;
using TimesyncStatus = px4_msgs::msg::TimesyncStatus;
using SensorCombined = px4_msgs::msg::SensorCombined;
using VehicleTrajectoryWaypointDesired = px4_msgs::msg::VehicleTrajectoryWaypointDesired;

RtpsTopics::RtpsTopics()
: Node("micrortps_agent",
	   rclcpp::NodeOptions()
	   	.use_intra_process_comms(true)
		.enable_topic_statistics(false)
		.enable_rosout(false)) {
}

#endif // ROS_BRIDGE

bool RtpsTopics::init(std::condition_variable *t_send_queue_cv, std::mutex *t_send_queue_mutex,
		      std::queue<uint8_t> *t_send_queue, const std::string &ns)
{
	// Initialise subscribers
#ifdef __ANDROID__
	LOGD("---   Subscribers   ---");
#else
	std::cout << "\033[0;36m---   Subscribers   ---\033[0m" << std::endl;
#endif // ANDROID

#ifdef ROS_BRIDGE
	RCL_UNUSED(t_send_queue_mutex);
	RCL_UNUSED(t_send_queue_cv);
	RCL_UNUSED(t_send_queue);

	auto sub_opt = rclcpp::SubscriptionOptions();
	sub_opt.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

#if 0
	LOGD("- debug_array subscriber started");
	debug_array_sub_ = this->create_subscription<DebugArray>(
		ns + "fmu/debug_array/in",
		10,
		[this](DebugArray::UniquePtr msg) {
			std::unique_lock lk(this->mtx_debug_array_);
			this->cv_debug_array_.wait(lk, [this] { return !this->debug_array_.get(); } );
			this->debug_array_ = std::move(msg);
		},
		sub_opt
	);
	RCL_UNUSED(debug_array_sub_);

	LOGD("- debug_key_value subscriber started");
	debug_key_value_sub_ = this->create_subscription<DebugKeyValue>(
		ns + "fmu/debug_key_value/in",
		10,
		[this](DebugKeyValue::UniquePtr msg) {
			std::unique_lock lk(this->mtx_debug_key_value_);
			this->cv_debug_key_value_.wait(lk, [this] { return !this->debug_key_value_.get(); });
			this->debug_key_value_ = std::move(msg);
		},
		sub_opt
	);
	RCL_UNUSED(debug_key_value_sub_);

	LOGD("- debug_value subscriber started");
	debug_value_sub_ = this->create_subscription<DebugValue>(
		ns + "fmu/debug_value/in",
		10,
		[this](DebugValue::UniquePtr msg) {
			std::unique_lock lk(this->mtx_debug_value_);
			this->cv_debug_value_.wait(lk, [this] { return !this->debug_value_.get(); });
			this->debug_value_ = std::move(msg);
		},
		sub_opt
	);
	RCL_UNUSED(debug_value_sub_);

	LOGD("- debug_vect subscriber started");
	debug_vect_sub_ = this->create_subscription<DebugVect>(
		ns + "fmu/debug_vect/in",
		10,
		[this](DebugVect::UniquePtr msg) {
			std::unique_lock lk(this->mtx_debug_vect_);
			this->cv_debug_vect_.wait(lk, [this] { return !this->debug_vect_.get(); });
			this->debug_vect_ = std::move(msg);
		},
		sub_opt
	);
	RCL_UNUSED(debug_vect_sub_);
#endif // if 0

#if 0
	LOGD("- offboard_control_mode subscriber started");
	offboard_control_mode_sub_ = this->create_subscription<OffboardControlMode>(
		ns + "fmu/offboard_control_mode/in",
		10,
		[this](OffboardControlMode::UniquePtr msg) {
			std::unique_lock lk(this->mtx_offboard_control_mode_);
			this->cv_offboard_control_mode_.wait(lk, [this] { return !this->offboard_control_mode_.get(); });
			this->offboard_control_mode_ = std::move(msg);
		},
		sub_opt
	);
	RCL_UNUSED(offboard_control_mode_sub_);
#endif

#if 0
	LOGD("- optical_flow subscriber started");
	optical_flow_sub_ = this->create_subscription<OpticalFlow>(
		ns + "fmu/optical_flow/in",
		10,
		[this](OpticalFlow::UniquePtr msg) {
			std::unique_lock lk(this->mtx_optical_flow_);
			this->cv_optical_flow_.wait(lk, [this] { return !this->optical_flow_.get(); });
			this->optical_flow_ = std::move(msg);
		},
		sub_opt
	);
	RCL_UNUSED(optical_flow_sub_);

	auto sub2_opt = rclcpp::SubscriptionOptions();
	sub2_opt.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	LOGD("- position_setpoint subscriber started");
	position_setpoint_sub_ = this->create_subscription<PositionSetpoint>(
		ns + "fmu/position_setpoint/in",
		10,
		[this](PositionSetpoint::UniquePtr msg) {
			std::unique_lock lk(this->mtx_position_setpoint_);
			this->cv_position_setpoint_.wait(lk, [this] { return !this->position_setpoint_.get(); });
			this->position_setpoint_ = std::move(msg);
		},
		sub2_opt
	);
	RCL_UNUSED(position_setpoint_sub_);

	auto sub3_opt = rclcpp::SubscriptionOptions();
	sub3_opt.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	LOGD("- position_setpoint_triplet subscriber started");
	position_setpoint_triplet_sub_ = this->create_subscription<PositionSetpointTriplet>(
		ns + "fmu/position_setpoint_triplet/in",
		10,
		[this](PositionSetpointTriplet::UniquePtr msg) {
			std::unique_lock lk(this->mtx_position_setpoint_triplet_);
			this->cv_position_setpoint_triplet_.wait(lk, [this] { return !this->position_setpoint_triplet_.get(); });
			this->position_setpoint_triplet_ = std::move(msg);
		},
		sub3_opt
	);
	RCL_UNUSED(position_setpoint_triplet_sub_);

	auto sub4_opt = rclcpp::SubscriptionOptions();
	sub4_opt.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	LOGD("- telemetry_status subscriber started");
	telemetry_status_sub_ = this->create_subscription<TelemetryStatus>(
		ns + "fmu/telemetry_status/in",
		10,
		[this](TelemetryStatus::UniquePtr msg) {
			std::unique_lock lk(this->mtx_telemetry_status_);
			this->cv_telemetry_status_.wait(lk, [this] { return !this->telemetry_status_.get(); });
			this->telemetry_status_ = std::move(msg);
		},
		sub4_opt
	);
	RCL_UNUSED(telemetry_status_sub_);
#endif

	// TimeSync is never sent over Fast-RTPS.
#if 0
	LOGD("- timesync subscriber started");
	timesync_sub_ = this->create_subscription<Timesync>(
		ns + "fmu/timesync/in",
		10,
		[this](Timesync::UniquePtr msg) {
			std::unique_lock lk(this->mtx_timesync_);
			this->cv_timesync_.wait(lk, [this] { return !this->timesync_.get(); });
			this->timesync_ = std::move(msg);
		},
		sub_opt
	);
	RCL_UNUSED(timesync_sub_);

	auto sub5_opt = rclcpp::SubscriptionOptions();
	sub5_opt.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	LOGD("- vehicle_command subscriber started");
	vehicle_command_sub_ = this->create_subscription<VehicleCommand>(
		ns + "fmu/vehicle_command/in",
		10,
		[this](VehicleCommand::UniquePtr msg) {
			std::unique_lock lk(this->mtx_vehicle_command_);
			this->cv_vehicle_command_.wait(lk, [this] { return !this->vehicle_command_.get(); });
			this->vehicle_command_ = std::move(msg);
		},
		sub5_opt
	);
	RCL_UNUSED(vehicle_command_sub_);
#endif

	auto sub6_opt = rclcpp::SubscriptionOptions();
	sub6_opt.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	LOGD("- vehicle_local_position_setpoint subscriber started");
	vehicle_local_position_setpoint_sub_ = this->create_subscription<VehicleLocalPositionSetpoint>(
		ns + "fmu/vehicle_local_position_setpoint/in",
		10,
		[this](VehicleLocalPositionSetpoint::UniquePtr msg) {
			std::unique_lock lk(this->mtx_vehicle_local_position_setpoint_);
			this->cv_vehicle_local_position_setpoint_.wait(lk, [this] { return !this->vehicle_local_position_setpoint_.get(); });
			this->vehicle_local_position_setpoint_ = std::move(msg);
		},
		sub6_opt
	);
	RCL_UNUSED(vehicle_local_position_setpoint_sub_);

	auto sub7_opt = rclcpp::SubscriptionOptions();
	sub7_opt.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	LOGD("- trajectory_setpoint subscriber started");
	trajectory_setpoint_sub_ = this->create_subscription<TrajectorySetpoint>(
		ns + "fmu/trajectory_setpoint/in",
		10,
		[this](TrajectorySetpoint::UniquePtr msg) {
			std::unique_lock lk(this->mtx_trajectory_setpoint_);
			this->cv_trajectory_setpoint_.wait(lk, [this] { return !this->trajectory_setpoint_.get(); });
			this->trajectory_setpoint_ = std::move(msg);
		},
		sub7_opt
	);
	RCL_UNUSED(trajectory_setpoint_sub_);

	auto sub8_opt = rclcpp::SubscriptionOptions();
	sub8_opt.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	LOGD("- vehicle_trajectory_waypoint subscriber started");
	vehicle_trajectory_waypoint_sub_ = this->create_subscription<VehicleTrajectoryWaypoint>(
		ns + "fmu/vehicle_trajectory_waypoint/in",
		10,
		[this](VehicleTrajectoryWaypoint::UniquePtr msg) {
			std::unique_lock lk(this->mtx_vehicle_trajectory_waypoint_);
			this->cv_vehicle_trajectory_waypoint_.wait(lk, [this] { return !this->vehicle_trajectory_waypoint_.get(); });
			this->vehicle_trajectory_waypoint_ = std::move(msg);
		},
		sub8_opt
	);
	RCL_UNUSED(vehicle_trajectory_waypoint_sub_);

	auto sub9_opt = rclcpp::SubscriptionOptions();
	sub9_opt.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	LOGD("- onboard_computer_status subscriber started");
	onboard_computer_status_sub_ = this->create_subscription<OnboardComputerStatus>(
		ns + "fmu/onboard_computer_status/in",
		10,
		[this](OnboardComputerStatus::UniquePtr msg) {
			std::unique_lock lk(this->mtx_onboard_computer_status_);
			this->cv_onboard_computer_status_.wait(lk, [this] { return !this->onboard_computer_status_.get(); });
			this->onboard_computer_status_ = std::move(msg);
		},
		sub9_opt
	);
	RCL_UNUSED(onboard_computer_status_sub_);

	auto sub10_opt = rclcpp::SubscriptionOptions();
	sub10_opt.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	LOGD("- trajectory_bezier subscriber started");
	trajectory_bezier_sub_ = this->create_subscription<TrajectoryBezier>(
		ns + "fmu/trajectory_bezier/in",
		10,
		[this](TrajectoryBezier::UniquePtr msg) {
			std::unique_lock lk(this->mtx_trajectory_bezier_);
			this->cv_trajectory_bezier_.wait(lk, [this] { return !this->trajectory_bezier_.get(); });
			this->trajectory_bezier_ = std::move(msg);
		},
		sub10_opt
	);
	RCL_UNUSED(trajectory_bezier_sub_);

	auto sub11_opt = rclcpp::SubscriptionOptions();
	sub11_opt.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	LOGD("- vehicle_trajectory_bezier subscriber started");
	vehicle_trajectory_bezier_sub_ = this->create_subscription<VehicleTrajectoryBezier>(
		ns + "fmu/vehicle_trajectory_bezier/in",
		10,
		[this](VehicleTrajectoryBezier::UniquePtr msg) {
			std::unique_lock lk(this->mtx_vehicle_trajectory_bezier_);
			this->cv_vehicle_trajectory_bezier_.wait(lk, [this] { return !this->vehicle_trajectory_bezier_.get(); });
			this->vehicle_trajectory_bezier_ = std::move(msg);
		},
		sub11_opt
	);
	RCL_UNUSED(vehicle_trajectory_bezier_sub_);

#if 0
	LOGD("- vehicle_mocap_odometry subscriber started");
	vehicle_mocap_odometry_sub_ = this->create_subscription<VehicleMocapOdometry>(
		ns + "fmu/vehicle_mocap_odometry/in",
		10,
		[this](VehicleMocapOdometry::UniquePtr msg) {
			std::unique_lock lk(this->mtx_vehicle_mocap_odometry_);
			this->cv_vehicle_mocap_odometry_.wait(lk, [this] { return !this->vehicle_mocap_odometry_.get(); });
			this->vehicle_mocap_odometry_ = std::move(msg);
		},
		sub_opt
	);
	RCL_UNUSED(vehicle_mocap_odometry_sub_);
#endif

	auto sub12_opt = rclcpp::SubscriptionOptions();
	sub12_opt.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	LOGD("- vehicle_visual_odometry subscriber started");
	vehicle_visual_odometry_sub_ = this->create_subscription<VehicleVisualOdometry>(
		ns + "fmu/vehicle_visual_odometry/in",
		10,
		[this](VehicleVisualOdometry::UniquePtr msg) {
			std::unique_lock lk(this->mtx_vehicle_visual_odometry_);
			this->cv_vehicle_visual_odometry_.wait(lk, [this] { return !this->vehicle_visual_odometry_.get(); });
			this->vehicle_visual_odometry_ = std::move(msg);
		},
		sub12_opt
	);
	RCL_UNUSED(vehicle_visual_odometry_sub_);

#else
	if (_debug_array_sub.init(1, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		LOGD("- debug_array subscriber started");
	} else {
		LOGE("Failed starting debug_array subscriber");
		return false;
	}

	if (_debug_key_value_sub.init(2, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		LOGD("- debug_key_value subscriber started");
	} else {
		LOGE("Failed starting debug_key_value subscriber");
		return false;
	}


	if (_debug_value_sub.init(3, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		LOGD("- debug_value subscriber started");
	} else {
		LOGE("Failed starting debug_value subscriber");
		return false;
	}


	if (_debug_vect_sub.init(4, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		LOGD("- debug_vect subscriber started");
	} else {
		LOGD("Failed starting debug_vect subscriber");
		return false;
	}


	if (_offboard_control_mode_sub.init(5, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		LOGD("- offboard_control_mode subscriber started");
	} else {
		LOGE("Failed starting offboard_control_mode subscriber");
		return false;
	}


	if (_optical_flow_sub.init(6, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		LOGD("- optical_flow subscriber started");
	} else {
		LOGE("Failed starting optical_flow subscriber");
		return false;
	}


	if (_position_setpoint_sub.init(7, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		LOGD("- position_setpoint subscriber started");
	} else {
		LOGE("Failed starting position_setpoint subscriber");
		return false;
	}


	if (_position_setpoint_triplet_sub.init(8, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		LOGD("- position_setpoint_triplet subscriber started");
	} else {
		LOGE("Failed starting position_setpoint_triplet subscriber");
		return false;
	}


	if (_telemetry_status_sub.init(9, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		LOGD("- telemetry_status subscriber started");
	} else {
		LOGE("Failed starting telemetry_status subscriber");
		return false;
	}


#if 0
	if (_timesync_sub.init(10, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		LOGD("- timesync subscriber started");
	} else {
		LOGE("Failed starting timesync subscriber");
		return false;
	}
#endif

	if (_vehicle_command_sub.init(12, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		LOGD("- vehicle_command subscriber started");
	} else {
		LOGE("Failed starting vehicle_command subscriber");
		return false;
	}


	if (_vehicle_local_position_setpoint_sub.init(14, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		LOGD("- vehicle_local_position_setpoint subscriber started");
	} else {
		LOGE("Failed starting vehicle_local_position_setpoint subscriber");
		return false;
	}


	if (_trajectory_setpoint_sub.init(15, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		LOGD("- trajectory_setpoint subscriber started");
	} else {
		LOGE("Failed starting trajectory_setpoint subscriber");
		return false;
	}


	if (_vehicle_trajectory_waypoint_sub.init(20, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		LOGD("- vehicle_trajectory_waypoint subscriber started");
	} else {
		LOGE("Failed starting vehicle_trajectory_waypoint subscriber");
		return false;
	}


	if (_onboard_computer_status_sub.init(23, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		LOGD("- onboard_computer_status subscriber started");
	} else {
		LOGE("Failed starting onboard_computer_status subscriber");
		return false;
	}


	if (_trajectory_bezier_sub.init(24, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		LOGD("- trajectory_bezier subscriber started");
	} else {
		LOGE("Failed starting trajectory_bezier subscriber");
		return false;
	}


	if (_vehicle_trajectory_bezier_sub.init(25, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		LOGD("- vehicle_trajectory_bezier subscriber started");
	} else {
		LOGE("Failed starting vehicle_trajectory_bezier subscriber");
		return false;
	}


	if (_vehicle_mocap_odometry_sub.init(17, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		LOGD("- vehicle_mocap_odometry subscriber started");
	} else {
		LOGE("Failed starting vehicle_mocap_odometry subscriber");
		return false;
	}


	if (_vehicle_visual_odometry_sub.init(18, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		LOGD("- vehicle_visual_odometry subscriber started");
	} else {
		LOGE("Failed starting vehicle_visual_odometry subscriber");
		return false;
	}
#endif // ROS_BRIDGE

#ifdef __ANDROID__
	LOGD("-----------------------");
	LOGD("----   Publishers  ----");
#else
	std::cout << "\033[0;36m-----------------------\033[0m" << std::endl << std::endl;

	// Initialise publishers
	std::cout << "\033[0;36m----   Publishers  ----\033[0m" << std::endl;
#endif // ANDROID

#ifdef ROS_BRIDGE
	// TimeSync is never sent over Fast-RTPS.
#if 0
	// See also microRTPS_timersync.cpp
	timesync_timer_ = this->create_wall_timer(100ms, [this]{
		auto timesync = this->_timesync->newTimesyncMsg();

		Timesync msg;
		msg.timestamp = timesync.timestamp_();
		msg.seq = timesync.seq_();
		msg.tc1 = timesync.tc1_();
		msg.ts1 = timesync.ts1_();

		this->timesync_fmu_in_pub_->publish(msg);
	});
	RCL_UNUSED(timesync_timer_);

	LOGD("- timesync publishers started");
	timesync_pub_ = this->create_publisher<Timesync>(
		ns + "fmu/timesync/out",
		1
	);
	timesync_fmu_in_pub_ = this->create_publisher<Timesync>(
		ns + "fmu/timesync/in",
		1
	);
#endif

	LOGD("- trajectory_waypoint publisher started");
	trajectory_waypoint_pub_ = this->create_publisher<TrajectoryWaypoint>(
		ns + "fmu/trajectory_waypoint/out",
		10
	);

	LOGD("- vehicle_control_mode publisher started");
	vehicle_control_mode_pub_ = this->create_publisher<VehicleControlMode>(
		ns + "fmu/vehicle_control_mode/out",
		10
	);

	LOGD("- vehicle_odometry publisher started");
	vehicle_odometry_pub_ = this->create_publisher<VehicleOdometry>(
		ns + "fmu/vehicle_odometry/out",
		10
	);

	LOGD("- vehicle_status publisher started");
	vehicle_status_pub_ = this->create_publisher<VehicleStatus>(
		ns + "fmu/vehicle_status/out",
		10
	);

	LOGD("- collision_constraints publisher started");
	collision_constraints_pub_ = this->create_publisher<CollisionConstraints>(
		ns + "fmu/collision_constraints/out",
		10
	);

	LOGD("- timesync_status publisher started");
	timesync_status_pub_ = this->create_publisher<TimesyncStatus>(
		ns + "fmu/timesync_status/out",
		10
	);

	LOGD("- sensor_combined publisher started");
	sensor_combined_pub_ = this->create_publisher<SensorCombined>(
		ns + "fmu/sensor_combined/out",
		10
	);

	LOGD("- vehicle_trajectory_waypoint_desired publisher started");
	vehicle_trajectory_waypoint_desired_pub_ = this->create_publisher<VehicleTrajectoryWaypointDesired>(
		ns + "fmu/vehicle_trajectory_waypoint_desired/out",
		10
	);

#else
	if (_timesync_pub.init(ns)) {
		if (_timesync_fmu_in_pub.init(ns, std::string("fmu/timesync/in"))) {
			_timesync->start(&_timesync_fmu_in_pub);
			LOGD("- timesync publishers started");
		}

	} else {
		LOGE("ERROR starting timesync publisher");
		return false;
	}


	if (_trajectory_waypoint_pub.init(ns)) {
		LOGD("- trajectory_waypoint publisher started");
	} else {
		LOGE("ERROR starting trajectory_waypoint publisher");
		return false;
	}


	if (_vehicle_control_mode_pub.init(ns)) {
		LOGD("- vehicle_control_mode publisher started");
	} else {
		LOGE("ERROR starting vehicle_control_mode publisher");
		return false;
	}


	if (_vehicle_odometry_pub.init(ns)) {
		LOGD("- vehicle_odometry publisher started");
	} else {
		LOGE("ERROR starting vehicle_odometry publisher");
		return false;
	}


	if (_vehicle_status_pub.init(ns)) {
		LOGD("- vehicle_status publisher started");
	} else {
		LOGE("ERROR starting vehicle_status publisher");
		return false;
	}


	if (_collision_constraints_pub.init(ns)) {
		LOGD("- collision_constraints publisher started");
	} else {
		LOGE("ERROR starting collision_constraints publisher");
		return false;
	}


	if (_timesync_status_pub.init(ns, std::string("timesync_status"))) {
		_timesync->init_status_pub(&_timesync_status_pub);
		LOGD("- timesync_status publisher started");
	} else {
		LOGE("ERROR starting timesync_status publisher");
		return false;
	}


	if (_sensor_combined_pub.init(ns)) {
		LOGD("- sensor_combined publisher started");
	} else {
		LOGE("ERROR starting sensor_combined publisher");
		return false;
	}

	if (_vehicle_trajectory_waypoint_desired_pub.init(ns)) {
		LOGD("- vehicle_trajectory_waypoint_desired publisher started");
	} else {
		LOGE("ERROR starting vehicle_trajectory_waypoint_desired publisher");
		return false;
	}
#endif // ROS_BRIDGE

#ifdef __ANDROID__
	LOGD("-----------------------");
#else
	std::cout << "\033[0;36m-----------------------\033[0m" << std::endl;
#endif
	return true;
}

template <typename T>
void RtpsTopics::sync_timestamp_of_incoming_data(T &msg) {
	uint64_t timestamp = getMsgTimestamp(&msg);
	uint64_t timestamp_sample = getMsgTimestampSample(&msg);
	_timesync->subtractOffset(timestamp);
	setMsgTimestamp(&msg, timestamp);
	_timesync->subtractOffset(timestamp_sample);
	setMsgTimestampSample(&msg, timestamp_sample);
}

void RtpsTopics::publish(const uint8_t topic_ID, char data_buffer[], size_t len)
{
	switch (topic_ID) {

	case 10: { // timesync publisher
		timesync_msg_t st;
		eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
		eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
		st.deserialize(cdr_des);
#ifdef ROS_BRIDGE
		// TODO: processTimesyncMsg
		_timesync->processTimesyncMsg(&st, timesync_pub_);
#else
		_timesync->processTimesyncMsg(&st, &_timesync_pub);
#endif // ROS_BRIDGE

		// apply timestamp offset
		sync_timestamp_of_incoming_data(st);

#ifdef ROS_BRIDGE
		Timesync msg;
		msg.timestamp = st.timestamp_();
		msg.seq = st.seq_();
		msg.tc1 = st.tc1_();
		msg.ts1 = st.ts1_();
		timesync_pub_->publish(msg);
#else
		_timesync_pub.publish(&st);
#endif // ROS_BRIDGE
	}
	break;

	case 11: { // trajectory_waypoint publisher
		trajectory_waypoint_msg_t st;
		eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
		eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
		st.deserialize(cdr_des);

		// apply timestamp offset
		sync_timestamp_of_incoming_data(st);

#ifdef ROS_BRIDGE
		TrajectoryWaypoint msg;
		msg.timestamp = st.timestamp_();
		msg.position = st.position();
		msg.velocity = st.velocity();
		msg.yaw = st.yaw_();
		msg.yaw_speed = st.yaw_speed_();
		msg.point_valid = st.point_valid_();
		msg.type = st.type_();

		trajectory_waypoint_pub_->publish(msg);
#else
		_trajectory_waypoint_pub.publish(&st);
#endif // ROS_BRIDGE
	}
	break;

	case 13: { // vehicle_control_mode publisher
		vehicle_control_mode_msg_t st;
		eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
		eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
		st.deserialize(cdr_des);

		// apply timestamp offset
		sync_timestamp_of_incoming_data(st);

#ifdef ROS_BRIDGE
		VehicleControlMode msg;
		msg.timestamp = st.timestamp_();
		msg.flag_armed = st.flag_armed_();
		msg.flag_multicopter_position_control_enabled = st.flag_multicopter_position_control_enabled_();
		msg.flag_control_manual_enabled = st.flag_control_manual_enabled_();
		msg.flag_control_auto_enabled = st.flag_control_manual_enabled_();
		msg.flag_control_offboard_enabled = st.flag_control_offboard_enabled_();
		msg.flag_control_rates_enabled = st.flag_control_rates_enabled_();
		msg.flag_control_attitude_enabled = st.flag_control_attitude_enabled_();
		msg.flag_control_acceleration_enabled = st.flag_control_acceleration_enabled_();
		msg.flag_control_velocity_enabled = st.flag_control_velocity_enabled_();
		msg.flag_control_position_enabled = st.flag_control_position_enabled_();
		msg.flag_control_altitude_enabled = st.flag_control_altitude_enabled_();
		msg.flag_control_climb_rate_enabled = st.flag_control_climb_rate_enabled_();
		msg.flag_control_termination_enabled = st.flag_control_termination_enabled_();

		vehicle_control_mode_pub_->publish(msg);
#else
		_vehicle_control_mode_pub.publish(&st);
#endif // ROS_BRIDGE
	}
	break;

	case 18: { // vehicle_odometry publisher
		vehicle_odometry_msg_t st;
		eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
		eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
		st.deserialize(cdr_des);

		// apply timestamp offset
		sync_timestamp_of_incoming_data(st);

#ifdef ROS_BRIDGE
		VehicleOdometry msg;
		msg.timestamp = st.timestamp_();
		msg.timestamp_sample = st.timestamp_sample_();
		msg.local_frame = st.local_frame_();
		msg.x = st.x_();
		msg.y = st.y_();
		msg.z = st.z_();
		msg.q = st.q();
		msg.q_offset = st.q_offset();
		msg.pose_covariance = st.pose_covariance();
		msg.velocity_frame = st.velocity_frame_();
		msg.vx = st.vx_();
		msg.vy = st.vy_();
		msg.vz = st.vz_();
		msg.rollspeed = st.rollspeed_();
		msg.pitchspeed = st.pitchspeed_();
		msg.yawspeed = st.yawspeed_();
		msg.velocity_covariance = st.velocity_covariance();
		msg.reset_counter = st.reset_counter_();

		vehicle_odometry_pub_->publish(msg);
#else
		_vehicle_odometry_pub.publish(&st);
#endif // ROS_BRIDGE
	}
	break;

	case 21: { // vehicle_status publisher
		// TODO(rakuto): VehicleStatus.msg has been updated recently. Need to regenerate code.
#if 0
		vehicle_status_msg_t st;
		eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
		eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
		st.deserialize(cdr_des);

		// apply timestamp offset
		sync_timestamp_of_incoming_data(st);

#ifdef ROS_BRIDGE
		VehicleStatus msg;
		msg.timestamp = st.timestamp_();
		msg.armed_time = st.armed_time_();
		msg.takeoff_time = st.takeoff_time_();
		msg.arming_state = st.arming_state_();
		msg.latest_arming_reason = st.latest_arming_reason_();
		msg.latest_disarming_reason = st.latest_disarming_reason_();
		msg.nav_state_timestamp = st.nav_state_timestamp_();
		msg.nav_state = st.nav_state_();
		msg.failure_detector_status = st.failure_detector_status_();
		msg.hil_state = st.hil_state_();
		msg.vehicle_type = st.vehicle_type_();
		msg.failsafe = st.failsafe_();
		msg.failsafe_timestamp = st.failsafe_timestamp_();
		msg.rc_signal_lost = st.rc_signal_lost_();
		msg.data_link_lost = st.data_link_lost_();
		msg.data_link_lost_counter = st.data_link_lost_counter_();
		msg.high_latency_data_link_lost = st.high_latency_data_link_lost_();
		msg.is_vtol = st.is_vtol_();
		msg.is_vtol_tailsitter = st.is_vtol_tailsitter_();
		msg.in_transition_mode = st.in_transition_mode_();
		msg.in_transition_to_fw = st.in_transition_to_fw_();
		msg.mission_failure = st.mission_failure_();
		msg.geofence_violated = st.geofence_violated_();
		msg.system_type = st.system_type_();
		msg.system_id = st.system_id_();
		msg.component_id = st.component_id_();
		msg.onboard_control_sensors_present = st.onboard_control_sensors_present_();
		msg.onboard_control_sensors_enabled = st.onboard_control_sensors_enabled_();
		msg.onboard_control_sensors_health = st.onboard_control_sensors_health_();
		msg.safety_button_available = st.safety_button_available_();
		msg.safety_off = st.safety_off_();

		vehicle_status_pub_->publish(msg);
#else
		_vehicle_status_pub.publish(&st);
#endif // ROS_BRIDGE
#endif // if 0
	}
	break;

	case 24: { // collision_constraints publisher
		collision_constraints_msg_t st;
		eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
		eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
		st.deserialize(cdr_des);

		// apply timestamp offset
		sync_timestamp_of_incoming_data(st);

#ifdef ROS_BRIDGE
		CollisionConstraints msg;
		msg.timestamp = st.timestamp_();
		msg.original_setpoint = st.original_setpoint();
		msg.adapted_setpoint = st.adapted_setpoint();

		collision_constraints_pub_->publish(msg);
#else
		_collision_constraints_pub.publish(&st);
#endif // ROS_BRIDGE
	}
	break;

	case 28: { // timesync_status publisher
		timesync_status_msg_t st;
		eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
		eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
		st.deserialize(cdr_des);

		// apply timestamp offset
		sync_timestamp_of_incoming_data(st);

#ifdef ROS_BRIDGE
		TimesyncStatus msg;
		msg.timestamp = st.timestamp_();
		msg.source_protocol = st.source_protocol_();
		msg.remote_timestamp = st.remote_timestamp_();
		msg.observed_offset = st.observed_offset_();
		msg.estimated_offset = st.estimated_offset_();
		msg.round_trip_time = st.round_trip_time_();

		timesync_status_pub_->publish(msg);
#else
		_timesync_status_pub.publish(&st);
#endif // ROS_BRIDGE
	}
	break;

	case 29: { // sensor_combined publisher
		sensor_combined_msg_t st;
		eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
		eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
		st.deserialize(cdr_des);

		// apply timestamp offset
		sync_timestamp_of_incoming_data(st);

#ifdef ROS_BRIDGE
		SensorCombined msg;
		msg.timestamp = st.timestamp_();
		msg.gyro_rad = st.gyro_rad();
		msg.gyro_integral_dt = st.gyro_integral_dt_();
		msg.accelerometer_m_s2 = st.accelerometer_m_s2();
		msg.accelerometer_integral_dt = st.accelerometer_integral_dt_();
		msg.accelerometer_clipping = st.accelerometer_clipping_();
		msg.gyro_clipping = st.gyro_clipping_();
		msg.accel_calibration_count = st.accel_calibration_count_();
		msg.gyro_calibration_count = st.gyro_calibration_count_();

		sensor_combined_pub_->publish(msg);
#else
		_sensor_combined_pub.publish(&st);
#endif // ROS_BRIDGE
	}
	break;

	case 23: { // vehicle_trajectory_waypoint_desired publisher
		vehicle_trajectory_waypoint_desired_msg_t st;
		eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
		eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
		st.deserialize(cdr_des);

		// apply timestamp offset
		sync_timestamp_of_incoming_data(st);

#ifdef ROS_BRIDGE
		VehicleTrajectoryWaypointDesired msg;
		msg.timestamp = st.timestamp_();
		msg.type = st.type_();
		int i = 0;
		for (auto& wp : msg.waypoints) {
			auto st_wp = st.waypoints()[i];
			wp.timestamp = st_wp.timestamp_();
			wp.position = st_wp.position();
			wp.velocity = st_wp.velocity();
			wp.acceleration = st_wp.acceleration();
			wp.yaw = st_wp.yaw_();
			wp.yaw_speed = st_wp.yaw_speed_();
			wp.point_valid = st_wp.point_valid_();
			wp.type = st_wp.type_();
			++i;
		}
		vehicle_trajectory_waypoint_desired_pub_->publish(msg);
#else
		_vehicle_trajectory_waypoint_desired_pub.publish(&st);
#endif // ROS_BRIDGE
	}
	break;

	default:
		LOGW("[   micrortps_agent   ]\tUnexpected topic ID '%hhu' to publish. Please make sure the agent is capable of parsing the message associated to the topic ID '%hhu'",
		       topic_ID, topic_ID);
		break;
	}
}
template <typename T>
void RtpsTopics::sync_timestamp_of_outgoing_data(T &msg) {
	uint64_t timestamp = getMsgTimestamp(&msg);
	uint64_t timestamp_sample = getMsgTimestampSample(&msg);
	_timesync->addOffset(timestamp);
	setMsgTimestamp(&msg, timestamp);
	_timesync->addOffset(timestamp_sample);
	setMsgTimestampSample(&msg, timestamp_sample);
}

bool RtpsTopics::getMsg(const uint8_t topic_ID, eprosima::fastcdr::Cdr &scdr)
{
	bool ret = false;

	LOGD("getMsg: %d", topic_ID);

	switch (topic_ID) {

	case 1: { // debug_array subscriber
#ifdef ROS_BRIDGE
# if 0
		std::unique_lock lk(mtx_debug_array_);
		if (auto m = debug_array_.get()) {
			debug_array_msg_t msg;
            msg.timestamp_(m->timestamp);
			sync_timestamp_of_outgoing_data(msg);
			msg.id_(m->id);
			// FIXME: px4_msgs generated code doesn't compile
//			msg.name(m->name);
			msg.data(m->data);
			debug_array_.reset();
			cv_debug_array_.notify_one();
			msg.serialize(scdr);
			ret = true;
		}
# endif
#else
        if (_debug_array_sub.hasMsg()) {
            debug_array_msg_t msg = _debug_array_sub.getMsg();

            // apply timestamp offset
            sync_timestamp_of_outgoing_data(msg);

            msg.serialize(scdr);
            ret = true;

            _debug_array_sub.unlockMsg();
        }
#endif // ROS_BRIDGE
		break;
	}
	case 2: { // debug_key_value subscriber
#ifdef ROS_BRIDGE
# if 0
		std::unique_lock lk(mtx_debug_key_value_);
		if (auto m = debug_key_value_.get()) {
			debug_key_value_msg_t msg;
            msg.timestamp_(m->timestamp);
			sync_timestamp_of_outgoing_data(msg);
			msg.key(m->key);
			msg.value_(m->value);

			debug_key_value_.reset();
			cv_debug_key_value_.notify_one();
			msg.serialize(scdr);
			ret = true;
		}
# endif
#else
		if (_debug_key_value_sub.hasMsg()) {
			debug_key_value_msg_t msg = _debug_key_value_sub.getMsg();

			// apply timestamp offset
			sync_timestamp_of_outgoing_data(msg);

			msg.serialize(scdr);
			ret = true;

			_debug_key_value_sub.unlockMsg();
		}
#endif // ROS_BRIDGE
			break;
	}
	case 3: { // debug_value subscriber
#ifdef ROS_BRIDGE
# if 0
		std::unique_lock lk(mtx_debug_value_);
		if (auto m = debug_value_.get()) {
			debug_value_msg_t msg;
            msg.timestamp_(m->timestamp);
			sync_timestamp_of_outgoing_data(msg);
			msg.ind_(m->ind);
			msg.value_(m->value);

			debug_value_.reset();
			cv_debug_value_.notify_one();
			msg.serialize(scdr);
			ret = true;
		}
# endif
#else
        if (_debug_value_sub.hasMsg()) {
            debug_value_msg_t msg = _debug_value_sub.getMsg();

            // apply timestamp offset
            sync_timestamp_of_outgoing_data(msg);

            msg.serialize(scdr);
            ret = true;

            _debug_value_sub.unlockMsg();
        }
#endif // ROS_BRIDGE
		break;
	}
	case 4: { // debug_vect subscriber
#ifdef ROS_BRIDGE
# if 0
		std::unique_lock lk(mtx_debug_vect_);
		if (auto m = debug_vect_.get()) {
			debug_vect_msg_t msg;
            msg.timestamp_(m->timestamp);
			sync_timestamp_of_outgoing_data(msg);
			msg.name(m->name);
			msg.x_(m->x);
			msg.y_(m->y);
			msg.z_(m->z);

			debug_vect_.reset();
			cv_debug_vect_.notify_one();
			msg.serialize(scdr);
			ret = true;
		}
# endif
#else
        if (_debug_vect_sub.hasMsg()) {
            debug_vect_msg_t msg = _debug_vect_sub.getMsg();

            // apply timestamp offset
            sync_timestamp_of_outgoing_data(msg);

            msg.serialize(scdr);
            ret = true;

            _debug_vect_sub.unlockMsg();
        }
#endif // ROS_BRIDGE
		break;
	}
	case 5: { // offboard_control_mode subscriber
#ifdef ROS_BRIDGE
		std::unique_lock lk(mtx_offboard_control_mode_);
		if (auto m = offboard_control_mode_.get()) {
			offboard_control_mode_msg_t msg;
            msg.timestamp_(m->timestamp);
			sync_timestamp_of_outgoing_data(msg);
			msg.position_(m->position);
			msg.velocity_(m->velocity);
			msg.acceleration_(m->acceleration);
			msg.attitude_(m->attitude);
			msg.body_rate_(m->body_rate);
			msg.actuator_(m->actuator);

			offboard_control_mode_.reset();
			cv_offboard_control_mode_.notify_one();
			msg.serialize(scdr);
			ret = true;
		}
#else
        if (_offboard_control_mode_sub.hasMsg()) {
            offboard_control_mode_msg_t msg = _offboard_control_mode_sub.getMsg();

            // apply timestamp offset
            sync_timestamp_of_outgoing_data(msg);

            msg.serialize(scdr);
            ret = true;

            _offboard_control_mode_sub.unlockMsg();
        }
#endif // ROS_BRIDGE
		break;
	}
	case 6: { // sensor_optical_flow subscriber
#ifdef ROS_BRIDGE
// TODO: OpticalFlow.msg does not exist in latest px4_msg
# if 0
		std::unique_lock lk(mtx_optical_flow_);
		if (auto m = optical_flow_.get()) {
			optical_flow_msg_t msg;
            msg.timestamp_(m->timestamp);
			sync_timestamp_of_outgoing_data(msg);

//			msg.serialize(scdr);
//			ret = true;
		}
# endif
#else
        if (_optical_flow_sub.hasMsg()) {
            optical_flow_msg_t msg = _optical_flow_sub.getMsg();

            // apply timestamp offset
            sync_timestamp_of_outgoing_data(msg);

            msg.serialize(scdr);
            ret = true;

            _optical_flow_sub.unlockMsg();
        }
#endif // ROS_BRIDGE
		break;
	}
	case 7: { // position_setpoint subscriber
#ifdef ROS_BRIDGE
		std::unique_lock lk(mtx_position_setpoint_);
		if (auto m = position_setpoint_.get()) {
			position_setpoint_msg_t msg;
            msg.timestamp_(m->timestamp);
            copyPositionSetpoint(*m, msg);

			position_setpoint_.reset();
			cv_position_setpoint_.notify_one();
			msg.serialize(scdr);
			ret = true;
		}
#else
        if (_position_setpoint_sub.hasMsg()) {
            position_setpoint_msg_t msg = _position_setpoint_sub.getMsg();

            // apply timestamp offset
            sync_timestamp_of_outgoing_data(msg);

            msg.serialize(scdr);
            ret = true;

            _position_setpoint_sub.unlockMsg();
        }
#endif // ROS_BRIDGE
		break;
	}
	case 8: { // position_setpoint_triplet subscriber
#ifdef ROS_BRIDGE
		std::unique_lock lk(mtx_position_setpoint_triplet_);
		if (auto m = position_setpoint_triplet_.get()) {
			position_setpoint_triplet_msg_t msg;
            msg.timestamp_(m->timestamp);
			sync_timestamp_of_outgoing_data(msg);
            copyPositionSetpoint(m->previous, msg.previous_());
            copyPositionSetpoint(m->current, msg.current_());
            copyPositionSetpoint(m->next, msg.next_());

            position_setpoint_triplet_.reset();
            cv_position_setpoint_triplet_.notify_one();
			msg.serialize(scdr);
			ret = true;
		}
#else
        if (_position_setpoint_triplet_sub.hasMsg()) {
            position_setpoint_triplet_msg_t msg = _position_setpoint_triplet_sub.getMsg();

            // apply timestamp offset
            sync_timestamp_of_outgoing_data(msg);

            msg.serialize(scdr);
            ret = true;

            _position_setpoint_triplet_sub.unlockMsg();
        }
#endif // ROS_BRIDGE
		break;
	}
	case 9: { // telemetry_status subscriber
#ifdef ROS_BRIDGE
		std::unique_lock lk(mtx_telemetry_status_);
		if (auto m = telemetry_status_.get()) {
			telemetry_status_msg_t msg;
            msg.timestamp_(m->timestamp);
			sync_timestamp_of_outgoing_data(msg);
			msg.type_(m->type);
			msg.mode_(m->mode);
			msg.flow_control_(m->flow_control);
			msg.forwarding_(m->forwarding);
			msg.mavlink_v2_(m->mavlink_v2);
			msg.ftp_(m->ftp);
			msg.streams_(m->streams);
			msg.data_rate_(m->data_rate);
			msg.rate_multiplier_(m->rate_multiplier);
			msg.tx_rate_avg_(m->tx_rate_avg);
            msg.tx_error_rate_avg_(m->tx_error_rate_avg);
            msg.tx_message_count_(m->tx_message_count);
            msg.tx_buffer_overruns_(m->tx_buffer_overruns);
            msg.rx_rate_avg_(m->rx_rate_avg);
            msg.rx_message_count_(m->rx_message_count);
            msg.rx_message_lost_count_(m->rx_message_lost_count);
            msg.rx_buffer_overruns_(m->rx_buffer_overruns);
            msg.rx_parse_errors_(m->rx_parse_errors);
            msg.rx_packet_drop_count_(m->rx_packet_drop_count);
            msg.rx_message_lost_rate_(m->rx_message_lost_rate);
            msg.heartbeat_type_antenna_tracker_(m->heartbeat_type_antenna_tracker);
            msg.heartbeat_type_gcs_(m->heartbeat_type_gcs);
            msg.heartbeat_type_onboard_controller_(m->heartbeat_type_onboard_controller);
            msg.heartbeat_type_gimbal_(m->heartbeat_type_gimbal);
            msg.heartbeat_type_adsb_(m->heartbeat_type_adsb);
            msg.heartbeat_type_camera_(m->heartbeat_type_camera);
            msg.heartbeat_type_parachute_(m->heartbeat_type_parachute);
            msg.heartbeat_component_telemetry_radio_(m->heartbeat_component_telemetry_radio);
            msg.heartbeat_component_log_(m->heartbeat_component_log);
            msg.heartbeat_component_osd_(m->heartbeat_component_osd);
            msg.heartbeat_component_obstacle_avoidance_(m->heartbeat_component_obstacle_avoidance);
            msg.heartbeat_component_obstacle_avoidance_(m->heartbeat_component_vio);
            msg.heartbeat_component_udp_bridge_(m->heartbeat_component_udp_bridge);
            msg.heartbeat_component_uart_bridge_(m->heartbeat_component_uart_bridge);
            msg.avoidance_system_healthy_(m->avoidance_system_healthy);
            msg.parachute_system_healthy_(m->parachute_system_healthy);

            telemetry_status_.reset();
            cv_telemetry_status_.notify_one();
            msg.serialize(scdr);
            ret = true;
        }
#else
        if (_telemetry_status_sub.hasMsg()) {
            telemetry_status_msg_t msg = _telemetry_status_sub.getMsg();

            // apply timestamp offset
            sync_timestamp_of_outgoing_data(msg);

            msg.serialize(scdr);
            ret = true;

            _telemetry_status_sub.unlockMsg();
        }
#endif // ROS_BRIDGE
		break;
	}
	case 10: { // timesync subscriber
#ifdef ROS_BRIDGE
		std::unique_lock lk(mtx_timesync_);
		if (auto m = timesync_.get()) {
            timesync_msg_t msg;
            msg.timestamp_(m->timestamp);
            sync_timestamp_of_outgoing_data(msg);
            msg.seq_(m->seq);
            msg.tc1_(m->tc1);
            msg.ts1_(m->ts1);
			LOGD("Timesync: tc1=%ld ts1=%ld", m->tc1, m->ts1);

            timesync_.reset();
            cv_timesync_.notify_one();
            msg.serialize(scdr);
            ret = true;
		}
#else
        if (_timesync_sub.hasMsg()) {
            timesync_msg_t msg = _timesync_sub.getMsg();

            // apply timestamp offset
            sync_timestamp_of_outgoing_data(msg);

            msg.serialize(scdr);
            ret = true;

            _timesync_sub.unlockMsg();
        }
#endif // ROS_BRIDGE
		break;
	}
	case 12: { // vehicle_command subscriber
#ifdef ROS_BRIDGE
		std::unique_lock lk(mtx_vehicle_command_);
		if (auto m = vehicle_command_.get()) {
            vehicle_command_msg_t msg;
            msg.timestamp_(m->timestamp);
            sync_timestamp_of_outgoing_data(msg);
            msg.param1_(m->param1);
            msg.param2_(m->param2);
            msg.param3_(m->param3);
            msg.param4_(m->param4);
            msg.param5_(m->param5);
            msg.param6_(m->param6);
            msg.param7_(m->param7);
            msg.command_(m->command);
            msg.target_system_(m->target_system);
            msg.target_component_(m->target_component);
            msg.source_system_(m->source_system);
            msg.source_component_(m->source_component);
            msg.confirmation_(m->confirmation);
            msg.from_external_(m->from_external);

            vehicle_command_.reset();
            cv_vehicle_command_.notify_one();
            msg.serialize(scdr);
            ret = true;
		}
#else
        if (_vehicle_command_sub.hasMsg()) {
            vehicle_command_msg_t msg = _vehicle_command_sub.getMsg();

            // apply timestamp offset
            sync_timestamp_of_outgoing_data(msg);

            msg.serialize(scdr);
            ret = true;

            _vehicle_command_sub.unlockMsg();
        }
#endif // ROS_BRIDGE
		break;
	}
	case 14: { // vehicle_local_position_setpoint subscriber
#ifdef ROS_BRIDGE
		std::unique_lock lk(mtx_vehicle_local_position_setpoint_);
		if (auto m = vehicle_local_position_setpoint_.get()) {
            vehicle_local_position_setpoint_msg_t msg;
            msg.timestamp_(m->timestamp);
            sync_timestamp_of_outgoing_data(msg);
            msg.x_(m->x);
            msg.y_(m->y);
            msg.z_(m->z);
            msg.vx_(m->vx);
            msg.vy_(m->vy);
            msg.vz_(m->vz);
            msg.acceleration(m->acceleration);
            msg.thrust(m->thrust);
            msg.yaw_(m->yaw);
            msg.yawspeed_(m->yawspeed);

            vehicle_local_position_setpoint_.reset();
            cv_vehicle_local_position_setpoint_.notify_one();
            msg.serialize(scdr);
            ret = true;
        }
#else
        if (_vehicle_local_position_setpoint_sub.hasMsg()) {
            vehicle_local_position_setpoint_msg_t msg = _vehicle_local_position_setpoint_sub.getMsg();

            // apply timestamp offset
            sync_timestamp_of_outgoing_data(msg);

            msg.serialize(scdr);
            ret = true;

            _vehicle_local_position_setpoint_sub.unlockMsg();
        }
#endif // ROS_BRIDGE
		break;
	}
	case 15: { // trajectory_setpoint subscriber
#ifdef ROS_BRIDGE
		std::unique_lock lk(mtx_trajectory_setpoint_);
		if (auto m = trajectory_setpoint_.get()) {
            trajectory_setpoint_msg_t msg;
            msg.timestamp_(m->timestamp);
            sync_timestamp_of_outgoing_data(msg);
            msg.position(m->position);
            msg.velocity(m->velocity);
            msg.acceleration(m->acceleration);
            msg.jerk(m->jerk);
            msg.yaw_(m->yaw);
            msg.yawspeed_(m->yawspeed);

            trajectory_setpoint_.reset();
            cv_trajectory_setpoint_.notify_one();
            msg.serialize(scdr);
            ret = true;
        }
#else
        if (_trajectory_setpoint_sub.hasMsg()) {
            trajectory_setpoint_msg_t msg = _trajectory_setpoint_sub.getMsg();

            // apply timestamp offset
            sync_timestamp_of_outgoing_data(msg);

            msg.serialize(scdr);
            ret = true;

            _trajectory_setpoint_sub.unlockMsg();
        }
#endif // ROS_BRIDGE
		break;
	}
	case 16: { // vehicle_attitude_setpoint subscriber
		// TODO: vehicle_attitude_setpoint
		break;
	}
	case 17: { // trajectory_setpoint subscriber
		// TODO: trajectory_setpoint_subscriber
		break;
	}
	case 22: { // vehicle_trajectory_waypoint subscriber
#ifdef ROS_BRIDGE
		std::unique_lock lk(mtx_vehicle_trajectory_waypoint_);
		if (auto m = vehicle_trajectory_waypoint_.get()) {
            vehicle_trajectory_waypoint_msg_t msg;
            msg.timestamp_(m->timestamp);
            sync_timestamp_of_outgoing_data(msg);
            for (auto i = 0; i < VehicleTrajectoryWaypoint::NUMBER_POINTS; ++i) {
                auto& wp = msg.waypoints()[i];
                sync_timestamp_of_outgoing_data(wp);
                wp.position(m->waypoints[i].position);
                wp.velocity(m->waypoints[i].velocity);
                wp.acceleration(m->waypoints[i].acceleration);
                wp.yaw_(m->waypoints[i].yaw);
                wp.yaw_speed_(m->waypoints[i].yaw_speed);
                wp.point_valid_(m->waypoints[i].point_valid);
                wp.type_(m->waypoints[i].type);
            }

            vehicle_trajectory_waypoint_.reset();
            cv_vehicle_trajectory_waypoint_.notify_one();
            msg.serialize(scdr);
            ret = true;
        }
#else
        if (_vehicle_trajectory_waypoint_sub.hasMsg()) {
            vehicle_trajectory_waypoint_msg_t msg = _vehicle_trajectory_waypoint_sub.getMsg();

            // apply timestamp offset
            sync_timestamp_of_outgoing_data(msg);

            msg.serialize(scdr);
            ret = true;

            _vehicle_trajectory_waypoint_sub.unlockMsg();
        }
#endif // ROS_BRIDGE
		break;
	}
	case 25: { // onboard_computer_status subscriber
#ifdef ROS_BRIDGE
		std::unique_lock lk(mtx_onboard_computer_status_);
		if (auto m = onboard_computer_status_.get()) {
            onboard_computer_status_msg_t msg;
            msg.timestamp_(m->timestamp);
            sync_timestamp_of_outgoing_data(msg);
            msg.uptime_(m->uptime);
            msg.type_(m->type);
            msg.cpu_cores(m->cpu_cores);
            msg.cpu_combined(m->cpu_combined);
            msg.gpu_cores(m->gpu_cores);
            msg.gpu_combined(m->gpu_combined);
            msg.temperature_board_(m->temperature_board);
            for (auto i = 0u; i < msg.temperature_core().size(); ++i) {
                msg.temperature_core()[i] = static_cast<uint8_t>(m->temperature_core[i]);
            }
            msg.fan_speed(m->fan_speed);
            msg.ram_usage_(m->ram_usage);
            msg.ram_total_(m->ram_total);
            msg.storage_type(m->storage_type);
            msg.storage_usage(m->storage_usage);
            msg.storage_total(m->storage_total);
            msg.link_type(m->link_type);
            msg.link_tx_rate(m->link_tx_rate);
            msg.link_rx_rate(m->link_rx_rate);
            msg.link_tx_max(m->link_tx_max);
            msg.link_rx_max(m->link_rx_max);

            onboard_computer_status_.reset();
            cv_onboard_computer_status_.notify_one();
            msg.serialize(scdr);
            ret = true;
        }
#else
        if (_onboard_computer_status_sub.hasMsg()) {
            onboard_computer_status_msg_t msg = _onboard_computer_status_sub.getMsg();

            // apply timestamp offset
            sync_timestamp_of_outgoing_data(msg);

            msg.serialize(scdr);
            ret = true;

            _onboard_computer_status_sub.unlockMsg();
        }
#endif // ROS_BRIDGE
		break;
	}
	case 26: { // trajectory_bezier subscriber
#ifdef ROS_BRIDGE
		std::unique_lock lk(mtx_trajectory_bezier_);
		if (auto m = trajectory_bezier_.get()) {
            trajectory_bezier_msg_t msg;
            msg.timestamp_(m->timestamp);
            sync_timestamp_of_outgoing_data(msg);
            msg.position(m->position);
            msg.yaw_(m->yaw);
            msg.delta_(m->delta);

            trajectory_bezier_.reset();
            cv_trajectory_bezier_.notify_one();
            msg.serialize(scdr);
            ret = true;
        }
#else
        if (_trajectory_bezier_sub.hasMsg()) {
            trajectory_bezier_msg_t msg = _trajectory_bezier_sub.getMsg();

            // apply timestamp offset
            sync_timestamp_of_outgoing_data(msg);

            msg.serialize(scdr);
            ret = true;

            _trajectory_bezier_sub.unlockMsg();
        }
#endif // ROS_BRIDGE
		break;
	}
	case 27: { // vehicle_trajectory_bezier subscriber
#ifdef ROS_BRIDGE
		std::unique_lock lk(mtx_vehicle_trajectory_bezier_);
		if (auto m = vehicle_trajectory_bezier_.get()) {
            vehicle_trajectory_bezier_msg_t msg;
            msg.timestamp_(m->timestamp);
            sync_timestamp_of_outgoing_data(msg);
            int i = 0;
            for (auto& cp : m->control_points) {
                msg.control_points()[i].position(cp.position);
                msg.control_points()[i].yaw_(cp.yaw);
                msg.control_points()[i].delta_(cp.delta);
                ++i;
            }

            vehicle_trajectory_bezier_.reset();
            cv_vehicle_trajectory_bezier_.notify_one();
            msg.serialize(scdr);
            ret = true;
        }
#else
        if (_vehicle_trajectory_bezier_sub.hasMsg()) {
            vehicle_trajectory_bezier_msg_t msg = _vehicle_trajectory_bezier_sub.getMsg();

            // apply timestamp offset
            sync_timestamp_of_outgoing_data(msg);

            msg.serialize(scdr);
            ret = true;

            _vehicle_trajectory_bezier_sub.unlockMsg();
        }
#endif // ROS_BRIDGE
		break;
	}
	case 19: { // vehicle_mocap_odometry subscriber
#ifdef ROS_BRIDGE
		std::unique_lock lk(mtx_vehicle_mocap_odometry_);
		if (auto m = vehicle_mocap_odometry_.get()) {
            vehicle_mocap_odometry_msg_t msg;
            msg.timestamp_(m->timestamp);
            sync_timestamp_of_outgoing_data(msg);
            msg.timestamp_sample_(m->timestamp_sample);
            msg.local_frame_(m->local_frame);
            msg.x_(m->x);
            msg.y_(m->y);
            msg.z_(m->z);
            msg.q(m->q);
            msg.q_offset(m->q_offset);
            msg.pose_covariance(m->pose_covariance);
            msg.velocity_frame_(m->velocity_frame);
            msg.vx_(m->vx);
            msg.vy_(m->vy);
            msg.vz_(m->vz);
            msg.rollspeed_(m->rollspeed);
            msg.pitchspeed_(m->pitchspeed);
            msg.yawspeed_(m->yawspeed);
            msg.velocity_covariance(m->velocity_covariance);
            msg.reset_counter_(m->reset_counter);

            vehicle_mocap_odometry_.reset();
            cv_vehicle_mocap_odometry_.notify_one();
            msg.serialize(scdr);
            ret = true;
        }
#else
        if (_vehicle_mocap_odometry_sub.hasMsg()) {
            vehicle_mocap_odometry_msg_t msg = _vehicle_mocap_odometry_sub.getMsg();

            // apply timestamp offset
            sync_timestamp_of_outgoing_data(msg);

            msg.serialize(scdr);
            ret = true;

            _vehicle_mocap_odometry_sub.unlockMsg();
        }
#endif // ROS_BRIDGE
		break;
	}
	case 20: { // vehicle_visual_odometry subscriber
#ifdef ROS_BRIDGE
		std::unique_lock lk(mtx_vehicle_visual_odometry_);
		if (auto m = vehicle_visual_odometry_.get()) {
			vehicle_visual_odometry_msg_t msg;
            msg.timestamp_(m->timestamp);
			sync_timestamp_of_outgoing_data(msg);
			msg.timestamp_sample_(m->timestamp_sample);
			msg.local_frame_(m->local_frame);
			msg.x_(m->x);
			msg.y_(m->y);
			msg.z_(m->z);
			msg.q(m->q);
			msg.q_offset(m->q_offset);
			msg.pose_covariance(m->pose_covariance);
			msg.velocity_frame_(m->velocity_frame);
			msg.vx_(m->vx);
			msg.vy_(m->vy);
			msg.vz_(m->vz);
			msg.rollspeed_(m->rollspeed);
			msg.pitchspeed_(m->pitchspeed);
			msg.yawspeed_(m->yawspeed);
			msg.velocity_covariance(m->velocity_covariance);
			msg.reset_counter_(m->reset_counter);

			vehicle_visual_odometry_.reset();
			cv_vehicle_visual_odometry_.notify_one();
			msg.serialize(scdr);
			ret = true;
		}
#else
        if (_vehicle_visual_odometry_sub.hasMsg()) {
            vehicle_visual_odometry_msg_t msg = _vehicle_visual_odometry_sub.getMsg();

            // apply timestamp offset
            sync_timestamp_of_outgoing_data(msg);

            msg.serialize(scdr);
            ret = true;

            _vehicle_visual_odometry_sub.unlockMsg();
        }
#endif // ROS_BRIDGE
		break;
	}
	default:
		LOGD("[[   micrortps_agent   ]\tUnexpected topic ID '%hhu' to getMsg. Please make sure the agent is capable of parsing the message associated to the topic ID '%hhu'\033[0m\n",
		       topic_ID, topic_ID);
		break;
	}

	return ret;
}

#ifdef ROS_BRIDGE

void RtpsTopics::copyPositionSetpoint(const px4_msgs::msg::PositionSetpoint& m,
                                      position_setpoint_msg_t& msg) {
    sync_timestamp_of_outgoing_data(msg);
    msg.valid_(m.valid);
    msg.type_(m.type);
    msg.vx_(m.vx);
    msg.vy_(m.vy);
    msg.vz_(m.vz);
    msg.velocity_valid_(m.velocity_valid);
    msg.alt_valid_(m.alt_valid);
    msg.lat_(m.lat);
    msg.lon_(m.lon);
    msg.alt_(m.alt);
    msg.yaw_(m.yaw);
    msg.yaw_valid_(m.yaw_valid);
    msg.yawspeed_(m.yawspeed);
    msg.yawspeed_valid_(m.yawspeed_valid);
    msg.landing_gear_(m.landing_gear);
    msg.loiter_radius_(m.loiter_radius);
    msg.loiter_direction_(m.loiter_direction);
    msg.acceptance_radius_(m.acceptance_radius);
    msg.cruising_speed_(m.cruising_speed);
//			msg.gliding_enabled_(m.gliding_enabled);
    msg.cruising_throttle_(m.cruising_throttle);
//			msg.disable_wheather_vane(m->disable_weather_vane);

}

#endif // ROS_BRIDGE
