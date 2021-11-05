// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*!
 * @file vehicle_status.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _VEHICLE_STATUS_H_
#define _VEHICLE_STATUS_H_

// TODO Poner en el contexto.

#include <stdint.h>
#include <array>
#include <string>
#include <vector>
#include <map>
#include <bitset>

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#define eProsima_user_DllExport __declspec( dllexport )
#else
#define eProsima_user_DllExport
#endif
#else
#define eProsima_user_DllExport
#endif

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#if defined(vehicle_status_SOURCE)
#define vehicle_status_DllAPI __declspec( dllexport )
#else
#define vehicle_status_DllAPI __declspec( dllimport )
#endif // vehicle_status_SOURCE
#else
#define vehicle_status_DllAPI
#endif
#else
#define vehicle_status_DllAPI
#endif // _WIN32

namespace eprosima
{
    namespace fastcdr
    {
        class Cdr;
    }
}


const uint8_t vehicle_status__ARMING_STATE_INIT = 0;
const uint8_t vehicle_status__ARMING_STATE_STANDBY = 1;
const uint8_t vehicle_status__ARMING_STATE_ARMED = 2;
const uint8_t vehicle_status__ARMING_STATE_STANDBY_ERROR = 3;
const uint8_t vehicle_status__ARMING_STATE_SHUTDOWN = 4;
const uint8_t vehicle_status__ARMING_STATE_IN_AIR_RESTORE = 5;
const uint8_t vehicle_status__ARMING_STATE_MAX = 6;
const uint8_t vehicle_status__FAILURE_NONE = 0;
const uint8_t vehicle_status__FAILURE_ROLL = 1;
const uint8_t vehicle_status__FAILURE_PITCH = 2;
const uint8_t vehicle_status__FAILURE_ALT = 4;
const uint8_t vehicle_status__FAILURE_EXT = 8;
const uint8_t vehicle_status__FAILURE_ARM_ESC = 16;
const uint8_t vehicle_status__FAILURE_HIGH_WIND = 32;
const uint8_t vehicle_status__FAILURE_BATTERY = 64;
const uint8_t vehicle_status__FAILURE_IMBALANCED_PROP = 128;
const uint8_t vehicle_status__HIL_STATE_OFF = 0;
const uint8_t vehicle_status__HIL_STATE_ON = 1;
const uint8_t vehicle_status__NAVIGATION_STATE_MANUAL = 0;
const uint8_t vehicle_status__NAVIGATION_STATE_ALTCTL = 1;
const uint8_t vehicle_status__NAVIGATION_STATE_POSCTL = 2;
const uint8_t vehicle_status__NAVIGATION_STATE_AUTO_MISSION = 3;
const uint8_t vehicle_status__NAVIGATION_STATE_AUTO_LOITER = 4;
const uint8_t vehicle_status__NAVIGATION_STATE_AUTO_RTL = 5;
const uint8_t vehicle_status__NAVIGATION_STATE_AUTO_LANDENGFAIL = 8;
const uint8_t vehicle_status__NAVIGATION_STATE_UNUSED = 9;
const uint8_t vehicle_status__NAVIGATION_STATE_ACRO = 10;
const uint8_t vehicle_status__NAVIGATION_STATE_UNUSED1 = 11;
const uint8_t vehicle_status__NAVIGATION_STATE_DESCEND = 12;
const uint8_t vehicle_status__NAVIGATION_STATE_TERMINATION = 13;
const uint8_t vehicle_status__NAVIGATION_STATE_OFFBOARD = 14;
const uint8_t vehicle_status__NAVIGATION_STATE_STAB = 15;
const uint8_t vehicle_status__NAVIGATION_STATE_UNUSED2 = 16;
const uint8_t vehicle_status__NAVIGATION_STATE_AUTO_TAKEOFF = 17;
const uint8_t vehicle_status__NAVIGATION_STATE_AUTO_LAND = 18;
const uint8_t vehicle_status__NAVIGATION_STATE_AUTO_FOLLOW_TARGET = 19;
const uint8_t vehicle_status__NAVIGATION_STATE_AUTO_PRECLAND = 20;
const uint8_t vehicle_status__NAVIGATION_STATE_ORBIT = 21;
const uint8_t vehicle_status__NAVIGATION_STATE_MAX = 22;
const uint8_t vehicle_status__RC_IN_MODE_DEFAULT = 0;
const uint8_t vehicle_status__RC_IN_MODE_OFF = 1;
const uint8_t vehicle_status__RC_IN_MODE_GENERATED = 2;
const uint8_t vehicle_status__VEHICLE_TYPE_UNKNOWN = 0;
const uint8_t vehicle_status__VEHICLE_TYPE_ROTARY_WING = 1;
const uint8_t vehicle_status__VEHICLE_TYPE_FIXED_WING = 2;
const uint8_t vehicle_status__VEHICLE_TYPE_ROVER = 3;
const uint8_t vehicle_status__VEHICLE_TYPE_AIRSHIP = 4;
const uint8_t vehicle_status__ARM_DISARM_REASON_TRANSITION_TO_STANDBY = 0;
const uint8_t vehicle_status__ARM_DISARM_REASON_RC_STICK = 1;
const uint8_t vehicle_status__ARM_DISARM_REASON_RC_SWITCH = 2;
const uint8_t vehicle_status__ARM_DISARM_REASON_COMMAND_INTERNAL = 3;
const uint8_t vehicle_status__ARM_DISARM_REASON_COMMAND_EXTERNAL = 4;
const uint8_t vehicle_status__ARM_DISARM_REASON_MISSION_START = 5;
const uint8_t vehicle_status__ARM_DISARM_REASON_SAFETY_BUTTON = 6;
const uint8_t vehicle_status__ARM_DISARM_REASON_AUTO_DISARM_LAND = 7;
const uint8_t vehicle_status__ARM_DISARM_REASON_AUTO_DISARM_PREFLIGHT = 8;
const uint8_t vehicle_status__ARM_DISARM_REASON_KILL_SWITCH = 9;
const uint8_t vehicle_status__ARM_DISARM_REASON_LOCKDOWN = 10;
const uint8_t vehicle_status__ARM_DISARM_REASON_FAILURE_DETECTOR = 11;
const uint8_t vehicle_status__ARM_DISARM_REASON_SHUTDOWN = 12;
const uint8_t vehicle_status__ARM_DISARM_REASON_UNIT_TEST = 13;
/*!
 * @brief This class represents the structure vehicle_status defined by the user in the IDL file.
 * @ingroup VEHICLE_STATUS
 */
class vehicle_status
{
public:

    /*!
     * @brief Default constructor.
     */
    eProsima_user_DllExport vehicle_status();

    /*!
     * @brief Default destructor.
     */
    eProsima_user_DllExport ~vehicle_status();

    /*!
     * @brief Copy constructor.
     * @param x Reference to the object vehicle_status that will be copied.
     */
    eProsima_user_DllExport vehicle_status(const vehicle_status &x);

    /*!
     * @brief Move constructor.
     * @param x Reference to the object vehicle_status that will be copied.
     */
    eProsima_user_DllExport vehicle_status(vehicle_status &&x);

    /*!
     * @brief Copy assignment.
     * @param x Reference to the object vehicle_status that will be copied.
     */
    eProsima_user_DllExport vehicle_status& operator=(const vehicle_status &x);

    /*!
     * @brief Move assignment.
     * @param x Reference to the object vehicle_status that will be copied.
     */
    eProsima_user_DllExport vehicle_status& operator=(vehicle_status &&x);

    /*!
     * @brief This function sets a value in member timestamp_
     * @param _timestamp_ New value for member timestamp_
     */
    eProsima_user_DllExport void timestamp_(uint64_t _timestamp_);

    /*!
     * @brief This function returns the value of member timestamp_
     * @return Value of member timestamp_
     */
    eProsima_user_DllExport uint64_t timestamp_() const;

    /*!
     * @brief This function returns a reference to member timestamp_
     * @return Reference to member timestamp_
     */
    eProsima_user_DllExport uint64_t& timestamp_();

    /*!
     * @brief This function sets a value in member nav_state_
     * @param _nav_state_ New value for member nav_state_
     */
    eProsima_user_DllExport void nav_state_(uint8_t _nav_state_);

    /*!
     * @brief This function returns the value of member nav_state_
     * @return Value of member nav_state_
     */
    eProsima_user_DllExport uint8_t nav_state_() const;

    /*!
     * @brief This function returns a reference to member nav_state_
     * @return Reference to member nav_state_
     */
    eProsima_user_DllExport uint8_t& nav_state_();

    /*!
     * @brief This function sets a value in member nav_state_timestamp_
     * @param _nav_state_timestamp_ New value for member nav_state_timestamp_
     */
    eProsima_user_DllExport void nav_state_timestamp_(uint64_t _nav_state_timestamp_);

    /*!
     * @brief This function returns the value of member nav_state_timestamp_
     * @return Value of member nav_state_timestamp_
     */
    eProsima_user_DllExport uint64_t nav_state_timestamp_() const;

    /*!
     * @brief This function returns a reference to member nav_state_timestamp_
     * @return Reference to member nav_state_timestamp_
     */
    eProsima_user_DllExport uint64_t& nav_state_timestamp_();

    /*!
     * @brief This function sets a value in member arming_state_
     * @param _arming_state_ New value for member arming_state_
     */
    eProsima_user_DllExport void arming_state_(uint8_t _arming_state_);

    /*!
     * @brief This function returns the value of member arming_state_
     * @return Value of member arming_state_
     */
    eProsima_user_DllExport uint8_t arming_state_() const;

    /*!
     * @brief This function returns a reference to member arming_state_
     * @return Reference to member arming_state_
     */
    eProsima_user_DllExport uint8_t& arming_state_();

    /*!
     * @brief This function sets a value in member hil_state_
     * @param _hil_state_ New value for member hil_state_
     */
    eProsima_user_DllExport void hil_state_(uint8_t _hil_state_);

    /*!
     * @brief This function returns the value of member hil_state_
     * @return Value of member hil_state_
     */
    eProsima_user_DllExport uint8_t hil_state_() const;

    /*!
     * @brief This function returns a reference to member hil_state_
     * @return Reference to member hil_state_
     */
    eProsima_user_DllExport uint8_t& hil_state_();

    /*!
     * @brief This function sets a value in member failsafe_
     * @param _failsafe_ New value for member failsafe_
     */
    eProsima_user_DllExport void failsafe_(bool _failsafe_);

    /*!
     * @brief This function returns the value of member failsafe_
     * @return Value of member failsafe_
     */
    eProsima_user_DllExport bool failsafe_() const;

    /*!
     * @brief This function returns a reference to member failsafe_
     * @return Reference to member failsafe_
     */
    eProsima_user_DllExport bool& failsafe_();

    /*!
     * @brief This function sets a value in member failsafe_timestamp_
     * @param _failsafe_timestamp_ New value for member failsafe_timestamp_
     */
    eProsima_user_DllExport void failsafe_timestamp_(uint64_t _failsafe_timestamp_);

    /*!
     * @brief This function returns the value of member failsafe_timestamp_
     * @return Value of member failsafe_timestamp_
     */
    eProsima_user_DllExport uint64_t failsafe_timestamp_() const;

    /*!
     * @brief This function returns a reference to member failsafe_timestamp_
     * @return Reference to member failsafe_timestamp_
     */
    eProsima_user_DllExport uint64_t& failsafe_timestamp_();

    /*!
     * @brief This function sets a value in member system_type_
     * @param _system_type_ New value for member system_type_
     */
    eProsima_user_DllExport void system_type_(uint8_t _system_type_);

    /*!
     * @brief This function returns the value of member system_type_
     * @return Value of member system_type_
     */
    eProsima_user_DllExport uint8_t system_type_() const;

    /*!
     * @brief This function returns a reference to member system_type_
     * @return Reference to member system_type_
     */
    eProsima_user_DllExport uint8_t& system_type_();

    /*!
     * @brief This function sets a value in member system_id_
     * @param _system_id_ New value for member system_id_
     */
    eProsima_user_DllExport void system_id_(uint8_t _system_id_);

    /*!
     * @brief This function returns the value of member system_id_
     * @return Value of member system_id_
     */
    eProsima_user_DllExport uint8_t system_id_() const;

    /*!
     * @brief This function returns a reference to member system_id_
     * @return Reference to member system_id_
     */
    eProsima_user_DllExport uint8_t& system_id_();

    /*!
     * @brief This function sets a value in member component_id_
     * @param _component_id_ New value for member component_id_
     */
    eProsima_user_DllExport void component_id_(uint8_t _component_id_);

    /*!
     * @brief This function returns the value of member component_id_
     * @return Value of member component_id_
     */
    eProsima_user_DllExport uint8_t component_id_() const;

    /*!
     * @brief This function returns a reference to member component_id_
     * @return Reference to member component_id_
     */
    eProsima_user_DllExport uint8_t& component_id_();

    /*!
     * @brief This function sets a value in member vehicle_type_
     * @param _vehicle_type_ New value for member vehicle_type_
     */
    eProsima_user_DllExport void vehicle_type_(uint8_t _vehicle_type_);

    /*!
     * @brief This function returns the value of member vehicle_type_
     * @return Value of member vehicle_type_
     */
    eProsima_user_DllExport uint8_t vehicle_type_() const;

    /*!
     * @brief This function returns a reference to member vehicle_type_
     * @return Reference to member vehicle_type_
     */
    eProsima_user_DllExport uint8_t& vehicle_type_();

    /*!
     * @brief This function sets a value in member is_vtol_
     * @param _is_vtol_ New value for member is_vtol_
     */
    eProsima_user_DllExport void is_vtol_(bool _is_vtol_);

    /*!
     * @brief This function returns the value of member is_vtol_
     * @return Value of member is_vtol_
     */
    eProsima_user_DllExport bool is_vtol_() const;

    /*!
     * @brief This function returns a reference to member is_vtol_
     * @return Reference to member is_vtol_
     */
    eProsima_user_DllExport bool& is_vtol_();

    /*!
     * @brief This function sets a value in member is_vtol_tailsitter_
     * @param _is_vtol_tailsitter_ New value for member is_vtol_tailsitter_
     */
    eProsima_user_DllExport void is_vtol_tailsitter_(bool _is_vtol_tailsitter_);

    /*!
     * @brief This function returns the value of member is_vtol_tailsitter_
     * @return Value of member is_vtol_tailsitter_
     */
    eProsima_user_DllExport bool is_vtol_tailsitter_() const;

    /*!
     * @brief This function returns a reference to member is_vtol_tailsitter_
     * @return Reference to member is_vtol_tailsitter_
     */
    eProsima_user_DllExport bool& is_vtol_tailsitter_();

    /*!
     * @brief This function sets a value in member vtol_fw_permanent_stab_
     * @param _vtol_fw_permanent_stab_ New value for member vtol_fw_permanent_stab_
     */
    eProsima_user_DllExport void vtol_fw_permanent_stab_(bool _vtol_fw_permanent_stab_);

    /*!
     * @brief This function returns the value of member vtol_fw_permanent_stab_
     * @return Value of member vtol_fw_permanent_stab_
     */
    eProsima_user_DllExport bool vtol_fw_permanent_stab_() const;

    /*!
     * @brief This function returns a reference to member vtol_fw_permanent_stab_
     * @return Reference to member vtol_fw_permanent_stab_
     */
    eProsima_user_DllExport bool& vtol_fw_permanent_stab_();

    /*!
     * @brief This function sets a value in member in_transition_mode_
     * @param _in_transition_mode_ New value for member in_transition_mode_
     */
    eProsima_user_DllExport void in_transition_mode_(bool _in_transition_mode_);

    /*!
     * @brief This function returns the value of member in_transition_mode_
     * @return Value of member in_transition_mode_
     */
    eProsima_user_DllExport bool in_transition_mode_() const;

    /*!
     * @brief This function returns a reference to member in_transition_mode_
     * @return Reference to member in_transition_mode_
     */
    eProsima_user_DllExport bool& in_transition_mode_();

    /*!
     * @brief This function sets a value in member in_transition_to_fw_
     * @param _in_transition_to_fw_ New value for member in_transition_to_fw_
     */
    eProsima_user_DllExport void in_transition_to_fw_(bool _in_transition_to_fw_);

    /*!
     * @brief This function returns the value of member in_transition_to_fw_
     * @return Value of member in_transition_to_fw_
     */
    eProsima_user_DllExport bool in_transition_to_fw_() const;

    /*!
     * @brief This function returns a reference to member in_transition_to_fw_
     * @return Reference to member in_transition_to_fw_
     */
    eProsima_user_DllExport bool& in_transition_to_fw_();

    /*!
     * @brief This function sets a value in member rc_signal_lost_
     * @param _rc_signal_lost_ New value for member rc_signal_lost_
     */
    eProsima_user_DllExport void rc_signal_lost_(bool _rc_signal_lost_);

    /*!
     * @brief This function returns the value of member rc_signal_lost_
     * @return Value of member rc_signal_lost_
     */
    eProsima_user_DllExport bool rc_signal_lost_() const;

    /*!
     * @brief This function returns a reference to member rc_signal_lost_
     * @return Reference to member rc_signal_lost_
     */
    eProsima_user_DllExport bool& rc_signal_lost_();

    /*!
     * @brief This function sets a value in member rc_input_mode_
     * @param _rc_input_mode_ New value for member rc_input_mode_
     */
    eProsima_user_DllExport void rc_input_mode_(uint8_t _rc_input_mode_);

    /*!
     * @brief This function returns the value of member rc_input_mode_
     * @return Value of member rc_input_mode_
     */
    eProsima_user_DllExport uint8_t rc_input_mode_() const;

    /*!
     * @brief This function returns a reference to member rc_input_mode_
     * @return Reference to member rc_input_mode_
     */
    eProsima_user_DllExport uint8_t& rc_input_mode_();

    /*!
     * @brief This function sets a value in member data_link_lost_
     * @param _data_link_lost_ New value for member data_link_lost_
     */
    eProsima_user_DllExport void data_link_lost_(bool _data_link_lost_);

    /*!
     * @brief This function returns the value of member data_link_lost_
     * @return Value of member data_link_lost_
     */
    eProsima_user_DllExport bool data_link_lost_() const;

    /*!
     * @brief This function returns a reference to member data_link_lost_
     * @return Reference to member data_link_lost_
     */
    eProsima_user_DllExport bool& data_link_lost_();

    /*!
     * @brief This function sets a value in member data_link_lost_counter_
     * @param _data_link_lost_counter_ New value for member data_link_lost_counter_
     */
    eProsima_user_DllExport void data_link_lost_counter_(uint8_t _data_link_lost_counter_);

    /*!
     * @brief This function returns the value of member data_link_lost_counter_
     * @return Value of member data_link_lost_counter_
     */
    eProsima_user_DllExport uint8_t data_link_lost_counter_() const;

    /*!
     * @brief This function returns a reference to member data_link_lost_counter_
     * @return Reference to member data_link_lost_counter_
     */
    eProsima_user_DllExport uint8_t& data_link_lost_counter_();

    /*!
     * @brief This function sets a value in member high_latency_data_link_lost_
     * @param _high_latency_data_link_lost_ New value for member high_latency_data_link_lost_
     */
    eProsima_user_DllExport void high_latency_data_link_lost_(bool _high_latency_data_link_lost_);

    /*!
     * @brief This function returns the value of member high_latency_data_link_lost_
     * @return Value of member high_latency_data_link_lost_
     */
    eProsima_user_DllExport bool high_latency_data_link_lost_() const;

    /*!
     * @brief This function returns a reference to member high_latency_data_link_lost_
     * @return Reference to member high_latency_data_link_lost_
     */
    eProsima_user_DllExport bool& high_latency_data_link_lost_();

    /*!
     * @brief This function sets a value in member engine_failure_
     * @param _engine_failure_ New value for member engine_failure_
     */
    eProsima_user_DllExport void engine_failure_(bool _engine_failure_);

    /*!
     * @brief This function returns the value of member engine_failure_
     * @return Value of member engine_failure_
     */
    eProsima_user_DllExport bool engine_failure_() const;

    /*!
     * @brief This function returns a reference to member engine_failure_
     * @return Reference to member engine_failure_
     */
    eProsima_user_DllExport bool& engine_failure_();

    /*!
     * @brief This function sets a value in member mission_failure_
     * @param _mission_failure_ New value for member mission_failure_
     */
    eProsima_user_DllExport void mission_failure_(bool _mission_failure_);

    /*!
     * @brief This function returns the value of member mission_failure_
     * @return Value of member mission_failure_
     */
    eProsima_user_DllExport bool mission_failure_() const;

    /*!
     * @brief This function returns a reference to member mission_failure_
     * @return Reference to member mission_failure_
     */
    eProsima_user_DllExport bool& mission_failure_();

    /*!
     * @brief This function sets a value in member geofence_violated_
     * @param _geofence_violated_ New value for member geofence_violated_
     */
    eProsima_user_DllExport void geofence_violated_(bool _geofence_violated_);

    /*!
     * @brief This function returns the value of member geofence_violated_
     * @return Value of member geofence_violated_
     */
    eProsima_user_DllExport bool geofence_violated_() const;

    /*!
     * @brief This function returns a reference to member geofence_violated_
     * @return Reference to member geofence_violated_
     */
    eProsima_user_DllExport bool& geofence_violated_();

    /*!
     * @brief This function sets a value in member failure_detector_status_
     * @param _failure_detector_status_ New value for member failure_detector_status_
     */
    eProsima_user_DllExport void failure_detector_status_(uint8_t _failure_detector_status_);

    /*!
     * @brief This function returns the value of member failure_detector_status_
     * @return Value of member failure_detector_status_
     */
    eProsima_user_DllExport uint8_t failure_detector_status_() const;

    /*!
     * @brief This function returns a reference to member failure_detector_status_
     * @return Reference to member failure_detector_status_
     */
    eProsima_user_DllExport uint8_t& failure_detector_status_();

    /*!
     * @brief This function sets a value in member onboard_control_sensors_present_
     * @param _onboard_control_sensors_present_ New value for member onboard_control_sensors_present_
     */
    eProsima_user_DllExport void onboard_control_sensors_present_(uint32_t _onboard_control_sensors_present_);

    /*!
     * @brief This function returns the value of member onboard_control_sensors_present_
     * @return Value of member onboard_control_sensors_present_
     */
    eProsima_user_DllExport uint32_t onboard_control_sensors_present_() const;

    /*!
     * @brief This function returns a reference to member onboard_control_sensors_present_
     * @return Reference to member onboard_control_sensors_present_
     */
    eProsima_user_DllExport uint32_t& onboard_control_sensors_present_();

    /*!
     * @brief This function sets a value in member onboard_control_sensors_enabled_
     * @param _onboard_control_sensors_enabled_ New value for member onboard_control_sensors_enabled_
     */
    eProsima_user_DllExport void onboard_control_sensors_enabled_(uint32_t _onboard_control_sensors_enabled_);

    /*!
     * @brief This function returns the value of member onboard_control_sensors_enabled_
     * @return Value of member onboard_control_sensors_enabled_
     */
    eProsima_user_DllExport uint32_t onboard_control_sensors_enabled_() const;

    /*!
     * @brief This function returns a reference to member onboard_control_sensors_enabled_
     * @return Reference to member onboard_control_sensors_enabled_
     */
    eProsima_user_DllExport uint32_t& onboard_control_sensors_enabled_();

    /*!
     * @brief This function sets a value in member onboard_control_sensors_health_
     * @param _onboard_control_sensors_health_ New value for member onboard_control_sensors_health_
     */
    eProsima_user_DllExport void onboard_control_sensors_health_(uint32_t _onboard_control_sensors_health_);

    /*!
     * @brief This function returns the value of member onboard_control_sensors_health_
     * @return Value of member onboard_control_sensors_health_
     */
    eProsima_user_DllExport uint32_t onboard_control_sensors_health_() const;

    /*!
     * @brief This function returns a reference to member onboard_control_sensors_health_
     * @return Reference to member onboard_control_sensors_health_
     */
    eProsima_user_DllExport uint32_t& onboard_control_sensors_health_();

    /*!
     * @brief This function sets a value in member latest_arming_reason_
     * @param _latest_arming_reason_ New value for member latest_arming_reason_
     */
    eProsima_user_DllExport void latest_arming_reason_(uint8_t _latest_arming_reason_);

    /*!
     * @brief This function returns the value of member latest_arming_reason_
     * @return Value of member latest_arming_reason_
     */
    eProsima_user_DllExport uint8_t latest_arming_reason_() const;

    /*!
     * @brief This function returns a reference to member latest_arming_reason_
     * @return Reference to member latest_arming_reason_
     */
    eProsima_user_DllExport uint8_t& latest_arming_reason_();

    /*!
     * @brief This function sets a value in member latest_disarming_reason_
     * @param _latest_disarming_reason_ New value for member latest_disarming_reason_
     */
    eProsima_user_DllExport void latest_disarming_reason_(uint8_t _latest_disarming_reason_);

    /*!
     * @brief This function returns the value of member latest_disarming_reason_
     * @return Value of member latest_disarming_reason_
     */
    eProsima_user_DllExport uint8_t latest_disarming_reason_() const;

    /*!
     * @brief This function returns a reference to member latest_disarming_reason_
     * @return Reference to member latest_disarming_reason_
     */
    eProsima_user_DllExport uint8_t& latest_disarming_reason_();

    /*!
     * @brief This function sets a value in member armed_time_
     * @param _armed_time_ New value for member armed_time_
     */
    eProsima_user_DllExport void armed_time_(uint64_t _armed_time_);

    /*!
     * @brief This function returns the value of member armed_time_
     * @return Value of member armed_time_
     */
    eProsima_user_DllExport uint64_t armed_time_() const;

    /*!
     * @brief This function returns a reference to member armed_time_
     * @return Reference to member armed_time_
     */
    eProsima_user_DllExport uint64_t& armed_time_();

    /*!
     * @brief This function sets a value in member takeoff_time_
     * @param _takeoff_time_ New value for member takeoff_time_
     */
    eProsima_user_DllExport void takeoff_time_(uint64_t _takeoff_time_);

    /*!
     * @brief This function returns the value of member takeoff_time_
     * @return Value of member takeoff_time_
     */
    eProsima_user_DllExport uint64_t takeoff_time_() const;

    /*!
     * @brief This function returns a reference to member takeoff_time_
     * @return Reference to member takeoff_time_
     */
    eProsima_user_DllExport uint64_t& takeoff_time_();


    /*!
     * @brief This function returns the maximum serialized size of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    eProsima_user_DllExport static size_t getMaxCdrSerializedSize(size_t current_alignment = 0);

    /*!
     * @brief This function returns the serialized size of a data depending on the buffer alignment.
     * @param data Data which is calculated its serialized size.
     * @param current_alignment Buffer alignment.
     * @return Serialized size.
     */
    eProsima_user_DllExport static size_t getCdrSerializedSize(const vehicle_status& data, size_t current_alignment = 0);


    /*!
     * @brief This function serializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void serialize(eprosima::fastcdr::Cdr &cdr) const;

    /*!
     * @brief This function deserializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void deserialize(eprosima::fastcdr::Cdr &cdr);



    /*!
     * @brief This function returns the maximum serialized size of the Key of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    eProsima_user_DllExport static size_t getKeyMaxCdrSerializedSize(size_t current_alignment = 0);

    /*!
     * @brief This function tells you if the Key has been defined for this type
     */
    eProsima_user_DllExport static bool isKeyDefined();

    /*!
     * @brief This function serializes the key members of an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void serializeKey(eprosima::fastcdr::Cdr &cdr) const;

private:
    uint64_t m_timestamp_;
    uint8_t m_nav_state_;
    uint64_t m_nav_state_timestamp_;
    uint8_t m_arming_state_;
    uint8_t m_hil_state_;
    bool m_failsafe_;
    uint64_t m_failsafe_timestamp_;
    uint8_t m_system_type_;
    uint8_t m_system_id_;
    uint8_t m_component_id_;
    uint8_t m_vehicle_type_;
    bool m_is_vtol_;
    bool m_is_vtol_tailsitter_;
    bool m_vtol_fw_permanent_stab_;
    bool m_in_transition_mode_;
    bool m_in_transition_to_fw_;
    bool m_rc_signal_lost_;
    uint8_t m_rc_input_mode_;
    bool m_data_link_lost_;
    uint8_t m_data_link_lost_counter_;
    bool m_high_latency_data_link_lost_;
    bool m_engine_failure_;
    bool m_mission_failure_;
    bool m_geofence_violated_;
    uint8_t m_failure_detector_status_;
    uint32_t m_onboard_control_sensors_present_;
    uint32_t m_onboard_control_sensors_enabled_;
    uint32_t m_onboard_control_sensors_health_;
    uint8_t m_latest_arming_reason_;
    uint8_t m_latest_disarming_reason_;
    uint64_t m_armed_time_;
    uint64_t m_takeoff_time_;
};

#endif // _VEHICLE_STATUS_H_