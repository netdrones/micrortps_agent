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
 * @file position_setpoint.cpp
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace { char dummy; }
#endif

#include "position_setpoint.h"
#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>










position_setpoint::position_setpoint()
{
    // m_timestamp_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@f4168b8
    m_timestamp_ = 0;
    // m_valid_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@3bd94634
    m_valid_ = false;
    // m_type_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@58a90037
    m_type_ = 0;
    // m_vx_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@74294adb
    m_vx_ = 0.0;
    // m_vy_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@70a9f84e
    m_vy_ = 0.0;
    // m_vz_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@130f889
    m_vz_ = 0.0;
    // m_velocity_valid_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@1188e820
    m_velocity_valid_ = false;
    // m_velocity_frame_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@2f490758
    m_velocity_frame_ = 0;
    // m_alt_valid_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@101df177
    m_alt_valid_ = false;
    // m_lat_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@166fa74d
    m_lat_ = 0.0;
    // m_lon_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@40f08448
    m_lon_ = 0.0;
    // m_alt_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@276438c9
    m_alt_ = 0.0;
    // m_yaw_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@588df31b
    m_yaw_ = 0.0;
    // m_yaw_valid_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@33b37288
    m_yaw_valid_ = false;
    // m_yawspeed_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@77a57272
    m_yawspeed_ = 0.0;
    // m_yawspeed_valid_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@7181ae3f
    m_yawspeed_valid_ = false;
    // m_landing_gear_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@46238e3f
    m_landing_gear_ = 0;
    // m_loiter_radius_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@6e2c9341
    m_loiter_radius_ = 0.0;
    // m_loiter_direction_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@32464a14
    m_loiter_direction_ = 0;
    // m_acceptance_radius_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@4e4aea35
    m_acceptance_radius_ = 0.0;
    // m_cruising_speed_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@1442d7b5
    m_cruising_speed_ = 0.0;
    // m_cruising_throttle_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@1efee8e7
    m_cruising_throttle_ = 0.0;
    // m_disable_weather_vane_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@1ee807c6
    m_disable_weather_vane_ = false;

}

position_setpoint::~position_setpoint()
{























}

position_setpoint::position_setpoint(const position_setpoint &x)
{
    m_timestamp_ = x.m_timestamp_;
    m_valid_ = x.m_valid_;
    m_type_ = x.m_type_;
    m_vx_ = x.m_vx_;
    m_vy_ = x.m_vy_;
    m_vz_ = x.m_vz_;
    m_velocity_valid_ = x.m_velocity_valid_;
    m_velocity_frame_ = x.m_velocity_frame_;
    m_alt_valid_ = x.m_alt_valid_;
    m_lat_ = x.m_lat_;
    m_lon_ = x.m_lon_;
    m_alt_ = x.m_alt_;
    m_yaw_ = x.m_yaw_;
    m_yaw_valid_ = x.m_yaw_valid_;
    m_yawspeed_ = x.m_yawspeed_;
    m_yawspeed_valid_ = x.m_yawspeed_valid_;
    m_landing_gear_ = x.m_landing_gear_;
    m_loiter_radius_ = x.m_loiter_radius_;
    m_loiter_direction_ = x.m_loiter_direction_;
    m_acceptance_radius_ = x.m_acceptance_radius_;
    m_cruising_speed_ = x.m_cruising_speed_;
    m_cruising_throttle_ = x.m_cruising_throttle_;
    m_disable_weather_vane_ = x.m_disable_weather_vane_;
}

position_setpoint::position_setpoint(position_setpoint &&x)
{
    m_timestamp_ = x.m_timestamp_;
    m_valid_ = x.m_valid_;
    m_type_ = x.m_type_;
    m_vx_ = x.m_vx_;
    m_vy_ = x.m_vy_;
    m_vz_ = x.m_vz_;
    m_velocity_valid_ = x.m_velocity_valid_;
    m_velocity_frame_ = x.m_velocity_frame_;
    m_alt_valid_ = x.m_alt_valid_;
    m_lat_ = x.m_lat_;
    m_lon_ = x.m_lon_;
    m_alt_ = x.m_alt_;
    m_yaw_ = x.m_yaw_;
    m_yaw_valid_ = x.m_yaw_valid_;
    m_yawspeed_ = x.m_yawspeed_;
    m_yawspeed_valid_ = x.m_yawspeed_valid_;
    m_landing_gear_ = x.m_landing_gear_;
    m_loiter_radius_ = x.m_loiter_radius_;
    m_loiter_direction_ = x.m_loiter_direction_;
    m_acceptance_radius_ = x.m_acceptance_radius_;
    m_cruising_speed_ = x.m_cruising_speed_;
    m_cruising_throttle_ = x.m_cruising_throttle_;
    m_disable_weather_vane_ = x.m_disable_weather_vane_;
}

position_setpoint& position_setpoint::operator=(const position_setpoint &x)
{

    m_timestamp_ = x.m_timestamp_;
    m_valid_ = x.m_valid_;
    m_type_ = x.m_type_;
    m_vx_ = x.m_vx_;
    m_vy_ = x.m_vy_;
    m_vz_ = x.m_vz_;
    m_velocity_valid_ = x.m_velocity_valid_;
    m_velocity_frame_ = x.m_velocity_frame_;
    m_alt_valid_ = x.m_alt_valid_;
    m_lat_ = x.m_lat_;
    m_lon_ = x.m_lon_;
    m_alt_ = x.m_alt_;
    m_yaw_ = x.m_yaw_;
    m_yaw_valid_ = x.m_yaw_valid_;
    m_yawspeed_ = x.m_yawspeed_;
    m_yawspeed_valid_ = x.m_yawspeed_valid_;
    m_landing_gear_ = x.m_landing_gear_;
    m_loiter_radius_ = x.m_loiter_radius_;
    m_loiter_direction_ = x.m_loiter_direction_;
    m_acceptance_radius_ = x.m_acceptance_radius_;
    m_cruising_speed_ = x.m_cruising_speed_;
    m_cruising_throttle_ = x.m_cruising_throttle_;
    m_disable_weather_vane_ = x.m_disable_weather_vane_;

    return *this;
}

position_setpoint& position_setpoint::operator=(position_setpoint &&x)
{

    m_timestamp_ = x.m_timestamp_;
    m_valid_ = x.m_valid_;
    m_type_ = x.m_type_;
    m_vx_ = x.m_vx_;
    m_vy_ = x.m_vy_;
    m_vz_ = x.m_vz_;
    m_velocity_valid_ = x.m_velocity_valid_;
    m_velocity_frame_ = x.m_velocity_frame_;
    m_alt_valid_ = x.m_alt_valid_;
    m_lat_ = x.m_lat_;
    m_lon_ = x.m_lon_;
    m_alt_ = x.m_alt_;
    m_yaw_ = x.m_yaw_;
    m_yaw_valid_ = x.m_yaw_valid_;
    m_yawspeed_ = x.m_yawspeed_;
    m_yawspeed_valid_ = x.m_yawspeed_valid_;
    m_landing_gear_ = x.m_landing_gear_;
    m_loiter_radius_ = x.m_loiter_radius_;
    m_loiter_direction_ = x.m_loiter_direction_;
    m_acceptance_radius_ = x.m_acceptance_radius_;
    m_cruising_speed_ = x.m_cruising_speed_;
    m_cruising_throttle_ = x.m_cruising_throttle_;
    m_disable_weather_vane_ = x.m_disable_weather_vane_;

    return *this;
}

size_t position_setpoint::getMaxCdrSerializedSize(size_t current_alignment)
{
    size_t initial_alignment = current_alignment;


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);



    return current_alignment - initial_alignment;
}

size_t position_setpoint::getCdrSerializedSize(const position_setpoint& data, size_t current_alignment)
{
    (void)data;
    size_t initial_alignment = current_alignment;


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);



    return current_alignment - initial_alignment;
}

void position_setpoint::serialize(eprosima::fastcdr::Cdr &scdr) const
{

    scdr << m_timestamp_;
    scdr << m_valid_;
    scdr << m_type_;
    scdr << m_vx_;
    scdr << m_vy_;
    scdr << m_vz_;
    scdr << m_velocity_valid_;
    scdr << m_velocity_frame_;
    scdr << m_alt_valid_;
    scdr << m_lat_;
    scdr << m_lon_;
    scdr << m_alt_;
    scdr << m_yaw_;
    scdr << m_yaw_valid_;
    scdr << m_yawspeed_;
    scdr << m_yawspeed_valid_;
    scdr << m_landing_gear_;
    scdr << m_loiter_radius_;
    scdr << m_loiter_direction_;
    scdr << m_acceptance_radius_;
    scdr << m_cruising_speed_;
    scdr << m_cruising_throttle_;
    scdr << m_disable_weather_vane_;
}

void position_setpoint::deserialize(eprosima::fastcdr::Cdr &dcdr)
{

    dcdr >> m_timestamp_;
    dcdr >> m_valid_;
    dcdr >> m_type_;
    dcdr >> m_vx_;
    dcdr >> m_vy_;
    dcdr >> m_vz_;
    dcdr >> m_velocity_valid_;
    dcdr >> m_velocity_frame_;
    dcdr >> m_alt_valid_;
    dcdr >> m_lat_;
    dcdr >> m_lon_;
    dcdr >> m_alt_;
    dcdr >> m_yaw_;
    dcdr >> m_yaw_valid_;
    dcdr >> m_yawspeed_;
    dcdr >> m_yawspeed_valid_;
    dcdr >> m_landing_gear_;
    dcdr >> m_loiter_radius_;
    dcdr >> m_loiter_direction_;
    dcdr >> m_acceptance_radius_;
    dcdr >> m_cruising_speed_;
    dcdr >> m_cruising_throttle_;
    dcdr >> m_disable_weather_vane_;
}

/*!
 * @brief This function sets a value in member timestamp_
 * @param _timestamp_ New value for member timestamp_
 */
void position_setpoint::timestamp_(uint64_t _timestamp_)
{
m_timestamp_ = _timestamp_;
}

/*!
 * @brief This function returns the value of member timestamp_
 * @return Value of member timestamp_
 */
uint64_t position_setpoint::timestamp_() const
{
    return m_timestamp_;
}

/*!
 * @brief This function returns a reference to member timestamp_
 * @return Reference to member timestamp_
 */
uint64_t& position_setpoint::timestamp_()
{
    return m_timestamp_;
}

/*!
 * @brief This function sets a value in member valid_
 * @param _valid_ New value for member valid_
 */
void position_setpoint::valid_(bool _valid_)
{
m_valid_ = _valid_;
}

/*!
 * @brief This function returns the value of member valid_
 * @return Value of member valid_
 */
bool position_setpoint::valid_() const
{
    return m_valid_;
}

/*!
 * @brief This function returns a reference to member valid_
 * @return Reference to member valid_
 */
bool& position_setpoint::valid_()
{
    return m_valid_;
}

/*!
 * @brief This function sets a value in member type_
 * @param _type_ New value for member type_
 */
void position_setpoint::type_(uint8_t _type_)
{
m_type_ = _type_;
}

/*!
 * @brief This function returns the value of member type_
 * @return Value of member type_
 */
uint8_t position_setpoint::type_() const
{
    return m_type_;
}

/*!
 * @brief This function returns a reference to member type_
 * @return Reference to member type_
 */
uint8_t& position_setpoint::type_()
{
    return m_type_;
}

/*!
 * @brief This function sets a value in member vx_
 * @param _vx_ New value for member vx_
 */
void position_setpoint::vx_(float _vx_)
{
m_vx_ = _vx_;
}

/*!
 * @brief This function returns the value of member vx_
 * @return Value of member vx_
 */
float position_setpoint::vx_() const
{
    return m_vx_;
}

/*!
 * @brief This function returns a reference to member vx_
 * @return Reference to member vx_
 */
float& position_setpoint::vx_()
{
    return m_vx_;
}

/*!
 * @brief This function sets a value in member vy_
 * @param _vy_ New value for member vy_
 */
void position_setpoint::vy_(float _vy_)
{
m_vy_ = _vy_;
}

/*!
 * @brief This function returns the value of member vy_
 * @return Value of member vy_
 */
float position_setpoint::vy_() const
{
    return m_vy_;
}

/*!
 * @brief This function returns a reference to member vy_
 * @return Reference to member vy_
 */
float& position_setpoint::vy_()
{
    return m_vy_;
}

/*!
 * @brief This function sets a value in member vz_
 * @param _vz_ New value for member vz_
 */
void position_setpoint::vz_(float _vz_)
{
m_vz_ = _vz_;
}

/*!
 * @brief This function returns the value of member vz_
 * @return Value of member vz_
 */
float position_setpoint::vz_() const
{
    return m_vz_;
}

/*!
 * @brief This function returns a reference to member vz_
 * @return Reference to member vz_
 */
float& position_setpoint::vz_()
{
    return m_vz_;
}

/*!
 * @brief This function sets a value in member velocity_valid_
 * @param _velocity_valid_ New value for member velocity_valid_
 */
void position_setpoint::velocity_valid_(bool _velocity_valid_)
{
m_velocity_valid_ = _velocity_valid_;
}

/*!
 * @brief This function returns the value of member velocity_valid_
 * @return Value of member velocity_valid_
 */
bool position_setpoint::velocity_valid_() const
{
    return m_velocity_valid_;
}

/*!
 * @brief This function returns a reference to member velocity_valid_
 * @return Reference to member velocity_valid_
 */
bool& position_setpoint::velocity_valid_()
{
    return m_velocity_valid_;
}

/*!
 * @brief This function sets a value in member velocity_frame_
 * @param _velocity_frame_ New value for member velocity_frame_
 */
void position_setpoint::velocity_frame_(uint8_t _velocity_frame_)
{
m_velocity_frame_ = _velocity_frame_;
}

/*!
 * @brief This function returns the value of member velocity_frame_
 * @return Value of member velocity_frame_
 */
uint8_t position_setpoint::velocity_frame_() const
{
    return m_velocity_frame_;
}

/*!
 * @brief This function returns a reference to member velocity_frame_
 * @return Reference to member velocity_frame_
 */
uint8_t& position_setpoint::velocity_frame_()
{
    return m_velocity_frame_;
}

/*!
 * @brief This function sets a value in member alt_valid_
 * @param _alt_valid_ New value for member alt_valid_
 */
void position_setpoint::alt_valid_(bool _alt_valid_)
{
m_alt_valid_ = _alt_valid_;
}

/*!
 * @brief This function returns the value of member alt_valid_
 * @return Value of member alt_valid_
 */
bool position_setpoint::alt_valid_() const
{
    return m_alt_valid_;
}

/*!
 * @brief This function returns a reference to member alt_valid_
 * @return Reference to member alt_valid_
 */
bool& position_setpoint::alt_valid_()
{
    return m_alt_valid_;
}

/*!
 * @brief This function sets a value in member lat_
 * @param _lat_ New value for member lat_
 */
void position_setpoint::lat_(double _lat_)
{
m_lat_ = _lat_;
}

/*!
 * @brief This function returns the value of member lat_
 * @return Value of member lat_
 */
double position_setpoint::lat_() const
{
    return m_lat_;
}

/*!
 * @brief This function returns a reference to member lat_
 * @return Reference to member lat_
 */
double& position_setpoint::lat_()
{
    return m_lat_;
}

/*!
 * @brief This function sets a value in member lon_
 * @param _lon_ New value for member lon_
 */
void position_setpoint::lon_(double _lon_)
{
m_lon_ = _lon_;
}

/*!
 * @brief This function returns the value of member lon_
 * @return Value of member lon_
 */
double position_setpoint::lon_() const
{
    return m_lon_;
}

/*!
 * @brief This function returns a reference to member lon_
 * @return Reference to member lon_
 */
double& position_setpoint::lon_()
{
    return m_lon_;
}

/*!
 * @brief This function sets a value in member alt_
 * @param _alt_ New value for member alt_
 */
void position_setpoint::alt_(float _alt_)
{
m_alt_ = _alt_;
}

/*!
 * @brief This function returns the value of member alt_
 * @return Value of member alt_
 */
float position_setpoint::alt_() const
{
    return m_alt_;
}

/*!
 * @brief This function returns a reference to member alt_
 * @return Reference to member alt_
 */
float& position_setpoint::alt_()
{
    return m_alt_;
}

/*!
 * @brief This function sets a value in member yaw_
 * @param _yaw_ New value for member yaw_
 */
void position_setpoint::yaw_(float _yaw_)
{
m_yaw_ = _yaw_;
}

/*!
 * @brief This function returns the value of member yaw_
 * @return Value of member yaw_
 */
float position_setpoint::yaw_() const
{
    return m_yaw_;
}

/*!
 * @brief This function returns a reference to member yaw_
 * @return Reference to member yaw_
 */
float& position_setpoint::yaw_()
{
    return m_yaw_;
}

/*!
 * @brief This function sets a value in member yaw_valid_
 * @param _yaw_valid_ New value for member yaw_valid_
 */
void position_setpoint::yaw_valid_(bool _yaw_valid_)
{
m_yaw_valid_ = _yaw_valid_;
}

/*!
 * @brief This function returns the value of member yaw_valid_
 * @return Value of member yaw_valid_
 */
bool position_setpoint::yaw_valid_() const
{
    return m_yaw_valid_;
}

/*!
 * @brief This function returns a reference to member yaw_valid_
 * @return Reference to member yaw_valid_
 */
bool& position_setpoint::yaw_valid_()
{
    return m_yaw_valid_;
}

/*!
 * @brief This function sets a value in member yawspeed_
 * @param _yawspeed_ New value for member yawspeed_
 */
void position_setpoint::yawspeed_(float _yawspeed_)
{
m_yawspeed_ = _yawspeed_;
}

/*!
 * @brief This function returns the value of member yawspeed_
 * @return Value of member yawspeed_
 */
float position_setpoint::yawspeed_() const
{
    return m_yawspeed_;
}

/*!
 * @brief This function returns a reference to member yawspeed_
 * @return Reference to member yawspeed_
 */
float& position_setpoint::yawspeed_()
{
    return m_yawspeed_;
}

/*!
 * @brief This function sets a value in member yawspeed_valid_
 * @param _yawspeed_valid_ New value for member yawspeed_valid_
 */
void position_setpoint::yawspeed_valid_(bool _yawspeed_valid_)
{
m_yawspeed_valid_ = _yawspeed_valid_;
}

/*!
 * @brief This function returns the value of member yawspeed_valid_
 * @return Value of member yawspeed_valid_
 */
bool position_setpoint::yawspeed_valid_() const
{
    return m_yawspeed_valid_;
}

/*!
 * @brief This function returns a reference to member yawspeed_valid_
 * @return Reference to member yawspeed_valid_
 */
bool& position_setpoint::yawspeed_valid_()
{
    return m_yawspeed_valid_;
}

/*!
 * @brief This function sets a value in member landing_gear_
 * @param _landing_gear_ New value for member landing_gear_
 */
void position_setpoint::landing_gear_(uint8_t _landing_gear_)
{
m_landing_gear_ = _landing_gear_;
}

/*!
 * @brief This function returns the value of member landing_gear_
 * @return Value of member landing_gear_
 */
uint8_t position_setpoint::landing_gear_() const
{
    return m_landing_gear_;
}

/*!
 * @brief This function returns a reference to member landing_gear_
 * @return Reference to member landing_gear_
 */
uint8_t& position_setpoint::landing_gear_()
{
    return m_landing_gear_;
}

/*!
 * @brief This function sets a value in member loiter_radius_
 * @param _loiter_radius_ New value for member loiter_radius_
 */
void position_setpoint::loiter_radius_(float _loiter_radius_)
{
m_loiter_radius_ = _loiter_radius_;
}

/*!
 * @brief This function returns the value of member loiter_radius_
 * @return Value of member loiter_radius_
 */
float position_setpoint::loiter_radius_() const
{
    return m_loiter_radius_;
}

/*!
 * @brief This function returns a reference to member loiter_radius_
 * @return Reference to member loiter_radius_
 */
float& position_setpoint::loiter_radius_()
{
    return m_loiter_radius_;
}

/*!
 * @brief This function sets a value in member loiter_direction_
 * @param _loiter_direction_ New value for member loiter_direction_
 */
void position_setpoint::loiter_direction_(uint8_t _loiter_direction_)
{
m_loiter_direction_ = _loiter_direction_;
}

/*!
 * @brief This function returns the value of member loiter_direction_
 * @return Value of member loiter_direction_
 */
uint8_t position_setpoint::loiter_direction_() const
{
    return m_loiter_direction_;
}

/*!
 * @brief This function returns a reference to member loiter_direction_
 * @return Reference to member loiter_direction_
 */
uint8_t& position_setpoint::loiter_direction_()
{
    return m_loiter_direction_;
}

/*!
 * @brief This function sets a value in member acceptance_radius_
 * @param _acceptance_radius_ New value for member acceptance_radius_
 */
void position_setpoint::acceptance_radius_(float _acceptance_radius_)
{
m_acceptance_radius_ = _acceptance_radius_;
}

/*!
 * @brief This function returns the value of member acceptance_radius_
 * @return Value of member acceptance_radius_
 */
float position_setpoint::acceptance_radius_() const
{
    return m_acceptance_radius_;
}

/*!
 * @brief This function returns a reference to member acceptance_radius_
 * @return Reference to member acceptance_radius_
 */
float& position_setpoint::acceptance_radius_()
{
    return m_acceptance_radius_;
}

/*!
 * @brief This function sets a value in member cruising_speed_
 * @param _cruising_speed_ New value for member cruising_speed_
 */
void position_setpoint::cruising_speed_(float _cruising_speed_)
{
m_cruising_speed_ = _cruising_speed_;
}

/*!
 * @brief This function returns the value of member cruising_speed_
 * @return Value of member cruising_speed_
 */
float position_setpoint::cruising_speed_() const
{
    return m_cruising_speed_;
}

/*!
 * @brief This function returns a reference to member cruising_speed_
 * @return Reference to member cruising_speed_
 */
float& position_setpoint::cruising_speed_()
{
    return m_cruising_speed_;
}

/*!
 * @brief This function sets a value in member cruising_throttle_
 * @param _cruising_throttle_ New value for member cruising_throttle_
 */
void position_setpoint::cruising_throttle_(float _cruising_throttle_)
{
m_cruising_throttle_ = _cruising_throttle_;
}

/*!
 * @brief This function returns the value of member cruising_throttle_
 * @return Value of member cruising_throttle_
 */
float position_setpoint::cruising_throttle_() const
{
    return m_cruising_throttle_;
}

/*!
 * @brief This function returns a reference to member cruising_throttle_
 * @return Reference to member cruising_throttle_
 */
float& position_setpoint::cruising_throttle_()
{
    return m_cruising_throttle_;
}

/*!
 * @brief This function sets a value in member disable_weather_vane_
 * @param _disable_weather_vane_ New value for member disable_weather_vane_
 */
void position_setpoint::disable_weather_vane_(bool _disable_weather_vane_)
{
m_disable_weather_vane_ = _disable_weather_vane_;
}

/*!
 * @brief This function returns the value of member disable_weather_vane_
 * @return Value of member disable_weather_vane_
 */
bool position_setpoint::disable_weather_vane_() const
{
    return m_disable_weather_vane_;
}

/*!
 * @brief This function returns a reference to member disable_weather_vane_
 * @return Reference to member disable_weather_vane_
 */
bool& position_setpoint::disable_weather_vane_()
{
    return m_disable_weather_vane_;
}


size_t position_setpoint::getKeyMaxCdrSerializedSize(size_t current_alignment)
{
    size_t current_align = current_alignment;


























    return current_align;
}

bool position_setpoint::isKeyDefined()
{
   return false;
}

void position_setpoint::serializeKey(eprosima::fastcdr::Cdr &scdr) const
{
    (void) scdr;
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
}
