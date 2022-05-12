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
 * @file offboard_control_mode.cpp
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace { char dummy; }
#endif

#include "offboard_control_mode.h"
#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>

offboard_control_mode::offboard_control_mode()
{
    // m_timestamp_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@44c03695
    m_timestamp_ = 0;
    // m_position_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@7e6f74c
    m_position_ = false;
    // m_velocity_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@dd05255
    m_velocity_ = false;
    // m_acceleration_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@6a78afa0
    m_acceleration_ = false;
    // m_attitude_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@2f4948e4
    m_attitude_ = false;
    // m_body_rate_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@1f2586d6
    m_body_rate_ = false;
    // m_actuator_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@10683d9d
    m_actuator_ = false;

}

offboard_control_mode::~offboard_control_mode()
{







}

offboard_control_mode::offboard_control_mode(const offboard_control_mode &x)
{
    m_timestamp_ = x.m_timestamp_;
    m_position_ = x.m_position_;
    m_velocity_ = x.m_velocity_;
    m_acceleration_ = x.m_acceleration_;
    m_attitude_ = x.m_attitude_;
    m_body_rate_ = x.m_body_rate_;
    m_actuator_ = x.m_actuator_;
}

offboard_control_mode::offboard_control_mode(offboard_control_mode &&x)
{
    m_timestamp_ = x.m_timestamp_;
    m_position_ = x.m_position_;
    m_velocity_ = x.m_velocity_;
    m_acceleration_ = x.m_acceleration_;
    m_attitude_ = x.m_attitude_;
    m_body_rate_ = x.m_body_rate_;
    m_actuator_ = x.m_actuator_;
}

offboard_control_mode& offboard_control_mode::operator=(const offboard_control_mode &x)
{

    m_timestamp_ = x.m_timestamp_;
    m_position_ = x.m_position_;
    m_velocity_ = x.m_velocity_;
    m_acceleration_ = x.m_acceleration_;
    m_attitude_ = x.m_attitude_;
    m_body_rate_ = x.m_body_rate_;
    m_actuator_ = x.m_actuator_;

    return *this;
}

offboard_control_mode& offboard_control_mode::operator=(offboard_control_mode &&x)
{

    m_timestamp_ = x.m_timestamp_;
    m_position_ = x.m_position_;
    m_velocity_ = x.m_velocity_;
    m_acceleration_ = x.m_acceleration_;
    m_attitude_ = x.m_attitude_;
    m_body_rate_ = x.m_body_rate_;
    m_actuator_ = x.m_actuator_;

    return *this;
}

size_t offboard_control_mode::getMaxCdrSerializedSize(size_t current_alignment)
{
    size_t initial_alignment = current_alignment;


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);



    return current_alignment - initial_alignment;
}

size_t offboard_control_mode::getCdrSerializedSize(const offboard_control_mode& data, size_t current_alignment)
{
    (void)data;
    size_t initial_alignment = current_alignment;


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);



    return current_alignment - initial_alignment;
}

void offboard_control_mode::serialize(eprosima::fastcdr::Cdr &scdr) const
{

    scdr << m_timestamp_;
    scdr << m_position_;
    scdr << m_velocity_;
    scdr << m_acceleration_;
    scdr << m_attitude_;
    scdr << m_body_rate_;
    scdr << m_actuator_;
}

void offboard_control_mode::deserialize(eprosima::fastcdr::Cdr &dcdr)
{

    dcdr >> m_timestamp_;
    dcdr >> m_position_;
    dcdr >> m_velocity_;
    dcdr >> m_acceleration_;
    dcdr >> m_attitude_;
    dcdr >> m_body_rate_;
    dcdr >> m_actuator_;
}

/*!
 * @brief This function sets a value in member timestamp_
 * @param _timestamp_ New value for member timestamp_
 */
void offboard_control_mode::timestamp_(uint64_t _timestamp_)
{
m_timestamp_ = _timestamp_;
}

/*!
 * @brief This function returns the value of member timestamp_
 * @return Value of member timestamp_
 */
uint64_t offboard_control_mode::timestamp_() const
{
    return m_timestamp_;
}

/*!
 * @brief This function returns a reference to member timestamp_
 * @return Reference to member timestamp_
 */
uint64_t& offboard_control_mode::timestamp_()
{
    return m_timestamp_;
}

/*!
 * @brief This function sets a value in member position_
 * @param _position_ New value for member position_
 */
void offboard_control_mode::position_(bool _position_)
{
m_position_ = _position_;
}

/*!
 * @brief This function returns the value of member position_
 * @return Value of member position_
 */
bool offboard_control_mode::position_() const
{
    return m_position_;
}

/*!
 * @brief This function returns a reference to member position_
 * @return Reference to member position_
 */
bool& offboard_control_mode::position_()
{
    return m_position_;
}

/*!
 * @brief This function sets a value in member velocity_
 * @param _velocity_ New value for member velocity_
 */
void offboard_control_mode::velocity_(bool _velocity_)
{
m_velocity_ = _velocity_;
}

/*!
 * @brief This function returns the value of member velocity_
 * @return Value of member velocity_
 */
bool offboard_control_mode::velocity_() const
{
    return m_velocity_;
}

/*!
 * @brief This function returns a reference to member velocity_
 * @return Reference to member velocity_
 */
bool& offboard_control_mode::velocity_()
{
    return m_velocity_;
}

/*!
 * @brief This function sets a value in member acceleration_
 * @param _acceleration_ New value for member acceleration_
 */
void offboard_control_mode::acceleration_(bool _acceleration_)
{
m_acceleration_ = _acceleration_;
}

/*!
 * @brief This function returns the value of member acceleration_
 * @return Value of member acceleration_
 */
bool offboard_control_mode::acceleration_() const
{
    return m_acceleration_;
}

/*!
 * @brief This function returns a reference to member acceleration_
 * @return Reference to member acceleration_
 */
bool& offboard_control_mode::acceleration_()
{
    return m_acceleration_;
}

/*!
 * @brief This function sets a value in member attitude_
 * @param _attitude_ New value for member attitude_
 */
void offboard_control_mode::attitude_(bool _attitude_)
{
m_attitude_ = _attitude_;
}

/*!
 * @brief This function returns the value of member attitude_
 * @return Value of member attitude_
 */
bool offboard_control_mode::attitude_() const
{
    return m_attitude_;
}

/*!
 * @brief This function returns a reference to member attitude_
 * @return Reference to member attitude_
 */
bool& offboard_control_mode::attitude_()
{
    return m_attitude_;
}

/*!
 * @brief This function sets a value in member body_rate_
 * @param _body_rate_ New value for member body_rate_
 */
void offboard_control_mode::body_rate_(bool _body_rate_)
{
m_body_rate_ = _body_rate_;
}

/*!
 * @brief This function returns the value of member body_rate_
 * @return Value of member body_rate_
 */
bool offboard_control_mode::body_rate_() const
{
    return m_body_rate_;
}

/*!
 * @brief This function returns a reference to member body_rate_
 * @return Reference to member body_rate_
 */
bool& offboard_control_mode::body_rate_()
{
    return m_body_rate_;
}

/*!
 * @brief This function sets a value in member actuator_
 * @param _actuator_ New value for member actuator_
 */
void offboard_control_mode::actuator_(bool _actuator_)
{
m_actuator_ = _actuator_;
}

/*!
 * @brief This function returns the value of member actuator_
 * @return Value of member actuator_
 */
bool offboard_control_mode::actuator_() const
{
    return m_actuator_;
}

/*!
 * @brief This function returns a reference to member actuator_
 * @return Reference to member actuator_
 */
bool& offboard_control_mode::actuator_()
{
    return m_actuator_;
}


size_t offboard_control_mode::getKeyMaxCdrSerializedSize(size_t current_alignment)
{
    size_t current_align = current_alignment;










    return current_align;
}

bool offboard_control_mode::isKeyDefined()
{
   return false;
}

void offboard_control_mode::serializeKey(eprosima::fastcdr::Cdr &scdr) const
{
    (void) scdr;
     
     
     
     
     
     
     
}
