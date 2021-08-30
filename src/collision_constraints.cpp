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
 * @file collision_constraints.cpp
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace { char dummy; }
#endif

#include "collision_constraints.h"
#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>


collision_constraints::collision_constraints()
{
    // m_timestamp_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@2cbb3d47
    m_timestamp_ = 0;
    // m_original_setpoint com.eprosima.idl.parser.typecode.AliasTypeCode@527e5409
    memset(&m_original_setpoint, 0, (2) * 4);
    // m_adapted_setpoint com.eprosima.idl.parser.typecode.AliasTypeCode@527e5409
    memset(&m_adapted_setpoint, 0, (2) * 4);

}

collision_constraints::~collision_constraints()
{



}

collision_constraints::collision_constraints(const collision_constraints &x)
{
    m_timestamp_ = x.m_timestamp_;
    m_original_setpoint = x.m_original_setpoint;
    m_adapted_setpoint = x.m_adapted_setpoint;
}

collision_constraints::collision_constraints(collision_constraints &&x)
{
    m_timestamp_ = x.m_timestamp_;
    m_original_setpoint = std::move(x.m_original_setpoint);
    m_adapted_setpoint = std::move(x.m_adapted_setpoint);
}

collision_constraints& collision_constraints::operator=(const collision_constraints &x)
{

    m_timestamp_ = x.m_timestamp_;
    m_original_setpoint = x.m_original_setpoint;
    m_adapted_setpoint = x.m_adapted_setpoint;

    return *this;
}

collision_constraints& collision_constraints::operator=(collision_constraints &&x)
{

    m_timestamp_ = x.m_timestamp_;
    m_original_setpoint = std::move(x.m_original_setpoint);
    m_adapted_setpoint = std::move(x.m_adapted_setpoint);

    return *this;
}

size_t collision_constraints::getMaxCdrSerializedSize(size_t current_alignment)
{
    size_t initial_alignment = current_alignment;


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += ((2) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += ((2) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);



    return current_alignment - initial_alignment;
}

size_t collision_constraints::getCdrSerializedSize(const collision_constraints& data, size_t current_alignment)
{
    (void)data;
    size_t initial_alignment = current_alignment;


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    if ((2) > 0)
    {
        current_alignment += ((2) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    }

    if ((2) > 0)
    {
        current_alignment += ((2) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    }


    return current_alignment - initial_alignment;
}

void collision_constraints::serialize(eprosima::fastcdr::Cdr &scdr) const
{

    scdr << m_timestamp_;
    scdr << m_original_setpoint;

    scdr << m_adapted_setpoint;

}

void collision_constraints::deserialize(eprosima::fastcdr::Cdr &dcdr)
{

    dcdr >> m_timestamp_;
    dcdr >> m_original_setpoint;

    dcdr >> m_adapted_setpoint;

}

/*!
 * @brief This function sets a value in member timestamp_
 * @param _timestamp_ New value for member timestamp_
 */
void collision_constraints::timestamp_(uint64_t _timestamp_)
{
m_timestamp_ = _timestamp_;
}

/*!
 * @brief This function returns the value of member timestamp_
 * @return Value of member timestamp_
 */
uint64_t collision_constraints::timestamp_() const
{
    return m_timestamp_;
}

/*!
 * @brief This function returns a reference to member timestamp_
 * @return Reference to member timestamp_
 */
uint64_t& collision_constraints::timestamp_()
{
    return m_timestamp_;
}

/*!
 * @brief This function copies the value in member original_setpoint
 * @param _original_setpoint New value to be copied in member original_setpoint
 */
void collision_constraints::original_setpoint(const collision_constraints__float_array_2 &_original_setpoint)
{
m_original_setpoint = _original_setpoint;
}

/*!
 * @brief This function moves the value in member original_setpoint
 * @param _original_setpoint New value to be moved in member original_setpoint
 */
void collision_constraints::original_setpoint(collision_constraints__float_array_2 &&_original_setpoint)
{
m_original_setpoint = std::move(_original_setpoint);
}

/*!
 * @brief This function returns a constant reference to member original_setpoint
 * @return Constant reference to member original_setpoint
 */
const collision_constraints__float_array_2& collision_constraints::original_setpoint() const
{
    return m_original_setpoint;
}

/*!
 * @brief This function returns a reference to member original_setpoint
 * @return Reference to member original_setpoint
 */
collision_constraints__float_array_2& collision_constraints::original_setpoint()
{
    return m_original_setpoint;
}
/*!
 * @brief This function copies the value in member adapted_setpoint
 * @param _adapted_setpoint New value to be copied in member adapted_setpoint
 */
void collision_constraints::adapted_setpoint(const collision_constraints__float_array_2 &_adapted_setpoint)
{
m_adapted_setpoint = _adapted_setpoint;
}

/*!
 * @brief This function moves the value in member adapted_setpoint
 * @param _adapted_setpoint New value to be moved in member adapted_setpoint
 */
void collision_constraints::adapted_setpoint(collision_constraints__float_array_2 &&_adapted_setpoint)
{
m_adapted_setpoint = std::move(_adapted_setpoint);
}

/*!
 * @brief This function returns a constant reference to member adapted_setpoint
 * @return Constant reference to member adapted_setpoint
 */
const collision_constraints__float_array_2& collision_constraints::adapted_setpoint() const
{
    return m_adapted_setpoint;
}

/*!
 * @brief This function returns a reference to member adapted_setpoint
 * @return Reference to member adapted_setpoint
 */
collision_constraints__float_array_2& collision_constraints::adapted_setpoint()
{
    return m_adapted_setpoint;
}

size_t collision_constraints::getKeyMaxCdrSerializedSize(size_t current_alignment)
{
    size_t current_align = current_alignment;






    return current_align;
}

bool collision_constraints::isKeyDefined()
{
   return false;
}

void collision_constraints::serializeKey(eprosima::fastcdr::Cdr &scdr) const
{
    (void) scdr;
     
     
     
}
