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
 * @file sensor_combined.cpp
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace { char dummy; }
#endif

#include "sensor_combined.h"
#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>






sensor_combined::sensor_combined()
{
    // m_timestamp_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@56620197
    m_timestamp_ = 0;
    // m_gyro_rad com.eprosima.idl.parser.typecode.AliasTypeCode@6eda5c9
    memset(&m_gyro_rad, 0, (3) * 4);
    // m_gyro_integral_dt_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@55b7a4e0
    m_gyro_integral_dt_ = 0;
    // m_accelerometer_timestamp_relative_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@5f058f00
    m_accelerometer_timestamp_relative_ = 0;
    // m_accelerometer_m_s2 com.eprosima.idl.parser.typecode.AliasTypeCode@6eda5c9
    memset(&m_accelerometer_m_s2, 0, (3) * 4);
    // m_accelerometer_integral_dt_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@192d43ce
    m_accelerometer_integral_dt_ = 0;
    // m_accelerometer_clipping_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@72057ecf
    m_accelerometer_clipping_ = 0;
    // m_gyro_clipping_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@1afd44cb
    m_gyro_clipping_ = 0;
    // m_accel_calibration_count_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@6973b51b
    m_accel_calibration_count_ = 0;
    // m_gyro_calibration_count_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@1ab3a8c8
    m_gyro_calibration_count_ = 0;

}

sensor_combined::~sensor_combined()
{










}

sensor_combined::sensor_combined(const sensor_combined &x)
{
    m_timestamp_ = x.m_timestamp_;
    m_gyro_rad = x.m_gyro_rad;
    m_gyro_integral_dt_ = x.m_gyro_integral_dt_;
    m_accelerometer_timestamp_relative_ = x.m_accelerometer_timestamp_relative_;
    m_accelerometer_m_s2 = x.m_accelerometer_m_s2;
    m_accelerometer_integral_dt_ = x.m_accelerometer_integral_dt_;
    m_accelerometer_clipping_ = x.m_accelerometer_clipping_;
    m_gyro_clipping_ = x.m_gyro_clipping_;
    m_accel_calibration_count_ = x.m_accel_calibration_count_;
    m_gyro_calibration_count_ = x.m_gyro_calibration_count_;
}

sensor_combined::sensor_combined(sensor_combined &&x)
{
    m_timestamp_ = x.m_timestamp_;
    m_gyro_rad = std::move(x.m_gyro_rad);
    m_gyro_integral_dt_ = x.m_gyro_integral_dt_;
    m_accelerometer_timestamp_relative_ = x.m_accelerometer_timestamp_relative_;
    m_accelerometer_m_s2 = std::move(x.m_accelerometer_m_s2);
    m_accelerometer_integral_dt_ = x.m_accelerometer_integral_dt_;
    m_accelerometer_clipping_ = x.m_accelerometer_clipping_;
    m_gyro_clipping_ = x.m_gyro_clipping_;
    m_accel_calibration_count_ = x.m_accel_calibration_count_;
    m_gyro_calibration_count_ = x.m_gyro_calibration_count_;
}

sensor_combined& sensor_combined::operator=(const sensor_combined &x)
{

    m_timestamp_ = x.m_timestamp_;
    m_gyro_rad = x.m_gyro_rad;
    m_gyro_integral_dt_ = x.m_gyro_integral_dt_;
    m_accelerometer_timestamp_relative_ = x.m_accelerometer_timestamp_relative_;
    m_accelerometer_m_s2 = x.m_accelerometer_m_s2;
    m_accelerometer_integral_dt_ = x.m_accelerometer_integral_dt_;
    m_accelerometer_clipping_ = x.m_accelerometer_clipping_;
    m_gyro_clipping_ = x.m_gyro_clipping_;
    m_accel_calibration_count_ = x.m_accel_calibration_count_;
    m_gyro_calibration_count_ = x.m_gyro_calibration_count_;

    return *this;
}

sensor_combined& sensor_combined::operator=(sensor_combined &&x)
{

    m_timestamp_ = x.m_timestamp_;
    m_gyro_rad = std::move(x.m_gyro_rad);
    m_gyro_integral_dt_ = x.m_gyro_integral_dt_;
    m_accelerometer_timestamp_relative_ = x.m_accelerometer_timestamp_relative_;
    m_accelerometer_m_s2 = std::move(x.m_accelerometer_m_s2);
    m_accelerometer_integral_dt_ = x.m_accelerometer_integral_dt_;
    m_accelerometer_clipping_ = x.m_accelerometer_clipping_;
    m_gyro_clipping_ = x.m_gyro_clipping_;
    m_accel_calibration_count_ = x.m_accel_calibration_count_;
    m_gyro_calibration_count_ = x.m_gyro_calibration_count_;

    return *this;
}

size_t sensor_combined::getMaxCdrSerializedSize(size_t current_alignment)
{
    size_t initial_alignment = current_alignment;


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += ((3) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += ((3) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);



    return current_alignment - initial_alignment;
}

size_t sensor_combined::getCdrSerializedSize(const sensor_combined& data, size_t current_alignment)
{
    (void)data;
    size_t initial_alignment = current_alignment;


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    if ((3) > 0)
    {
        current_alignment += ((3) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    }

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    if ((3) > 0)
    {
        current_alignment += ((3) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    }

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);



    return current_alignment - initial_alignment;
}

void sensor_combined::serialize(eprosima::fastcdr::Cdr &scdr) const
{

    scdr << m_timestamp_;
    scdr << m_gyro_rad;

    scdr << m_gyro_integral_dt_;
    scdr << m_accelerometer_timestamp_relative_;
    scdr << m_accelerometer_m_s2;

    scdr << m_accelerometer_integral_dt_;
    scdr << m_accelerometer_clipping_;
    scdr << m_gyro_clipping_;
    scdr << m_accel_calibration_count_;
    scdr << m_gyro_calibration_count_;
}

void sensor_combined::deserialize(eprosima::fastcdr::Cdr &dcdr)
{

    dcdr >> m_timestamp_;
    dcdr >> m_gyro_rad;

    dcdr >> m_gyro_integral_dt_;
    dcdr >> m_accelerometer_timestamp_relative_;
    dcdr >> m_accelerometer_m_s2;

    dcdr >> m_accelerometer_integral_dt_;
    dcdr >> m_accelerometer_clipping_;
    dcdr >> m_gyro_clipping_;
    dcdr >> m_accel_calibration_count_;
    dcdr >> m_gyro_calibration_count_;
}

/*!
 * @brief This function sets a value in member timestamp_
 * @param _timestamp_ New value for member timestamp_
 */
void sensor_combined::timestamp_(uint64_t _timestamp_)
{
m_timestamp_ = _timestamp_;
}

/*!
 * @brief This function returns the value of member timestamp_
 * @return Value of member timestamp_
 */
uint64_t sensor_combined::timestamp_() const
{
    return m_timestamp_;
}

/*!
 * @brief This function returns a reference to member timestamp_
 * @return Reference to member timestamp_
 */
uint64_t& sensor_combined::timestamp_()
{
    return m_timestamp_;
}

/*!
 * @brief This function copies the value in member gyro_rad
 * @param _gyro_rad New value to be copied in member gyro_rad
 */
void sensor_combined::gyro_rad(const sensor_combined__float_array_3 &_gyro_rad)
{
m_gyro_rad = _gyro_rad;
}

/*!
 * @brief This function moves the value in member gyro_rad
 * @param _gyro_rad New value to be moved in member gyro_rad
 */
void sensor_combined::gyro_rad(sensor_combined__float_array_3 &&_gyro_rad)
{
m_gyro_rad = std::move(_gyro_rad);
}

/*!
 * @brief This function returns a constant reference to member gyro_rad
 * @return Constant reference to member gyro_rad
 */
const sensor_combined__float_array_3& sensor_combined::gyro_rad() const
{
    return m_gyro_rad;
}

/*!
 * @brief This function returns a reference to member gyro_rad
 * @return Reference to member gyro_rad
 */
sensor_combined__float_array_3& sensor_combined::gyro_rad()
{
    return m_gyro_rad;
}
/*!
 * @brief This function sets a value in member gyro_integral_dt_
 * @param _gyro_integral_dt_ New value for member gyro_integral_dt_
 */
void sensor_combined::gyro_integral_dt_(uint32_t _gyro_integral_dt_)
{
m_gyro_integral_dt_ = _gyro_integral_dt_;
}

/*!
 * @brief This function returns the value of member gyro_integral_dt_
 * @return Value of member gyro_integral_dt_
 */
uint32_t sensor_combined::gyro_integral_dt_() const
{
    return m_gyro_integral_dt_;
}

/*!
 * @brief This function returns a reference to member gyro_integral_dt_
 * @return Reference to member gyro_integral_dt_
 */
uint32_t& sensor_combined::gyro_integral_dt_()
{
    return m_gyro_integral_dt_;
}

/*!
 * @brief This function sets a value in member accelerometer_timestamp_relative_
 * @param _accelerometer_timestamp_relative_ New value for member accelerometer_timestamp_relative_
 */
void sensor_combined::accelerometer_timestamp_relative_(int32_t _accelerometer_timestamp_relative_)
{
m_accelerometer_timestamp_relative_ = _accelerometer_timestamp_relative_;
}

/*!
 * @brief This function returns the value of member accelerometer_timestamp_relative_
 * @return Value of member accelerometer_timestamp_relative_
 */
int32_t sensor_combined::accelerometer_timestamp_relative_() const
{
    return m_accelerometer_timestamp_relative_;
}

/*!
 * @brief This function returns a reference to member accelerometer_timestamp_relative_
 * @return Reference to member accelerometer_timestamp_relative_
 */
int32_t& sensor_combined::accelerometer_timestamp_relative_()
{
    return m_accelerometer_timestamp_relative_;
}

/*!
 * @brief This function copies the value in member accelerometer_m_s2
 * @param _accelerometer_m_s2 New value to be copied in member accelerometer_m_s2
 */
void sensor_combined::accelerometer_m_s2(const sensor_combined__float_array_3 &_accelerometer_m_s2)
{
m_accelerometer_m_s2 = _accelerometer_m_s2;
}

/*!
 * @brief This function moves the value in member accelerometer_m_s2
 * @param _accelerometer_m_s2 New value to be moved in member accelerometer_m_s2
 */
void sensor_combined::accelerometer_m_s2(sensor_combined__float_array_3 &&_accelerometer_m_s2)
{
m_accelerometer_m_s2 = std::move(_accelerometer_m_s2);
}

/*!
 * @brief This function returns a constant reference to member accelerometer_m_s2
 * @return Constant reference to member accelerometer_m_s2
 */
const sensor_combined__float_array_3& sensor_combined::accelerometer_m_s2() const
{
    return m_accelerometer_m_s2;
}

/*!
 * @brief This function returns a reference to member accelerometer_m_s2
 * @return Reference to member accelerometer_m_s2
 */
sensor_combined__float_array_3& sensor_combined::accelerometer_m_s2()
{
    return m_accelerometer_m_s2;
}
/*!
 * @brief This function sets a value in member accelerometer_integral_dt_
 * @param _accelerometer_integral_dt_ New value for member accelerometer_integral_dt_
 */
void sensor_combined::accelerometer_integral_dt_(uint32_t _accelerometer_integral_dt_)
{
m_accelerometer_integral_dt_ = _accelerometer_integral_dt_;
}

/*!
 * @brief This function returns the value of member accelerometer_integral_dt_
 * @return Value of member accelerometer_integral_dt_
 */
uint32_t sensor_combined::accelerometer_integral_dt_() const
{
    return m_accelerometer_integral_dt_;
}

/*!
 * @brief This function returns a reference to member accelerometer_integral_dt_
 * @return Reference to member accelerometer_integral_dt_
 */
uint32_t& sensor_combined::accelerometer_integral_dt_()
{
    return m_accelerometer_integral_dt_;
}

/*!
 * @brief This function sets a value in member accelerometer_clipping_
 * @param _accelerometer_clipping_ New value for member accelerometer_clipping_
 */
void sensor_combined::accelerometer_clipping_(uint8_t _accelerometer_clipping_)
{
m_accelerometer_clipping_ = _accelerometer_clipping_;
}

/*!
 * @brief This function returns the value of member accelerometer_clipping_
 * @return Value of member accelerometer_clipping_
 */
uint8_t sensor_combined::accelerometer_clipping_() const
{
    return m_accelerometer_clipping_;
}

/*!
 * @brief This function returns a reference to member accelerometer_clipping_
 * @return Reference to member accelerometer_clipping_
 */
uint8_t& sensor_combined::accelerometer_clipping_()
{
    return m_accelerometer_clipping_;
}

/*!
 * @brief This function sets a value in member gyro_clipping_
 * @param _gyro_clipping_ New value for member gyro_clipping_
 */
void sensor_combined::gyro_clipping_(uint8_t _gyro_clipping_)
{
m_gyro_clipping_ = _gyro_clipping_;
}

/*!
 * @brief This function returns the value of member gyro_clipping_
 * @return Value of member gyro_clipping_
 */
uint8_t sensor_combined::gyro_clipping_() const
{
    return m_gyro_clipping_;
}

/*!
 * @brief This function returns a reference to member gyro_clipping_
 * @return Reference to member gyro_clipping_
 */
uint8_t& sensor_combined::gyro_clipping_()
{
    return m_gyro_clipping_;
}

/*!
 * @brief This function sets a value in member accel_calibration_count_
 * @param _accel_calibration_count_ New value for member accel_calibration_count_
 */
void sensor_combined::accel_calibration_count_(uint8_t _accel_calibration_count_)
{
m_accel_calibration_count_ = _accel_calibration_count_;
}

/*!
 * @brief This function returns the value of member accel_calibration_count_
 * @return Value of member accel_calibration_count_
 */
uint8_t sensor_combined::accel_calibration_count_() const
{
    return m_accel_calibration_count_;
}

/*!
 * @brief This function returns a reference to member accel_calibration_count_
 * @return Reference to member accel_calibration_count_
 */
uint8_t& sensor_combined::accel_calibration_count_()
{
    return m_accel_calibration_count_;
}

/*!
 * @brief This function sets a value in member gyro_calibration_count_
 * @param _gyro_calibration_count_ New value for member gyro_calibration_count_
 */
void sensor_combined::gyro_calibration_count_(uint8_t _gyro_calibration_count_)
{
m_gyro_calibration_count_ = _gyro_calibration_count_;
}

/*!
 * @brief This function returns the value of member gyro_calibration_count_
 * @return Value of member gyro_calibration_count_
 */
uint8_t sensor_combined::gyro_calibration_count_() const
{
    return m_gyro_calibration_count_;
}

/*!
 * @brief This function returns a reference to member gyro_calibration_count_
 * @return Reference to member gyro_calibration_count_
 */
uint8_t& sensor_combined::gyro_calibration_count_()
{
    return m_gyro_calibration_count_;
}


size_t sensor_combined::getKeyMaxCdrSerializedSize(size_t current_alignment)
{
    size_t current_align = current_alignment;













    return current_align;
}

bool sensor_combined::isKeyDefined()
{
   return false;
}

void sensor_combined::serializeKey(eprosima::fastcdr::Cdr &scdr) const
{
    (void) scdr;
     
     
     
     
     
     
     
     
     
     
}
