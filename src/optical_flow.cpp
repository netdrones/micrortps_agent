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
 * @file optical_flow.cpp
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace { char dummy; }
#endif

#include "optical_flow.h"
#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>





optical_flow::optical_flow()
{
    // m_timestamp_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@6572421
    m_timestamp_ = 0;
    // m_sensor_id_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@6b81ce95
    m_sensor_id_ = 0;
    // m_pixel_flow_x_integral_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@2a798d51
    m_pixel_flow_x_integral_ = 0.0;
    // m_pixel_flow_y_integral_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@6d763516
    m_pixel_flow_y_integral_ = 0.0;
    // m_gyro_x_rate_integral_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@52bf72b5
    m_gyro_x_rate_integral_ = 0.0;
    // m_gyro_y_rate_integral_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@37afeb11
    m_gyro_y_rate_integral_ = 0.0;
    // m_gyro_z_rate_integral_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@515aebb0
    m_gyro_z_rate_integral_ = 0.0;
    // m_ground_distance_m_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@dd8ba08
    m_ground_distance_m_ = 0.0;
    // m_integration_timespan_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@245b4bdc
    m_integration_timespan_ = 0;
    // m_time_since_last_sonar_update_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@6c64cb25
    m_time_since_last_sonar_update_ = 0;
    // m_frame_count_since_last_readout_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@6ae5aa72
    m_frame_count_since_last_readout_ = 0;
    // m_gyro_temperature_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@222545dc
    m_gyro_temperature_ = 0;
    // m_quality_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@5c5eefef
    m_quality_ = 0;
    // m_max_flow_rate_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@16293aa2
    m_max_flow_rate_ = 0.0;
    // m_min_ground_distance_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@5158b42f
    m_min_ground_distance_ = 0.0;
    // m_max_ground_distance_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@595b007d
    m_max_ground_distance_ = 0.0;
    // m_mode_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@72d1ad2e
    m_mode_ = 0;

}

optical_flow::~optical_flow()
{

















}

optical_flow::optical_flow(const optical_flow &x)
{
    m_timestamp_ = x.m_timestamp_;
    m_sensor_id_ = x.m_sensor_id_;
    m_pixel_flow_x_integral_ = x.m_pixel_flow_x_integral_;
    m_pixel_flow_y_integral_ = x.m_pixel_flow_y_integral_;
    m_gyro_x_rate_integral_ = x.m_gyro_x_rate_integral_;
    m_gyro_y_rate_integral_ = x.m_gyro_y_rate_integral_;
    m_gyro_z_rate_integral_ = x.m_gyro_z_rate_integral_;
    m_ground_distance_m_ = x.m_ground_distance_m_;
    m_integration_timespan_ = x.m_integration_timespan_;
    m_time_since_last_sonar_update_ = x.m_time_since_last_sonar_update_;
    m_frame_count_since_last_readout_ = x.m_frame_count_since_last_readout_;
    m_gyro_temperature_ = x.m_gyro_temperature_;
    m_quality_ = x.m_quality_;
    m_max_flow_rate_ = x.m_max_flow_rate_;
    m_min_ground_distance_ = x.m_min_ground_distance_;
    m_max_ground_distance_ = x.m_max_ground_distance_;
    m_mode_ = x.m_mode_;
}

optical_flow::optical_flow(optical_flow &&x)
{
    m_timestamp_ = x.m_timestamp_;
    m_sensor_id_ = x.m_sensor_id_;
    m_pixel_flow_x_integral_ = x.m_pixel_flow_x_integral_;
    m_pixel_flow_y_integral_ = x.m_pixel_flow_y_integral_;
    m_gyro_x_rate_integral_ = x.m_gyro_x_rate_integral_;
    m_gyro_y_rate_integral_ = x.m_gyro_y_rate_integral_;
    m_gyro_z_rate_integral_ = x.m_gyro_z_rate_integral_;
    m_ground_distance_m_ = x.m_ground_distance_m_;
    m_integration_timespan_ = x.m_integration_timespan_;
    m_time_since_last_sonar_update_ = x.m_time_since_last_sonar_update_;
    m_frame_count_since_last_readout_ = x.m_frame_count_since_last_readout_;
    m_gyro_temperature_ = x.m_gyro_temperature_;
    m_quality_ = x.m_quality_;
    m_max_flow_rate_ = x.m_max_flow_rate_;
    m_min_ground_distance_ = x.m_min_ground_distance_;
    m_max_ground_distance_ = x.m_max_ground_distance_;
    m_mode_ = x.m_mode_;
}

optical_flow& optical_flow::operator=(const optical_flow &x)
{

    m_timestamp_ = x.m_timestamp_;
    m_sensor_id_ = x.m_sensor_id_;
    m_pixel_flow_x_integral_ = x.m_pixel_flow_x_integral_;
    m_pixel_flow_y_integral_ = x.m_pixel_flow_y_integral_;
    m_gyro_x_rate_integral_ = x.m_gyro_x_rate_integral_;
    m_gyro_y_rate_integral_ = x.m_gyro_y_rate_integral_;
    m_gyro_z_rate_integral_ = x.m_gyro_z_rate_integral_;
    m_ground_distance_m_ = x.m_ground_distance_m_;
    m_integration_timespan_ = x.m_integration_timespan_;
    m_time_since_last_sonar_update_ = x.m_time_since_last_sonar_update_;
    m_frame_count_since_last_readout_ = x.m_frame_count_since_last_readout_;
    m_gyro_temperature_ = x.m_gyro_temperature_;
    m_quality_ = x.m_quality_;
    m_max_flow_rate_ = x.m_max_flow_rate_;
    m_min_ground_distance_ = x.m_min_ground_distance_;
    m_max_ground_distance_ = x.m_max_ground_distance_;
    m_mode_ = x.m_mode_;

    return *this;
}

optical_flow& optical_flow::operator=(optical_flow &&x)
{

    m_timestamp_ = x.m_timestamp_;
    m_sensor_id_ = x.m_sensor_id_;
    m_pixel_flow_x_integral_ = x.m_pixel_flow_x_integral_;
    m_pixel_flow_y_integral_ = x.m_pixel_flow_y_integral_;
    m_gyro_x_rate_integral_ = x.m_gyro_x_rate_integral_;
    m_gyro_y_rate_integral_ = x.m_gyro_y_rate_integral_;
    m_gyro_z_rate_integral_ = x.m_gyro_z_rate_integral_;
    m_ground_distance_m_ = x.m_ground_distance_m_;
    m_integration_timespan_ = x.m_integration_timespan_;
    m_time_since_last_sonar_update_ = x.m_time_since_last_sonar_update_;
    m_frame_count_since_last_readout_ = x.m_frame_count_since_last_readout_;
    m_gyro_temperature_ = x.m_gyro_temperature_;
    m_quality_ = x.m_quality_;
    m_max_flow_rate_ = x.m_max_flow_rate_;
    m_min_ground_distance_ = x.m_min_ground_distance_;
    m_max_ground_distance_ = x.m_max_ground_distance_;
    m_mode_ = x.m_mode_;

    return *this;
}

size_t optical_flow::getMaxCdrSerializedSize(size_t current_alignment)
{
    size_t initial_alignment = current_alignment;


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);


    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);



    return current_alignment - initial_alignment;
}

size_t optical_flow::getCdrSerializedSize(const optical_flow& data, size_t current_alignment)
{
    (void)data;
    size_t initial_alignment = current_alignment;


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);


    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);



    return current_alignment - initial_alignment;
}

void optical_flow::serialize(eprosima::fastcdr::Cdr &scdr) const
{

    scdr << m_timestamp_;
    scdr << m_sensor_id_;
    scdr << m_pixel_flow_x_integral_;
    scdr << m_pixel_flow_y_integral_;
    scdr << m_gyro_x_rate_integral_;
    scdr << m_gyro_y_rate_integral_;
    scdr << m_gyro_z_rate_integral_;
    scdr << m_ground_distance_m_;
    scdr << m_integration_timespan_;
    scdr << m_time_since_last_sonar_update_;
    scdr << m_frame_count_since_last_readout_;
    scdr << m_gyro_temperature_;
    scdr << m_quality_;
    scdr << m_max_flow_rate_;
    scdr << m_min_ground_distance_;
    scdr << m_max_ground_distance_;
    scdr << m_mode_;
}

void optical_flow::deserialize(eprosima::fastcdr::Cdr &dcdr)
{

    dcdr >> m_timestamp_;
    dcdr >> m_sensor_id_;
    dcdr >> m_pixel_flow_x_integral_;
    dcdr >> m_pixel_flow_y_integral_;
    dcdr >> m_gyro_x_rate_integral_;
    dcdr >> m_gyro_y_rate_integral_;
    dcdr >> m_gyro_z_rate_integral_;
    dcdr >> m_ground_distance_m_;
    dcdr >> m_integration_timespan_;
    dcdr >> m_time_since_last_sonar_update_;
    dcdr >> m_frame_count_since_last_readout_;
    dcdr >> m_gyro_temperature_;
    dcdr >> m_quality_;
    dcdr >> m_max_flow_rate_;
    dcdr >> m_min_ground_distance_;
    dcdr >> m_max_ground_distance_;
    dcdr >> m_mode_;
}

/*!
 * @brief This function sets a value in member timestamp_
 * @param _timestamp_ New value for member timestamp_
 */
void optical_flow::timestamp_(uint64_t _timestamp_)
{
m_timestamp_ = _timestamp_;
}

/*!
 * @brief This function returns the value of member timestamp_
 * @return Value of member timestamp_
 */
uint64_t optical_flow::timestamp_() const
{
    return m_timestamp_;
}

/*!
 * @brief This function returns a reference to member timestamp_
 * @return Reference to member timestamp_
 */
uint64_t& optical_flow::timestamp_()
{
    return m_timestamp_;
}

/*!
 * @brief This function sets a value in member sensor_id_
 * @param _sensor_id_ New value for member sensor_id_
 */
void optical_flow::sensor_id_(uint8_t _sensor_id_)
{
m_sensor_id_ = _sensor_id_;
}

/*!
 * @brief This function returns the value of member sensor_id_
 * @return Value of member sensor_id_
 */
uint8_t optical_flow::sensor_id_() const
{
    return m_sensor_id_;
}

/*!
 * @brief This function returns a reference to member sensor_id_
 * @return Reference to member sensor_id_
 */
uint8_t& optical_flow::sensor_id_()
{
    return m_sensor_id_;
}

/*!
 * @brief This function sets a value in member pixel_flow_x_integral_
 * @param _pixel_flow_x_integral_ New value for member pixel_flow_x_integral_
 */
void optical_flow::pixel_flow_x_integral_(float _pixel_flow_x_integral_)
{
m_pixel_flow_x_integral_ = _pixel_flow_x_integral_;
}

/*!
 * @brief This function returns the value of member pixel_flow_x_integral_
 * @return Value of member pixel_flow_x_integral_
 */
float optical_flow::pixel_flow_x_integral_() const
{
    return m_pixel_flow_x_integral_;
}

/*!
 * @brief This function returns a reference to member pixel_flow_x_integral_
 * @return Reference to member pixel_flow_x_integral_
 */
float& optical_flow::pixel_flow_x_integral_()
{
    return m_pixel_flow_x_integral_;
}

/*!
 * @brief This function sets a value in member pixel_flow_y_integral_
 * @param _pixel_flow_y_integral_ New value for member pixel_flow_y_integral_
 */
void optical_flow::pixel_flow_y_integral_(float _pixel_flow_y_integral_)
{
m_pixel_flow_y_integral_ = _pixel_flow_y_integral_;
}

/*!
 * @brief This function returns the value of member pixel_flow_y_integral_
 * @return Value of member pixel_flow_y_integral_
 */
float optical_flow::pixel_flow_y_integral_() const
{
    return m_pixel_flow_y_integral_;
}

/*!
 * @brief This function returns a reference to member pixel_flow_y_integral_
 * @return Reference to member pixel_flow_y_integral_
 */
float& optical_flow::pixel_flow_y_integral_()
{
    return m_pixel_flow_y_integral_;
}

/*!
 * @brief This function sets a value in member gyro_x_rate_integral_
 * @param _gyro_x_rate_integral_ New value for member gyro_x_rate_integral_
 */
void optical_flow::gyro_x_rate_integral_(float _gyro_x_rate_integral_)
{
m_gyro_x_rate_integral_ = _gyro_x_rate_integral_;
}

/*!
 * @brief This function returns the value of member gyro_x_rate_integral_
 * @return Value of member gyro_x_rate_integral_
 */
float optical_flow::gyro_x_rate_integral_() const
{
    return m_gyro_x_rate_integral_;
}

/*!
 * @brief This function returns a reference to member gyro_x_rate_integral_
 * @return Reference to member gyro_x_rate_integral_
 */
float& optical_flow::gyro_x_rate_integral_()
{
    return m_gyro_x_rate_integral_;
}

/*!
 * @brief This function sets a value in member gyro_y_rate_integral_
 * @param _gyro_y_rate_integral_ New value for member gyro_y_rate_integral_
 */
void optical_flow::gyro_y_rate_integral_(float _gyro_y_rate_integral_)
{
m_gyro_y_rate_integral_ = _gyro_y_rate_integral_;
}

/*!
 * @brief This function returns the value of member gyro_y_rate_integral_
 * @return Value of member gyro_y_rate_integral_
 */
float optical_flow::gyro_y_rate_integral_() const
{
    return m_gyro_y_rate_integral_;
}

/*!
 * @brief This function returns a reference to member gyro_y_rate_integral_
 * @return Reference to member gyro_y_rate_integral_
 */
float& optical_flow::gyro_y_rate_integral_()
{
    return m_gyro_y_rate_integral_;
}

/*!
 * @brief This function sets a value in member gyro_z_rate_integral_
 * @param _gyro_z_rate_integral_ New value for member gyro_z_rate_integral_
 */
void optical_flow::gyro_z_rate_integral_(float _gyro_z_rate_integral_)
{
m_gyro_z_rate_integral_ = _gyro_z_rate_integral_;
}

/*!
 * @brief This function returns the value of member gyro_z_rate_integral_
 * @return Value of member gyro_z_rate_integral_
 */
float optical_flow::gyro_z_rate_integral_() const
{
    return m_gyro_z_rate_integral_;
}

/*!
 * @brief This function returns a reference to member gyro_z_rate_integral_
 * @return Reference to member gyro_z_rate_integral_
 */
float& optical_flow::gyro_z_rate_integral_()
{
    return m_gyro_z_rate_integral_;
}

/*!
 * @brief This function sets a value in member ground_distance_m_
 * @param _ground_distance_m_ New value for member ground_distance_m_
 */
void optical_flow::ground_distance_m_(float _ground_distance_m_)
{
m_ground_distance_m_ = _ground_distance_m_;
}

/*!
 * @brief This function returns the value of member ground_distance_m_
 * @return Value of member ground_distance_m_
 */
float optical_flow::ground_distance_m_() const
{
    return m_ground_distance_m_;
}

/*!
 * @brief This function returns a reference to member ground_distance_m_
 * @return Reference to member ground_distance_m_
 */
float& optical_flow::ground_distance_m_()
{
    return m_ground_distance_m_;
}

/*!
 * @brief This function sets a value in member integration_timespan_
 * @param _integration_timespan_ New value for member integration_timespan_
 */
void optical_flow::integration_timespan_(uint32_t _integration_timespan_)
{
m_integration_timespan_ = _integration_timespan_;
}

/*!
 * @brief This function returns the value of member integration_timespan_
 * @return Value of member integration_timespan_
 */
uint32_t optical_flow::integration_timespan_() const
{
    return m_integration_timespan_;
}

/*!
 * @brief This function returns a reference to member integration_timespan_
 * @return Reference to member integration_timespan_
 */
uint32_t& optical_flow::integration_timespan_()
{
    return m_integration_timespan_;
}

/*!
 * @brief This function sets a value in member time_since_last_sonar_update_
 * @param _time_since_last_sonar_update_ New value for member time_since_last_sonar_update_
 */
void optical_flow::time_since_last_sonar_update_(uint32_t _time_since_last_sonar_update_)
{
m_time_since_last_sonar_update_ = _time_since_last_sonar_update_;
}

/*!
 * @brief This function returns the value of member time_since_last_sonar_update_
 * @return Value of member time_since_last_sonar_update_
 */
uint32_t optical_flow::time_since_last_sonar_update_() const
{
    return m_time_since_last_sonar_update_;
}

/*!
 * @brief This function returns a reference to member time_since_last_sonar_update_
 * @return Reference to member time_since_last_sonar_update_
 */
uint32_t& optical_flow::time_since_last_sonar_update_()
{
    return m_time_since_last_sonar_update_;
}

/*!
 * @brief This function sets a value in member frame_count_since_last_readout_
 * @param _frame_count_since_last_readout_ New value for member frame_count_since_last_readout_
 */
void optical_flow::frame_count_since_last_readout_(uint16_t _frame_count_since_last_readout_)
{
m_frame_count_since_last_readout_ = _frame_count_since_last_readout_;
}

/*!
 * @brief This function returns the value of member frame_count_since_last_readout_
 * @return Value of member frame_count_since_last_readout_
 */
uint16_t optical_flow::frame_count_since_last_readout_() const
{
    return m_frame_count_since_last_readout_;
}

/*!
 * @brief This function returns a reference to member frame_count_since_last_readout_
 * @return Reference to member frame_count_since_last_readout_
 */
uint16_t& optical_flow::frame_count_since_last_readout_()
{
    return m_frame_count_since_last_readout_;
}

/*!
 * @brief This function sets a value in member gyro_temperature_
 * @param _gyro_temperature_ New value for member gyro_temperature_
 */
void optical_flow::gyro_temperature_(int16_t _gyro_temperature_)
{
m_gyro_temperature_ = _gyro_temperature_;
}

/*!
 * @brief This function returns the value of member gyro_temperature_
 * @return Value of member gyro_temperature_
 */
int16_t optical_flow::gyro_temperature_() const
{
    return m_gyro_temperature_;
}

/*!
 * @brief This function returns a reference to member gyro_temperature_
 * @return Reference to member gyro_temperature_
 */
int16_t& optical_flow::gyro_temperature_()
{
    return m_gyro_temperature_;
}

/*!
 * @brief This function sets a value in member quality_
 * @param _quality_ New value for member quality_
 */
void optical_flow::quality_(uint8_t _quality_)
{
m_quality_ = _quality_;
}

/*!
 * @brief This function returns the value of member quality_
 * @return Value of member quality_
 */
uint8_t optical_flow::quality_() const
{
    return m_quality_;
}

/*!
 * @brief This function returns a reference to member quality_
 * @return Reference to member quality_
 */
uint8_t& optical_flow::quality_()
{
    return m_quality_;
}

/*!
 * @brief This function sets a value in member max_flow_rate_
 * @param _max_flow_rate_ New value for member max_flow_rate_
 */
void optical_flow::max_flow_rate_(float _max_flow_rate_)
{
m_max_flow_rate_ = _max_flow_rate_;
}

/*!
 * @brief This function returns the value of member max_flow_rate_
 * @return Value of member max_flow_rate_
 */
float optical_flow::max_flow_rate_() const
{
    return m_max_flow_rate_;
}

/*!
 * @brief This function returns a reference to member max_flow_rate_
 * @return Reference to member max_flow_rate_
 */
float& optical_flow::max_flow_rate_()
{
    return m_max_flow_rate_;
}

/*!
 * @brief This function sets a value in member min_ground_distance_
 * @param _min_ground_distance_ New value for member min_ground_distance_
 */
void optical_flow::min_ground_distance_(float _min_ground_distance_)
{
m_min_ground_distance_ = _min_ground_distance_;
}

/*!
 * @brief This function returns the value of member min_ground_distance_
 * @return Value of member min_ground_distance_
 */
float optical_flow::min_ground_distance_() const
{
    return m_min_ground_distance_;
}

/*!
 * @brief This function returns a reference to member min_ground_distance_
 * @return Reference to member min_ground_distance_
 */
float& optical_flow::min_ground_distance_()
{
    return m_min_ground_distance_;
}

/*!
 * @brief This function sets a value in member max_ground_distance_
 * @param _max_ground_distance_ New value for member max_ground_distance_
 */
void optical_flow::max_ground_distance_(float _max_ground_distance_)
{
m_max_ground_distance_ = _max_ground_distance_;
}

/*!
 * @brief This function returns the value of member max_ground_distance_
 * @return Value of member max_ground_distance_
 */
float optical_flow::max_ground_distance_() const
{
    return m_max_ground_distance_;
}

/*!
 * @brief This function returns a reference to member max_ground_distance_
 * @return Reference to member max_ground_distance_
 */
float& optical_flow::max_ground_distance_()
{
    return m_max_ground_distance_;
}

/*!
 * @brief This function sets a value in member mode_
 * @param _mode_ New value for member mode_
 */
void optical_flow::mode_(uint8_t _mode_)
{
m_mode_ = _mode_;
}

/*!
 * @brief This function returns the value of member mode_
 * @return Value of member mode_
 */
uint8_t optical_flow::mode_() const
{
    return m_mode_;
}

/*!
 * @brief This function returns a reference to member mode_
 * @return Reference to member mode_
 */
uint8_t& optical_flow::mode_()
{
    return m_mode_;
}


size_t optical_flow::getKeyMaxCdrSerializedSize(size_t current_alignment)
{
    size_t current_align = current_alignment;




















    return current_align;
}

bool optical_flow::isKeyDefined()
{
   return false;
}

void optical_flow::serializeKey(eprosima::fastcdr::Cdr &scdr) const
{
    (void) scdr;
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
}
