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
 * @file CameraSensor.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _FAST_DDS_GENERATED_PCPD_MSGS_MSG_CAMERASENSOR_H_
#define _FAST_DDS_GENERATED_PCPD_MSGS_MSG_CAMERASENSOR_H_


#include <fastrtps/utils/fixed_size_string.hpp>

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
#endif  // EPROSIMA_USER_DLL_EXPORT
#else
#define eProsima_user_DllExport
#endif  // _WIN32

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#if defined(CameraSensor_SOURCE)
#define CameraSensor_DllAPI __declspec( dllexport )
#else
#define CameraSensor_DllAPI __declspec( dllimport )
#endif // CameraSensor_SOURCE
#else
#define CameraSensor_DllAPI
#endif  // EPROSIMA_USER_DLL_EXPORT
#else
#define CameraSensor_DllAPI
#endif // _WIN32

namespace eprosima {
namespace fastcdr {
class Cdr;
} // namespace fastcdr
} // namespace eprosima


namespace pcpd_msgs {
    namespace msg {
        /*!
         * @brief This class represents the enumeration CameraSensorTypesEnum defined by the user in the IDL file.
         * @ingroup CAMERASENSOR
         */
        enum CameraSensorTypesEnum : uint32_t
        {
            GENERIC_RGB,
            GENERIC_RGBD,
            KINECT_AZURE
        };
        /*!
         * @brief This class represents the enumeration CameraDepthModesEnum defined by the user in the IDL file.
         * @ingroup CAMERASENSOR
         */
        enum CameraDepthModesEnum : uint32_t
        {
            CAMERA_K4A_DEPTH_MODE_OFF,
            CAMERA_K4A_DEPTH_MODE_NFOV_2X2BINNED,
            CAMERA_K4A_DEPTH_MODE_NFOV_UNBINNED,
            CAMERA_K4A_DEPTH_MODE_WFOV_2X2BINNED,
            CAMERA_K4A_DEPTH_MODE_WFOV_UNBINNED,
            CAMERA_K4A_DEPTH_MODE_PASSIVE_IR
        };
        /*!
         * @brief This class represents the enumeration CameraColorResolutionsEnum defined by the user in the IDL file.
         * @ingroup CAMERASENSOR
         */
        enum CameraColorResolutionsEnum : uint32_t
        {
            CAMERA_K4A_COLOR_RESOLUTION_OFF,
            CAMERA_K4A_COLOR_RESOLUTION_720P,
            CAMERA_K4A_COLOR_RESOLUTION_1080P,
            CAMERA_K4A_COLOR_RESOLUTION_1440P,
            CAMERA_K4A_COLOR_RESOLUTION_1536P,
            CAMERA_K4A_COLOR_RESOLUTION_2160P,
            CAMERA_K4A_COLOR_RESOLUTION_3072P
        };
        /*!
         * @brief This class represents the structure CameraSensor defined by the user in the IDL file.
         * @ingroup CAMERASENSOR
         */
        class CameraSensor
        {
        public:

            /*!
             * @brief Default constructor.
             */
            eProsima_user_DllExport CameraSensor();

            /*!
             * @brief Default destructor.
             */
            eProsima_user_DllExport ~CameraSensor();

            /*!
             * @brief Copy constructor.
             * @param x Reference to the object pcpd_msgs::msg::CameraSensor that will be copied.
             */
            eProsima_user_DllExport CameraSensor(
                    const CameraSensor& x);

            /*!
             * @brief Move constructor.
             * @param x Reference to the object pcpd_msgs::msg::CameraSensor that will be copied.
             */
            eProsima_user_DllExport CameraSensor(
                    CameraSensor&& x) noexcept;

            /*!
             * @brief Copy assignment.
             * @param x Reference to the object pcpd_msgs::msg::CameraSensor that will be copied.
             */
            eProsima_user_DllExport CameraSensor& operator =(
                    const CameraSensor& x);

            /*!
             * @brief Move assignment.
             * @param x Reference to the object pcpd_msgs::msg::CameraSensor that will be copied.
             */
            eProsima_user_DllExport CameraSensor& operator =(
                    CameraSensor&& x) noexcept;

            /*!
             * @brief Comparison operator.
             * @param x pcpd_msgs::msg::CameraSensor object to compare.
             */
            eProsima_user_DllExport bool operator ==(
                    const CameraSensor& x) const;

            /*!
             * @brief Comparison operator.
             * @param x pcpd_msgs::msg::CameraSensor object to compare.
             */
            eProsima_user_DllExport bool operator !=(
                    const CameraSensor& x) const;

            /*!
             * @brief This function copies the value in member name
             * @param _name New value to be copied in member name
             */
            eProsima_user_DllExport void name(
                    const std::string& _name);

            /*!
             * @brief This function moves the value in member name
             * @param _name New value to be moved in member name
             */
            eProsima_user_DllExport void name(
                    std::string&& _name);

            /*!
             * @brief This function returns a constant reference to member name
             * @return Constant reference to member name
             */
            eProsima_user_DllExport const std::string& name() const;

            /*!
             * @brief This function returns a reference to member name
             * @return Reference to member name
             */
            eProsima_user_DllExport std::string& name();
            /*!
             * @brief This function copies the value in member serial_number
             * @param _serial_number New value to be copied in member serial_number
             */
            eProsima_user_DllExport void serial_number(
                    const std::string& _serial_number);

            /*!
             * @brief This function moves the value in member serial_number
             * @param _serial_number New value to be moved in member serial_number
             */
            eProsima_user_DllExport void serial_number(
                    std::string&& _serial_number);

            /*!
             * @brief This function returns a constant reference to member serial_number
             * @return Constant reference to member serial_number
             */
            eProsima_user_DllExport const std::string& serial_number() const;

            /*!
             * @brief This function returns a reference to member serial_number
             * @return Reference to member serial_number
             */
            eProsima_user_DllExport std::string& serial_number();
            /*!
             * @brief This function sets a value in member camera_type
             * @param _camera_type New value for member camera_type
             */
            eProsima_user_DllExport void camera_type(
                    pcpd_msgs::msg::CameraSensorTypesEnum _camera_type);

            /*!
             * @brief This function returns the value of member camera_type
             * @return Value of member camera_type
             */
            eProsima_user_DllExport pcpd_msgs::msg::CameraSensorTypesEnum camera_type() const;

            /*!
             * @brief This function returns a reference to member camera_type
             * @return Reference to member camera_type
             */
            eProsima_user_DllExport pcpd_msgs::msg::CameraSensorTypesEnum& camera_type();

            /*!
             * @brief This function sets a value in member depth_mode
             * @param _depth_mode New value for member depth_mode
             */
            eProsima_user_DllExport void depth_mode(
                    pcpd_msgs::msg::CameraDepthModesEnum _depth_mode);

            /*!
             * @brief This function returns the value of member depth_mode
             * @return Value of member depth_mode
             */
            eProsima_user_DllExport pcpd_msgs::msg::CameraDepthModesEnum depth_mode() const;

            /*!
             * @brief This function returns a reference to member depth_mode
             * @return Reference to member depth_mode
             */
            eProsima_user_DllExport pcpd_msgs::msg::CameraDepthModesEnum& depth_mode();

            /*!
             * @brief This function sets a value in member color_resolution
             * @param _color_resolution New value for member color_resolution
             */
            eProsima_user_DllExport void color_resolution(
                    pcpd_msgs::msg::CameraColorResolutionsEnum _color_resolution);

            /*!
             * @brief This function returns the value of member color_resolution
             * @return Value of member color_resolution
             */
            eProsima_user_DllExport pcpd_msgs::msg::CameraColorResolutionsEnum color_resolution() const;

            /*!
             * @brief This function returns a reference to member color_resolution
             * @return Reference to member color_resolution
             */
            eProsima_user_DllExport pcpd_msgs::msg::CameraColorResolutionsEnum& color_resolution();

            /*!
             * @brief This function copies the value in member raw_calibration
             * @param _raw_calibration New value to be copied in member raw_calibration
             */
            eProsima_user_DllExport void raw_calibration(
                    const std::vector<uint8_t>& _raw_calibration);

            /*!
             * @brief This function moves the value in member raw_calibration
             * @param _raw_calibration New value to be moved in member raw_calibration
             */
            eProsima_user_DllExport void raw_calibration(
                    std::vector<uint8_t>&& _raw_calibration);

            /*!
             * @brief This function returns a constant reference to member raw_calibration
             * @return Constant reference to member raw_calibration
             */
            eProsima_user_DllExport const std::vector<uint8_t>& raw_calibration() const;

            /*!
             * @brief This function returns a reference to member raw_calibration
             * @return Reference to member raw_calibration
             */
            eProsima_user_DllExport std::vector<uint8_t>& raw_calibration();
            /*!
             * @brief This function sets a value in member depth_units_per_meter
             * @param _depth_units_per_meter New value for member depth_units_per_meter
             */
            eProsima_user_DllExport void depth_units_per_meter(
                    float _depth_units_per_meter);

            /*!
             * @brief This function returns the value of member depth_units_per_meter
             * @return Value of member depth_units_per_meter
             */
            eProsima_user_DllExport float depth_units_per_meter() const;

            /*!
             * @brief This function returns a reference to member depth_units_per_meter
             * @return Reference to member depth_units_per_meter
             */
            eProsima_user_DllExport float& depth_units_per_meter();

            /*!
             * @brief This function sets a value in member timestamp_offset_ns
             * @param _timestamp_offset_ns New value for member timestamp_offset_ns
             */
            eProsima_user_DllExport void timestamp_offset_ns(
                    uint64_t _timestamp_offset_ns);

            /*!
             * @brief This function returns the value of member timestamp_offset_ns
             * @return Value of member timestamp_offset_ns
             */
            eProsima_user_DllExport uint64_t timestamp_offset_ns() const;

            /*!
             * @brief This function returns a reference to member timestamp_offset_ns
             * @return Reference to member timestamp_offset_ns
             */
            eProsima_user_DllExport uint64_t& timestamp_offset_ns();


            /*!
            * @brief This function returns the maximum serialized size of an object
            * depending on the buffer alignment.
            * @param current_alignment Buffer alignment.
            * @return Maximum serialized size.
            */
            eProsima_user_DllExport static size_t getMaxCdrSerializedSize(
                    size_t current_alignment = 0);

            /*!
             * @brief This function returns the serialized size of a data depending on the buffer alignment.
             * @param data Data which is calculated its serialized size.
             * @param current_alignment Buffer alignment.
             * @return Serialized size.
             */
            eProsima_user_DllExport static size_t getCdrSerializedSize(
                    const pcpd_msgs::msg::CameraSensor& data,
                    size_t current_alignment = 0);


            /*!
             * @brief This function serializes an object using CDR serialization.
             * @param cdr CDR serialization object.
             */
            eProsima_user_DllExport void serialize(
                    eprosima::fastcdr::Cdr& cdr) const;

            /*!
             * @brief This function deserializes an object using CDR serialization.
             * @param cdr CDR serialization object.
             */
            eProsima_user_DllExport void deserialize(
                    eprosima::fastcdr::Cdr& cdr);



            /*!
             * @brief This function returns the maximum serialized size of the Key of an object
             * depending on the buffer alignment.
             * @param current_alignment Buffer alignment.
             * @return Maximum serialized size.
             */
            eProsima_user_DllExport static size_t getKeyMaxCdrSerializedSize(
                    size_t current_alignment = 0);

            /*!
             * @brief This function tells you if the Key has been defined for this type
             */
            eProsima_user_DllExport static bool isKeyDefined();

            /*!
             * @brief This function serializes the key members of an object using CDR serialization.
             * @param cdr CDR serialization object.
             */
            eProsima_user_DllExport void serializeKey(
                    eprosima::fastcdr::Cdr& cdr) const;

        private:

            std::string m_name;
            std::string m_serial_number;
            pcpd_msgs::msg::CameraSensorTypesEnum m_camera_type;
            pcpd_msgs::msg::CameraDepthModesEnum m_depth_mode;
            pcpd_msgs::msg::CameraColorResolutionsEnum m_color_resolution;
            std::vector<uint8_t> m_raw_calibration;
            float m_depth_units_per_meter;
            uint64_t m_timestamp_offset_ns;

        };
    } // namespace msg
} // namespace pcpd_msgs

#endif // _FAST_DDS_GENERATED_PCPD_MSGS_MSG_CAMERASENSOR_H_