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
 * @file Pose.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _FAST_DDS_GENERATED_GEOMETRY_MSGS_MSG_POSE_H_
#define _FAST_DDS_GENERATED_GEOMETRY_MSGS_MSG_POSE_H_

#include "geometry_msgs/msg/Point.h"
#include "geometry_msgs/msg/Quaternion.h"

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
#if defined(Pose_SOURCE)
#define Pose_DllAPI __declspec( dllexport )
#else
#define Pose_DllAPI __declspec( dllimport )
#endif // Pose_SOURCE
#else
#define Pose_DllAPI
#endif  // EPROSIMA_USER_DLL_EXPORT
#else
#define Pose_DllAPI
#endif // _WIN32

namespace eprosima {
namespace fastcdr {
class Cdr;
} // namespace fastcdr
} // namespace eprosima


namespace geometry_msgs {
    namespace msg {
        /*!
         * @brief This class represents the structure Pose defined by the user in the IDL file.
         * @ingroup POSE
         */
        class Pose
        {
        public:

            /*!
             * @brief Default constructor.
             */
            eProsima_user_DllExport Pose();

            /*!
             * @brief Default destructor.
             */
            eProsima_user_DllExport ~Pose();

            /*!
             * @brief Copy constructor.
             * @param x Reference to the object geometry_msgs::msg::Pose that will be copied.
             */
            eProsima_user_DllExport Pose(
                    const Pose& x);

            /*!
             * @brief Move constructor.
             * @param x Reference to the object geometry_msgs::msg::Pose that will be copied.
             */
            eProsima_user_DllExport Pose(
                    Pose&& x) noexcept;

            /*!
             * @brief Copy assignment.
             * @param x Reference to the object geometry_msgs::msg::Pose that will be copied.
             */
            eProsima_user_DllExport Pose& operator =(
                    const Pose& x);

            /*!
             * @brief Move assignment.
             * @param x Reference to the object geometry_msgs::msg::Pose that will be copied.
             */
            eProsima_user_DllExport Pose& operator =(
                    Pose&& x) noexcept;

            /*!
             * @brief Comparison operator.
             * @param x geometry_msgs::msg::Pose object to compare.
             */
            eProsima_user_DllExport bool operator ==(
                    const Pose& x) const;

            /*!
             * @brief Comparison operator.
             * @param x geometry_msgs::msg::Pose object to compare.
             */
            eProsima_user_DllExport bool operator !=(
                    const Pose& x) const;

            /*!
             * @brief This function copies the value in member position
             * @param _position New value to be copied in member position
             */
            eProsima_user_DllExport void position(
                    const geometry_msgs::msg::Point& _position);

            /*!
             * @brief This function moves the value in member position
             * @param _position New value to be moved in member position
             */
            eProsima_user_DllExport void position(
                    geometry_msgs::msg::Point&& _position);

            /*!
             * @brief This function returns a constant reference to member position
             * @return Constant reference to member position
             */
            eProsima_user_DllExport const geometry_msgs::msg::Point& position() const;

            /*!
             * @brief This function returns a reference to member position
             * @return Reference to member position
             */
            eProsima_user_DllExport geometry_msgs::msg::Point& position();
            /*!
             * @brief This function copies the value in member orientation
             * @param _orientation New value to be copied in member orientation
             */
            eProsima_user_DllExport void orientation(
                    const geometry_msgs::msg::Quaternion& _orientation);

            /*!
             * @brief This function moves the value in member orientation
             * @param _orientation New value to be moved in member orientation
             */
            eProsima_user_DllExport void orientation(
                    geometry_msgs::msg::Quaternion&& _orientation);

            /*!
             * @brief This function returns a constant reference to member orientation
             * @return Constant reference to member orientation
             */
            eProsima_user_DllExport const geometry_msgs::msg::Quaternion& orientation() const;

            /*!
             * @brief This function returns a reference to member orientation
             * @return Reference to member orientation
             */
            eProsima_user_DllExport geometry_msgs::msg::Quaternion& orientation();

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
                    const geometry_msgs::msg::Pose& data,
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

            geometry_msgs::msg::Point m_position;
            geometry_msgs::msg::Quaternion m_orientation;

        };
    } // namespace msg
} // namespace geometry_msgs

#endif // _FAST_DDS_GENERATED_GEOMETRY_MSGS_MSG_POSE_H_