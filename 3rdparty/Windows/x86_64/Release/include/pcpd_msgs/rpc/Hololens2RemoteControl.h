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
 * @file Hololens2RemoteControl.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _FAST_DDS_GENERATED_PCPD_MSGS_RPC_HOLOLENS2REMOTECONTROL_H_
#define _FAST_DDS_GENERATED_PCPD_MSGS_RPC_HOLOLENS2REMOTECONTROL_H_

#include "pcpd_msgs/rpc/Types.h"

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
#if defined(Hololens2RemoteControl_SOURCE)
#define Hololens2RemoteControl_DllAPI __declspec( dllexport )
#else
#define Hololens2RemoteControl_DllAPI __declspec( dllimport )
#endif // Hololens2RemoteControl_SOURCE
#else
#define Hololens2RemoteControl_DllAPI
#endif  // EPROSIMA_USER_DLL_EXPORT
#else
#define Hololens2RemoteControl_DllAPI
#endif // _WIN32

namespace eprosima {
namespace fastcdr {
class Cdr;
} // namespace fastcdr
} // namespace eprosima


namespace pcpd_msgs {
    namespace rpc {
        /*!
         * @brief This class represents the structure HL2RCResponse_GetApplicationVersion defined by the user in the IDL file.
         * @ingroup HOLOLENS2REMOTECONTROL
         */
        class HL2RCResponse_GetApplicationVersion
        {
        public:

            /*!
             * @brief Default constructor.
             */
            eProsima_user_DllExport HL2RCResponse_GetApplicationVersion();

            /*!
             * @brief Default destructor.
             */
            eProsima_user_DllExport ~HL2RCResponse_GetApplicationVersion();

            /*!
             * @brief Copy constructor.
             * @param x Reference to the object pcpd_msgs::rpc::HL2RCResponse_GetApplicationVersion that will be copied.
             */
            eProsima_user_DllExport HL2RCResponse_GetApplicationVersion(
                    const HL2RCResponse_GetApplicationVersion& x);

            /*!
             * @brief Move constructor.
             * @param x Reference to the object pcpd_msgs::rpc::HL2RCResponse_GetApplicationVersion that will be copied.
             */
            eProsima_user_DllExport HL2RCResponse_GetApplicationVersion(
                    HL2RCResponse_GetApplicationVersion&& x) noexcept;

            /*!
             * @brief Copy assignment.
             * @param x Reference to the object pcpd_msgs::rpc::HL2RCResponse_GetApplicationVersion that will be copied.
             */
            eProsima_user_DllExport HL2RCResponse_GetApplicationVersion& operator =(
                    const HL2RCResponse_GetApplicationVersion& x);

            /*!
             * @brief Move assignment.
             * @param x Reference to the object pcpd_msgs::rpc::HL2RCResponse_GetApplicationVersion that will be copied.
             */
            eProsima_user_DllExport HL2RCResponse_GetApplicationVersion& operator =(
                    HL2RCResponse_GetApplicationVersion&& x) noexcept;

            /*!
             * @brief Comparison operator.
             * @param x pcpd_msgs::rpc::HL2RCResponse_GetApplicationVersion object to compare.
             */
            eProsima_user_DllExport bool operator ==(
                    const HL2RCResponse_GetApplicationVersion& x) const;

            /*!
             * @brief Comparison operator.
             * @param x pcpd_msgs::rpc::HL2RCResponse_GetApplicationVersion object to compare.
             */
            eProsima_user_DllExport bool operator !=(
                    const HL2RCResponse_GetApplicationVersion& x) const;

            /*!
             * @brief This function copies the value in member data
             * @param _data New value to be copied in member data
             */
            eProsima_user_DllExport void data(
                    const pcpd_msgs::rpc::uint16__4& _data);

            /*!
             * @brief This function moves the value in member data
             * @param _data New value to be moved in member data
             */
            eProsima_user_DllExport void data(
                    pcpd_msgs::rpc::uint16__4&& _data);

            /*!
             * @brief This function returns a constant reference to member data
             * @return Constant reference to member data
             */
            eProsima_user_DllExport const pcpd_msgs::rpc::uint16__4& data() const;

            /*!
             * @brief This function returns a reference to member data
             * @return Reference to member data
             */
            eProsima_user_DllExport pcpd_msgs::rpc::uint16__4& data();
            /*!
             * @brief This function sets a value in member status
             * @param _status New value for member status
             */
            eProsima_user_DllExport void status(
                    pcpd_msgs::rpc::RPCResponseStatus _status);

            /*!
             * @brief This function returns the value of member status
             * @return Value of member status
             */
            eProsima_user_DllExport pcpd_msgs::rpc::RPCResponseStatus status() const;

            /*!
             * @brief This function returns a reference to member status
             * @return Reference to member status
             */
            eProsima_user_DllExport pcpd_msgs::rpc::RPCResponseStatus& status();


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
                    const pcpd_msgs::rpc::HL2RCResponse_GetApplicationVersion& data,
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

            pcpd_msgs::rpc::uint16__4 m_data;
            pcpd_msgs::rpc::RPCResponseStatus m_status;

        };
        /*!
         * @brief This class represents the structure HL2RCRequest_SetPVFocus defined by the user in the IDL file.
         * @ingroup HOLOLENS2REMOTECONTROL
         */
        class HL2RCRequest_SetPVFocus
        {
        public:

            /*!
             * @brief Default constructor.
             */
            eProsima_user_DllExport HL2RCRequest_SetPVFocus();

            /*!
             * @brief Default destructor.
             */
            eProsima_user_DllExport ~HL2RCRequest_SetPVFocus();

            /*!
             * @brief Copy constructor.
             * @param x Reference to the object pcpd_msgs::rpc::HL2RCRequest_SetPVFocus that will be copied.
             */
            eProsima_user_DllExport HL2RCRequest_SetPVFocus(
                    const HL2RCRequest_SetPVFocus& x);

            /*!
             * @brief Move constructor.
             * @param x Reference to the object pcpd_msgs::rpc::HL2RCRequest_SetPVFocus that will be copied.
             */
            eProsima_user_DllExport HL2RCRequest_SetPVFocus(
                    HL2RCRequest_SetPVFocus&& x) noexcept;

            /*!
             * @brief Copy assignment.
             * @param x Reference to the object pcpd_msgs::rpc::HL2RCRequest_SetPVFocus that will be copied.
             */
            eProsima_user_DllExport HL2RCRequest_SetPVFocus& operator =(
                    const HL2RCRequest_SetPVFocus& x);

            /*!
             * @brief Move assignment.
             * @param x Reference to the object pcpd_msgs::rpc::HL2RCRequest_SetPVFocus that will be copied.
             */
            eProsima_user_DllExport HL2RCRequest_SetPVFocus& operator =(
                    HL2RCRequest_SetPVFocus&& x) noexcept;

            /*!
             * @brief Comparison operator.
             * @param x pcpd_msgs::rpc::HL2RCRequest_SetPVFocus object to compare.
             */
            eProsima_user_DllExport bool operator ==(
                    const HL2RCRequest_SetPVFocus& x) const;

            /*!
             * @brief Comparison operator.
             * @param x pcpd_msgs::rpc::HL2RCRequest_SetPVFocus object to compare.
             */
            eProsima_user_DllExport bool operator !=(
                    const HL2RCRequest_SetPVFocus& x) const;

            /*!
             * @brief This function sets a value in member focus_mode
             * @param _focus_mode New value for member focus_mode
             */
            eProsima_user_DllExport void focus_mode(
                    uint32_t _focus_mode);

            /*!
             * @brief This function returns the value of member focus_mode
             * @return Value of member focus_mode
             */
            eProsima_user_DllExport uint32_t focus_mode() const;

            /*!
             * @brief This function returns a reference to member focus_mode
             * @return Reference to member focus_mode
             */
            eProsima_user_DllExport uint32_t& focus_mode();

            /*!
             * @brief This function sets a value in member autofocus_range
             * @param _autofocus_range New value for member autofocus_range
             */
            eProsima_user_DllExport void autofocus_range(
                    uint32_t _autofocus_range);

            /*!
             * @brief This function returns the value of member autofocus_range
             * @return Value of member autofocus_range
             */
            eProsima_user_DllExport uint32_t autofocus_range() const;

            /*!
             * @brief This function returns a reference to member autofocus_range
             * @return Reference to member autofocus_range
             */
            eProsima_user_DllExport uint32_t& autofocus_range();

            /*!
             * @brief This function sets a value in member distance
             * @param _distance New value for member distance
             */
            eProsima_user_DllExport void distance(
                    uint32_t _distance);

            /*!
             * @brief This function returns the value of member distance
             * @return Value of member distance
             */
            eProsima_user_DllExport uint32_t distance() const;

            /*!
             * @brief This function returns a reference to member distance
             * @return Reference to member distance
             */
            eProsima_user_DllExport uint32_t& distance();

            /*!
             * @brief This function sets a value in member value
             * @param _value New value for member value
             */
            eProsima_user_DllExport void value(
                    uint32_t _value);

            /*!
             * @brief This function returns the value of member value
             * @return Value of member value
             */
            eProsima_user_DllExport uint32_t value() const;

            /*!
             * @brief This function returns a reference to member value
             * @return Reference to member value
             */
            eProsima_user_DllExport uint32_t& value();

            /*!
             * @brief This function sets a value in member disable_driver_fallback
             * @param _disable_driver_fallback New value for member disable_driver_fallback
             */
            eProsima_user_DllExport void disable_driver_fallback(
                    uint32_t _disable_driver_fallback);

            /*!
             * @brief This function returns the value of member disable_driver_fallback
             * @return Value of member disable_driver_fallback
             */
            eProsima_user_DllExport uint32_t disable_driver_fallback() const;

            /*!
             * @brief This function returns a reference to member disable_driver_fallback
             * @return Reference to member disable_driver_fallback
             */
            eProsima_user_DllExport uint32_t& disable_driver_fallback();


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
                    const pcpd_msgs::rpc::HL2RCRequest_SetPVFocus& data,
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

            uint32_t m_focus_mode;
            uint32_t m_autofocus_range;
            uint32_t m_distance;
            uint32_t m_value;
            uint32_t m_disable_driver_fallback;

        };
        /*!
         * @brief This class represents the structure HL2RCRequest_SetPVExposure defined by the user in the IDL file.
         * @ingroup HOLOLENS2REMOTECONTROL
         */
        class HL2RCRequest_SetPVExposure
        {
        public:

            /*!
             * @brief Default constructor.
             */
            eProsima_user_DllExport HL2RCRequest_SetPVExposure();

            /*!
             * @brief Default destructor.
             */
            eProsima_user_DllExport ~HL2RCRequest_SetPVExposure();

            /*!
             * @brief Copy constructor.
             * @param x Reference to the object pcpd_msgs::rpc::HL2RCRequest_SetPVExposure that will be copied.
             */
            eProsima_user_DllExport HL2RCRequest_SetPVExposure(
                    const HL2RCRequest_SetPVExposure& x);

            /*!
             * @brief Move constructor.
             * @param x Reference to the object pcpd_msgs::rpc::HL2RCRequest_SetPVExposure that will be copied.
             */
            eProsima_user_DllExport HL2RCRequest_SetPVExposure(
                    HL2RCRequest_SetPVExposure&& x) noexcept;

            /*!
             * @brief Copy assignment.
             * @param x Reference to the object pcpd_msgs::rpc::HL2RCRequest_SetPVExposure that will be copied.
             */
            eProsima_user_DllExport HL2RCRequest_SetPVExposure& operator =(
                    const HL2RCRequest_SetPVExposure& x);

            /*!
             * @brief Move assignment.
             * @param x Reference to the object pcpd_msgs::rpc::HL2RCRequest_SetPVExposure that will be copied.
             */
            eProsima_user_DllExport HL2RCRequest_SetPVExposure& operator =(
                    HL2RCRequest_SetPVExposure&& x) noexcept;

            /*!
             * @brief Comparison operator.
             * @param x pcpd_msgs::rpc::HL2RCRequest_SetPVExposure object to compare.
             */
            eProsima_user_DllExport bool operator ==(
                    const HL2RCRequest_SetPVExposure& x) const;

            /*!
             * @brief Comparison operator.
             * @param x pcpd_msgs::rpc::HL2RCRequest_SetPVExposure object to compare.
             */
            eProsima_user_DllExport bool operator !=(
                    const HL2RCRequest_SetPVExposure& x) const;

            /*!
             * @brief This function sets a value in member mode
             * @param _mode New value for member mode
             */
            eProsima_user_DllExport void mode(
                    uint32_t _mode);

            /*!
             * @brief This function returns the value of member mode
             * @return Value of member mode
             */
            eProsima_user_DllExport uint32_t mode() const;

            /*!
             * @brief This function returns a reference to member mode
             * @return Reference to member mode
             */
            eProsima_user_DllExport uint32_t& mode();

            /*!
             * @brief This function sets a value in member value
             * @param _value New value for member value
             */
            eProsima_user_DllExport void value(
                    uint32_t _value);

            /*!
             * @brief This function returns the value of member value
             * @return Value of member value
             */
            eProsima_user_DllExport uint32_t value() const;

            /*!
             * @brief This function returns a reference to member value
             * @return Reference to member value
             */
            eProsima_user_DllExport uint32_t& value();


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
                    const pcpd_msgs::rpc::HL2RCRequest_SetPVExposure& data,
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

            uint32_t m_mode;
            uint32_t m_value;

        };
        /*!
         * @brief This class represents the structure HL2RCRequest_SetPVIsoSpeed defined by the user in the IDL file.
         * @ingroup HOLOLENS2REMOTECONTROL
         */
        class HL2RCRequest_SetPVIsoSpeed
        {
        public:

            /*!
             * @brief Default constructor.
             */
            eProsima_user_DllExport HL2RCRequest_SetPVIsoSpeed();

            /*!
             * @brief Default destructor.
             */
            eProsima_user_DllExport ~HL2RCRequest_SetPVIsoSpeed();

            /*!
             * @brief Copy constructor.
             * @param x Reference to the object pcpd_msgs::rpc::HL2RCRequest_SetPVIsoSpeed that will be copied.
             */
            eProsima_user_DllExport HL2RCRequest_SetPVIsoSpeed(
                    const HL2RCRequest_SetPVIsoSpeed& x);

            /*!
             * @brief Move constructor.
             * @param x Reference to the object pcpd_msgs::rpc::HL2RCRequest_SetPVIsoSpeed that will be copied.
             */
            eProsima_user_DllExport HL2RCRequest_SetPVIsoSpeed(
                    HL2RCRequest_SetPVIsoSpeed&& x) noexcept;

            /*!
             * @brief Copy assignment.
             * @param x Reference to the object pcpd_msgs::rpc::HL2RCRequest_SetPVIsoSpeed that will be copied.
             */
            eProsima_user_DllExport HL2RCRequest_SetPVIsoSpeed& operator =(
                    const HL2RCRequest_SetPVIsoSpeed& x);

            /*!
             * @brief Move assignment.
             * @param x Reference to the object pcpd_msgs::rpc::HL2RCRequest_SetPVIsoSpeed that will be copied.
             */
            eProsima_user_DllExport HL2RCRequest_SetPVIsoSpeed& operator =(
                    HL2RCRequest_SetPVIsoSpeed&& x) noexcept;

            /*!
             * @brief Comparison operator.
             * @param x pcpd_msgs::rpc::HL2RCRequest_SetPVIsoSpeed object to compare.
             */
            eProsima_user_DllExport bool operator ==(
                    const HL2RCRequest_SetPVIsoSpeed& x) const;

            /*!
             * @brief Comparison operator.
             * @param x pcpd_msgs::rpc::HL2RCRequest_SetPVIsoSpeed object to compare.
             */
            eProsima_user_DllExport bool operator !=(
                    const HL2RCRequest_SetPVIsoSpeed& x) const;

            /*!
             * @brief This function sets a value in member setauto
             * @param _setauto New value for member setauto
             */
            eProsima_user_DllExport void setauto(
                    uint32_t _setauto);

            /*!
             * @brief This function returns the value of member setauto
             * @return Value of member setauto
             */
            eProsima_user_DllExport uint32_t setauto() const;

            /*!
             * @brief This function returns a reference to member setauto
             * @return Reference to member setauto
             */
            eProsima_user_DllExport uint32_t& setauto();

            /*!
             * @brief This function sets a value in member value
             * @param _value New value for member value
             */
            eProsima_user_DllExport void value(
                    uint32_t _value);

            /*!
             * @brief This function returns the value of member value
             * @return Value of member value
             */
            eProsima_user_DllExport uint32_t value() const;

            /*!
             * @brief This function returns a reference to member value
             * @return Reference to member value
             */
            eProsima_user_DllExport uint32_t& value();


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
                    const pcpd_msgs::rpc::HL2RCRequest_SetPVIsoSpeed& data,
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

            uint32_t m_setauto;
            uint32_t m_value;

        };
    } // namespace rpc
} // namespace pcpd_msgs

#endif // _FAST_DDS_GENERATED_PCPD_MSGS_RPC_HOLOLENS2REMOTECONTROL_H_