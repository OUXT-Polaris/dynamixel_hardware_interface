/**
 * @file constants.hpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief Header for defineing constant values.
 * @version 0.1
 * @date 2021-05-01
 *
 * @copyright Copyright (c) OUXT Polaris 2021
 *
 */

// Copyright (c) 2019 OUXT Polaris
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

#ifndef DYNAMIXEL_HARDWARE_INTERFACE__CONSTANTS_HPP_
#define DYNAMIXEL_HARDWARE_INTERFACE__CONSTANTS_HPP_

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace dynamixel_hardware_interface
{
constexpr double PROTOCOL_VERSION = 2.0;
constexpr int DXL_HOME_POSITION = 0;  // value range:0 ~ 1023
constexpr double DXL_MAX_POSITION = 1023.0;
constexpr double DXL_MAX_POSITION_DEGREES = 300.0;
constexpr double TO_RADIANS = (DXL_MAX_POSITION_DEGREES / DXL_MAX_POSITION) * M_PI / 180.0;
constexpr double TO_DXL_POS = 1.0 / TO_RADIANS;
constexpr double TO_SPEED_REV_PER_MIN = 0.111;
constexpr double TO_SPEED_RAD_PER_MIN = TO_SPEED_REV_PER_MIN * 2.0 * M_PI;
constexpr double TO_SPEED_RAD_PER_SEC = TO_SPEED_RAD_PER_MIN / 60.0;
constexpr double TO_LOAD_PERCENT = 0.1;
constexpr double TO_VOLTAGE = 0.1;
constexpr double PULSE_RESOLUTION = 4096;

#define GENERATE_ENUM_ITERATOR(T, LAST_VALUE)                                          \
  inline T operator++(T & x) { return x = (T)(std::underlying_type<T>::type(x) + 1); } \
  inline T operator*(T c) { return c; }                                                \
  inline T begin(T) { return static_cast<T>(0); }                                      \
  inline T end(T)                                                                      \
  {                                                                                    \
    T l = T::LAST_VALUE;                                                               \
    return l;                                                                          \
  }

/**
 * @brief Enum class of the commands
 */
enum class Operation {
  TORQUE_ENABLE,
  GOAL_POSITION,
  MOVING_SPEED,
  PRESENT_POSITION,
  PRESENT_SPEED,
  PRESENT_LOAD,
  PRESENT_VOLTAGE,
  PRESENT_TEMPERATURE
};

GENERATE_ENUM_ITERATOR(Operation, PRESENT_TEMPERATURE)

/**
 * @brief Enum class of the supported motor
 */
enum class SupportedMotors {
  /**
   * @brief Robotis xw540-t260 motor
   * @sa https://emanual.robotis.com/docs/en/dxl/x/xw540-t260/
   */
  XW540_T260,
  /**
   * @brief Invalid motor type
   */
  INVALID
};

GENERATE_ENUM_ITERATOR(SupportedMotors, INVALID)

enum class PacketByteSize { ONE_BYTE, TWO_BYTE, FOUR_BYTE, INVALID };

GENERATE_ENUM_ITERATOR(PacketByteSize, INVALID)

enum class DiagnosticsType { TEMPELATURE };

GENERATE_ENUM_ITERATOR(DiagnosticsType, TEMPELATURE)

}  //  namespace dynamixel_hardware_interface

#endif  // DYNAMIXEL_HARDWARE_INTERFACE__CONSTANTS_HPP_
