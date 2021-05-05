/**
 * @file util.hpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief utility functions
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

#ifndef DYNAMIXEL_HARDWARE_INTERFACE__UTIL_HPP_
#define DYNAMIXEL_HARDWARE_INTERFACE__UTIL_HPP_

#include <dynamixel_hardware_interface/constants.hpp>

namespace dynamixel_hardware_interface
{
const std::string toString(const SupportedMotors motor);
}  //  namespace dynamixel_hardware_interface

#endif  // DYNAMIXEL_HARDWARE_INTERFACE__UTIL_HPP_
