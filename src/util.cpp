/**
 * @file util.cpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief implementation of the utility function.
 * @version 0.1
 * @date 2021-05-01
 *
 * @copyright Copyright (c) OUXT Polaris 2021
 *
 */

// Copyright (c) 2021 OUXT Polaris
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

#include <dynamixel_hardware_interface/util.hpp>

namespace dynamixel_hardware_interface
{
const std::string toString(const SupportedMotors motor)
{
  switch (motor) {
    case SupportedMotors::XM430_W350:
      return "XM430-W350";
      break;
    case SupportedMotors::XW540_T260:
      return "XW540-T260";
      break;
    case SupportedMotors::INVALID:
      throw std::runtime_error("invalid moter type");
      break;
  }
  return "";
}
}  //  namespace dynamixel_hardware_interface
