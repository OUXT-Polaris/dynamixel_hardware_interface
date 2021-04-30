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

#ifndef DYNAMIXEL_HARDWARE_INTERFACE__ADDRESS_TABLE_HPP_
#define DYNAMIXEL_HARDWARE_INTERFACE__ADDRESS_TABLE_HPP_

#include "constants.hpp"
#include <limits>


namespace dynamixel_hardware_interface
{
class AddressTable
{
  const uint16_t ADDR_TORQUE_ENABLE = std::numeric_limits<uint16_t>::quiet_NaN();
  const uint16_t ADDR_GOAL_POSITION = std::numeric_limits<uint16_t>::quiet_NaN();
  const uint16_t ADDR_MOVING_SPEED = std::numeric_limits<uint16_t>::quiet_NaN();
  const uint16_t ADDR_PRESENT_POSITION = std::numeric_limits<uint16_t>::quiet_NaN();
  const uint16_t ADDR_PRESENT_SPEED = std::numeric_limits<uint16_t>::quiet_NaN();
  const uint16_t ADDR_PRESENT_LOAD = std::numeric_limits<uint16_t>::quiet_NaN();
  const uint16_t ADDR_PRESENT_VOLTAGE = std::numeric_limits<uint16_t>::quiet_NaN();
  const uint16_t ADDR_PRESENT_TEMPERATURE = std::numeric_limits<uint16_t>::quiet_NaN();
};
}


#endif  // DYNAMIXEL_HARDWARE_INTERFACE__ADDRESS_TABLE_HPP_
