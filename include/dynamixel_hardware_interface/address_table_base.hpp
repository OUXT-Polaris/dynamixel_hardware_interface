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

#ifndef DYNAMIXEL_HARDWARE_INTERFACE__ADDRESS_TABLE_BASE_HPP_
#define DYNAMIXEL_HARDWARE_INTERFACE__ADDRESS_TABLE_BASE_HPP_

#include <boost/optional.hpp>
#include <cmath>
#include <dynamixel_hardware_interface/constants.hpp>
#include <limits>

namespace dynamixel_hardware_interface
{
class AddressTableBase
{
public:
  AddressTableBase() = delete;
  explicit AddressTableBase(
    boost::optional<uint16_t> ADDR_TORQUE_ENABLE, boost::optional<uint16_t> ADDR_GOAL_POSITION,
    boost::optional<uint16_t> ADDR_MOVING_SPEED, boost::optional<uint16_t> ADDR_PRESENT_POSITION,
    boost::optional<uint16_t> ADDR_PRESENT_SPEED, boost::optional<uint16_t> ADDR_PRESENT_LOAD,
    boost::optional<uint16_t> ADDR_PRESENT_VOLTAGE,
    boost::optional<uint16_t> ADDR_PRESENT_TEMPERATURE)
  : ADDR_TORQUE_ENABLE(ADDR_TORQUE_ENABLE),
    ADDR_GOAL_POSITION(ADDR_GOAL_POSITION),
    ADDR_MOVING_SPEED(ADDR_MOVING_SPEED),
    ADDR_PRESENT_POSITION(ADDR_PRESENT_POSITION),
    ADDR_PRESENT_SPEED(ADDR_PRESENT_SPEED),
    ADDR_PRESENT_LOAD(ADDR_PRESENT_LOAD),
    ADDR_PRESENT_VOLTAGE(ADDR_PRESENT_VOLTAGE),
    ADDR_PRESENT_TEMPERATURE(ADDR_PRESENT_TEMPERATURE)
  {
  }
  boost::optional<uint16_t> getAddress(const Operation & operaiton) const
  {
    switch (operaiton) {
      case Operation::TORQUE_ENABLE:
        return ADDR_TORQUE_ENABLE;
      case Operation::GOAL_POSITION:
        return ADDR_GOAL_POSITION;
      case Operation::MOVING_SPEED:
        return ADDR_MOVING_SPEED;
      case Operation::PRESENT_POSITION:
        return ADDR_PRESENT_POSITION;
      case Operation::PRESENT_SPEED:
        return ADDR_PRESENT_SPEED;
      case Operation::PRESENT_LOAD:
        return ADDR_PRESENT_LOAD;
      case Operation::PRESENT_VOLTAGE:
        return ADDR_PRESENT_VOLTAGE;
      case Operation::PRESENT_TEMPERATURE:
        return ADDR_PRESENT_TEMPERATURE;
      default:
        return boost::none;
    }
  }
  bool addressExists(const Operation & operation) const
  {
    if (getAddress(operation)) {
      return true;
    }
    return false;
  }

private:
  const boost::optional<uint16_t> ADDR_TORQUE_ENABLE = boost::none;
  const boost::optional<uint16_t> ADDR_GOAL_POSITION = boost::none;
  const boost::optional<uint16_t> ADDR_MOVING_SPEED = boost::none;
  const boost::optional<uint16_t> ADDR_PRESENT_POSITION = boost::none;
  const boost::optional<uint16_t> ADDR_PRESENT_SPEED = boost::none;
  const boost::optional<uint16_t> ADDR_PRESENT_LOAD = boost::none;
  const boost::optional<uint16_t> ADDR_PRESENT_VOLTAGE = boost::none;
  const boost::optional<uint16_t> ADDR_PRESENT_TEMPERATURE = boost::none;
};
}  // namespace dynamixel_hardware_interface

#endif  // DYNAMIXEL_HARDWARE_INTERFACE__ADDRESS_TABLE_BASE_HPP_
