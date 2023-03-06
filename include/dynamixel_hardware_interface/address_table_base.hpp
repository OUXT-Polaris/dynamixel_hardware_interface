/**
 * @file address_table_base.hpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief Base class for the address tabele
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

#ifndef DYNAMIXEL_HARDWARE_INTERFACE__ADDRESS_TABLE_BASE_HPP_
#define DYNAMIXEL_HARDWARE_INTERFACE__ADDRESS_TABLE_BASE_HPP_

#include <boost/optional.hpp>
#include <cmath>
#include <dynamixel_hardware_interface/constants.hpp>
#include <limits>

namespace dynamixel_hardware_interface
{
class Address
{
public:
  Address(uint16_t address, PacketByteSize byte_size) : address(address), byte_size(byte_size) {}
  Address() : address(0), byte_size(PacketByteSize::INVALID) {}
  const uint16_t address;
  const PacketByteSize byte_size;
  bool exists() const
  {
    if (byte_size == PacketByteSize::INVALID) {
      return false;
    }
    return true;
  }
};

/**
   * @brief base class for address table class
   */
class AddressTableBase
{
public:
  /**
     * @brief Construct a new Address Table Base object, each parameter describes the address of the operation.
     * @param ADDR_TORQUE_ENABLE If this value is boost::none, writing torque_enable command address exists.
     * @param ADDR_GOAL_POSITION If this value is boost::none, writing goal_position command address exists.
     * @param ADDR_MOVING_SPEED If this value is boost::none, writing moving_speed command address exists.
     * @param ADDR_PRESENT_POSITION If this value is boost::none, reading present_position command address exists.
     * @param ADDR_PRESENT_SPEED If this value is boost::none, reading present_speed command address exists.
     * @param ADDR_PRESENT_LOAD If this value is boost::none, reading present_load command address exists.
     * @param ADDR_PRESENT_VOLTAGE If this value is boost::none, reading present_voltage command address exists.
     * @param ADDR_PRESENT_TEMPERATURE If this value is boost::none, reading present_temperature command address exists.
     * @param ADDR_MAX_POSITION_LIMIT
     * @param ADDR_MIN_POSITION_LIMIT
     */
  explicit AddressTableBase(
    Address ADDR_TORQUE_ENABLE, Address ADDR_GOAL_POSITION, Address ADDR_MOVING_SPEED,
    Address ADDR_PRESENT_POSITION, Address ADDR_PRESENT_SPEED, Address ADDR_PRESENT_LOAD,
    Address ADDR_PRESENT_VOLTAGE, Address ADDR_PRESENT_TEMPERATURE, Address ADDR_MAX_POSITION_LIMIT,
    Address ADDR_MIN_POSITION_LIMIT)
  : ADDR_TORQUE_ENABLE(ADDR_TORQUE_ENABLE),
    ADDR_GOAL_POSITION(ADDR_GOAL_POSITION),
    ADDR_MOVING_SPEED(ADDR_MOVING_SPEED),
    ADDR_PRESENT_POSITION(ADDR_PRESENT_POSITION),
    ADDR_PRESENT_SPEED(ADDR_PRESENT_SPEED),
    ADDR_PRESENT_LOAD(ADDR_PRESENT_LOAD),
    ADDR_PRESENT_VOLTAGE(ADDR_PRESENT_VOLTAGE),
    ADDR_PRESENT_TEMPERATURE(ADDR_PRESENT_TEMPERATURE),
    ADDR_MAX_POSITION_LIMIT(ADDR_MAX_POSITION_LIMIT),
    ADDR_MIN_POSITION_LIMIT(ADDR_MIN_POSITION_LIMIT)
  {
  }
  /**
     * @brief Get address of which operation you want to execute.
     * @param operaiton operation you want to execute
     * @retval boost::none operation is not supported
     * @retval uint16_t address of the operation you want to execute
     */
  Address getAddress(const Operation & operaiton) const
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
      case Operation::MAX_POSITION_LIMIT:
        return ADDR_MAX_POSITION_LIMIT;
      case Operation::MIN_POSITION_LIMIT:
        return ADDR_MIN_POSITION_LIMIT;
      default:
        return Address();
    }
  }
  /**
     * @brief Check the address exists or not.
     * @param operation operation you want to execute
     * @return true address exist
     * @return false address does not exist
     */
  bool addressExists(const Operation & operation) const { return getAddress(operation).exists(); }

private:
  AddressTableBase() = delete;
  const Address ADDR_TORQUE_ENABLE;
  const Address ADDR_GOAL_POSITION;
  const Address ADDR_MOVING_SPEED;
  const Address ADDR_PRESENT_POSITION;
  const Address ADDR_PRESENT_SPEED;
  const Address ADDR_PRESENT_LOAD;
  const Address ADDR_PRESENT_VOLTAGE;
  const Address ADDR_PRESENT_TEMPERATURE;
  const Address ADDR_MAX_POSITION_LIMIT;
  const Address ADDR_MIN_POSITION_LIMIT;
};
}  // namespace dynamixel_hardware_interface

#endif  // DYNAMIXEL_HARDWARE_INTERFACE__ADDRESS_TABLE_BASE_HPP_
