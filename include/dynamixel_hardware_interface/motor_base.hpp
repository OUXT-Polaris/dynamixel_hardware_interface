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

#ifndef DYNAMIXEL_HARDWARE_INTERFACE__MOTOR_BASE_HPP_
#define DYNAMIXEL_HARDWARE_INTERFACE__MOTOR_BASE_HPP_

#include <dynamixel_hardware_interface/constants.hpp>
#include <dynamixel_hardware_interface/address_table_base.hpp>

#include <dynamixel_sdk/dynamixel_sdk.h>

#include <memory>
#include <string>

namespace dynamixel_hardware_interface
{
struct Result
{
  const std::string description;
  const bool success;
  Result(const std::string & description, bool success)
  : description(description), success(success) {}
};

class MotorBase
{
public:
  const std::string motor_type;
  const int baudrate;
  const uint8_t id;
  MotorBase() = delete;
  template<typename AddressTable>
  MotorBase(
    const std::string & motor_type,
    const AddressTable & table,
    int baudrate,
    uint8_t id)
  : motor_type(motor_type),
    baudrate(baudrate),
    id(id)
  {
    address_table_ = std::make_shared<AddressTableBase>(table);
  }
  ~MotorBase();
  Result torqueEnable();

private:
  std::shared_ptr<AddressTableBase> address_table_;
  std::shared_ptr<dynamixel::PortHandler> port_handler_;
  std::shared_ptr<dynamixel::PacketHandler> packet_handler_;
};
}  //  namespace dynamixel_hardware_interface

#endif  // DYNAMIXEL_HARDWARE_INTERFACE__MOTOR_BASE_HPP_
