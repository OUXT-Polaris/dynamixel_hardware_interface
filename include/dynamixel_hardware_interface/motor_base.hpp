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

#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_status_values.hpp>

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
  const std::string joint_name;
  const int baudrate;
  const uint8_t id;
  MotorBase() = delete;
  template<typename AddressTable>
  MotorBase(
    const std::string & motor_type,
    const std::string & joint_name,
    const AddressTable & table,
    int baudrate,
    uint8_t id,
    std::shared_ptr<dynamixel::PortHandler> port_handler,
    std::shared_ptr<dynamixel::PacketHandler> packet_handler)
  : motor_type(motor_type),
    joint_name(joint_name),
    baudrate(baudrate),
    id(id),
    port_handler_(port_handler),
    packet_handler_(packet_handler)
  {
    address_table_ = std::make_shared<AddressTableBase>(table);
  }
  ~MotorBase();
  Result torqueEnable(bool enable);
  Result setGoalPosition(double goal_position);
  double getJointPosition() const;
  Result updateJointPosition();
  std::vector<hardware_interface::StateInterface> getStateInterfaces();

private:
  Result getResult(int communication_result, uint8_t packet_error);
  uint16_t radianToPosition(double radian) const;
  double positionToRadian(const uint16_t position) const;
  std::shared_ptr<AddressTableBase> address_table_;
  std::shared_ptr<dynamixel::PortHandler> port_handler_;
  std::shared_ptr<dynamixel::PacketHandler> packet_handler_;
  double joint_position_;
};
}  //  namespace dynamixel_hardware_interface

#endif  // DYNAMIXEL_HARDWARE_INTERFACE__MOTOR_BASE_HPP_
