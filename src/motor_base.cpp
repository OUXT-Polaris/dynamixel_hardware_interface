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

#include <dynamixel_hardware_interface/motor_base.hpp>

#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace dynamixel_hardware_interface
{
MotorBase::~MotorBase()
{
}

uint16_t MotorBase::radianToPosition(double radian) const
{
  return radian * TO_DXL_POS + DXL_HOME_POSITION;
}

double MotorBase::positionToRadian(const uint16_t position) const
{
  return (position - DXL_HOME_POSITION) * TO_RADIANS;
}

Result MotorBase::getResult(int communication_result, uint8_t packet_error)
{
  if (communication_result != COMM_SUCCESS) {
    return Result(std::string(packet_handler_->getTxRxResult(communication_result)), false);
  }
  if (packet_error != 0) {
    return Result(std::string(packet_handler_->getRxPacketError(packet_error)), true);
  }
  return Result("", true);
}

std::vector<hardware_interface::StateInterface> MotorBase::getStateInterfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces = {};
  for (const auto operation : Operation()) {
    if (!address_table_->addressExists(operation)) {
      switch (operation) {
        case Operation::PRESENT_POSITION:
          interfaces.emplace_back(
            hardware_interface::StateInterface(
              joint_name, hardware_interface::HW_IF_POSITION, &joint_position_));
          break;
        default:
          break;
      }
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> MotorBase::getCommandInterfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces = {};
  for (const auto operation : Operation()) {
    if (!address_table_->addressExists(operation)) {
      switch (operation) {
        case Operation::GOAL_POSITION:
          interfaces.emplace_back(
            hardware_interface::CommandInterface(
              joint_name,
              hardware_interface::HW_IF_POSITION,
              &goal_position_));
          break;
        default:
          break;
      }
    }
  }
  return interfaces;
}

Result MotorBase::torqueEnable(bool enable)
{
  if (!address_table_->addressExists(Operation::TORQUE_ENABLE)) {
    return Result("TORQUE_ENABLE operation does not support in " + motor_type, false);
  }
  uint8_t error = 0;
  const auto address = address_table_->getAddress(Operation::TORQUE_ENABLE);
  const auto result = packet_handler_->write1ByteTxRx(
    port_handler_.get(), id, address, enable, &error);
  return getResult(result, error);
}

Result MotorBase::setGoalPosition(double goal_position)
{
  if (!address_table_->addressExists(Operation::GOAL_POSITION)) {
    return Result("TORQUE_ENABLE operation does not support in " + motor_type, false);
  }
  uint8_t error = 0;
  const auto address = address_table_->getAddress(Operation::GOAL_POSITION);
  const auto result = packet_handler_->write2ByteTxRx(
    port_handler_.get(), id, address, radianToPosition(goal_position), &error);
  return getResult(result, error);
}

Result MotorBase::updateJointPosition()
{
  if (!address_table_->addressExists(Operation::PRESENT_POSITION)) {
    return Result("PRESENT_POSITION operation does not support in " + motor_type, false);
  }
  uint8_t error = 0;
  uint16_t present_position = 0;
  const auto address = address_table_->getAddress(Operation::PRESENT_POSITION);
  const auto result = packet_handler_->read2ByteTxRx(
    port_handler_.get(),
    id, address, &present_position, &error);
  joint_position_ = positionToRadian(present_position);
  return getResult(result, error);
}
}  //  namespace dynamixel_hardware_interface
