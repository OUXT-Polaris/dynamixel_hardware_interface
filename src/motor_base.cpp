/**
 * @file motor_base.cpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief Implementation of the motor class.
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

#include <dynamixel_hardware_interface/motor_base.hpp>
#include <dynamixel_hardware_interface/util.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <string>
#include <vector>

namespace dynamixel_hardware_interface
{
MotorBase::~MotorBase() {}

bool MotorBase::operationSupports(const Operation & operation)
{
  const auto address = address_table_->getAddress(operation);
  if (!address) {
    return false;
  }
  return true;
}

std::vector<Operation> MotorBase::getSupportedOperations()
{
  std::vector<Operation> ret = {};
  for (const auto operation : Operation()) {
    const auto address = address_table_->getAddress(operation);
    if (address) {
      ret.emplace_back(operation);
    }
  }
  return ret;
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

Result MotorBase::configure()
{
  if (address_table_->addressExists(Operation::PRESENT_POSITION)) {
    joint_position_ = 0;
  }
  if (address_table_->addressExists(Operation::GOAL_POSITION)) {
    goal_position_ = 0;
  }
  return Result("", true);
}

void MotorBase::appendStateInterfaces(std::vector<hardware_interface::StateInterface> & interfaces)
{
  for (const auto operation : Operation()) {
    if (address_table_->addressExists(operation)) {
      switch (operation) {
        case Operation::PRESENT_POSITION:
          interfaces.emplace_back(hardware_interface::StateInterface(
            joint_name, hardware_interface::HW_IF_POSITION, &joint_position_));
          break;
        default:
          break;
      }
    }
  }
}

void MotorBase::appendCommandInterfaces(
  std::vector<hardware_interface::CommandInterface> & interfaces)
{
  for (const auto operation : Operation()) {
    if (address_table_->addressExists(operation)) {
      switch (operation) {
        case Operation::GOAL_POSITION:
          interfaces.emplace_back(hardware_interface::CommandInterface(
            joint_name, hardware_interface::HW_IF_POSITION, &goal_position_));
          break;
        default:
          break;
      }
    }
  }
}

Result MotorBase::torqueEnable(bool enable)
{
  const auto address = address_table_->getAddress(Operation::TORQUE_ENABLE);
  if (!address) {
    return Result("TORQUE_ENABLE operation does not support in " + toString(motor_type), false);
  }
  uint8_t error = 0;
  const auto result =
    packet_handler_->write1ByteTxRx(port_handler_.get(), id, address.get(), enable, &error);
  return getResult(result, error);
}

Result MotorBase::setGoalPosition(double goal_position)
{
  const auto address = address_table_->getAddress(Operation::GOAL_POSITION);
  if (!address) {
    return Result("TORQUE_ENABLE operation does not support in " + toString(motor_type), false);
  }
  uint8_t error = 0;
  const auto result = packet_handler_->write2ByteTxRx(
    port_handler_.get(), id, address.get(), radianToPosition(goal_position), &error);
  return getResult(result, error);
}

Result MotorBase::updateJointPosition()
{
  const auto address = address_table_->getAddress(Operation::PRESENT_POSITION);
  if (!address) {
    return Result("PRESENT_POSITION operation does not support in " + toString(motor_type), false);
  }
  if (enable_dummy) {
    joint_position_ = goal_position_;
    return Result("", true);
  } else {
    uint8_t error = 0;
    uint32_t present_position = 0;

    const auto result = packet_handler_->read4ByteTxRx(
      port_handler_.get(), id, address.get(), &present_position, &error);
    joint_position_ = positionToRadian(present_position);
    return getResult(result, error);
  }
}
}  //  namespace dynamixel_hardware_interface
