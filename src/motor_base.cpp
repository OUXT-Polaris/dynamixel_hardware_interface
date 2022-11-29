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

#include <dynamixel_hardware_interface/motor_base.hpp>
#include <dynamixel_hardware_interface/util.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <string>
#include <vector>

namespace dynamixel_hardware_interface
{
MotorBase::~MotorBase()
{
  if (!enable_dummy) {
    torqueEnable(false);
  }
}

bool MotorBase::operationSupports(const Operation & operation)
{
  const auto address = address_table_->getAddress(operation);
  if (!address.exists()) {
    return false;
  }
  return true;
}

std::vector<Operation> MotorBase::getSupportedOperations()
{
  std::vector<Operation> ret = {};
  for (const auto operation : Operation()) {
    const auto address = address_table_->getAddress(operation);
    if (address.exists()) {
      ret.emplace_back(operation);
    }
  }
  return ret;
}

double MotorBase::valueToRpm(uint8_t) const
{
  throw std::runtime_error("value to rpm function should be implemented for each motor");
}

double MotorBase::valueToRpm(uint16_t) const
{
  throw std::runtime_error("value to rpm function should be implemented for each motor");
}

double MotorBase::valueToRpm(uint32_t) const
{
  throw std::runtime_error("value to rpm function should be implemented for each motor");
}

double MotorBase::positionToRadian(const uint8_t) const
{
  throw std::runtime_error("position to radian function should be implemented for each motor");
}

double MotorBase::positionToRadian(const uint16_t) const
{
  throw std::runtime_error("position to radian function should be implemented for each motor");
}

double MotorBase::positionToRadian(const uint32_t) const
{
  throw std::runtime_error("position to radian function should be implemented for each motor");
}

void MotorBase::radianToPosition(double, uint8_t &) const
{
  throw std::runtime_error("radian to position function should be implemented for each motor");
}

void MotorBase::radianToPosition(double, uint16_t &) const
{
  throw std::runtime_error("radian to position function should be implemented for each motor");
}
void MotorBase::radianToPosition(double, uint32_t &) const
{
  throw std::runtime_error("radian to position function should be implemented for each motor");
}

double MotorBase::valueToTemperature(uint8_t) const
{
  throw std::runtime_error("value to temperature function should be implemented for each motor");
}

double MotorBase::valueToTemperature(uint16_t) const
{
  throw std::runtime_error("value to temperature function should be implemented for each motor");
}

double MotorBase::valueToTemperature(uint32_t) const
{
  throw std::runtime_error("value to temperature function should be implemented for each motor");
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

  if (!enable_dummy) {
    Result torque_result = torqueEnable(true);
    if (!torque_result.success) {
      return torque_result;
    }
  }
  return setJointPositionLimit(max_joint_limit, min_joint_limit);
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
        case Operation::PRESENT_SPEED:
          interfaces.emplace_back(hardware_interface::StateInterface(
            joint_name, hardware_interface::HW_IF_VELOCITY, &joint_position_));
          break;
        case Operation::PRESENT_TEMPERATURE:
          interfaces.emplace_back(
            hardware_interface::StateInterface(joint_name, "temperature", &present_temperature_));
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
        case Operation::MOVING_SPEED:
          interfaces.emplace_back(hardware_interface::CommandInterface(
            joint_name, hardware_interface::HW_IF_VELOCITY, &goal_velocity_));
        default:
          break;
      }
    }
  }
}

Result MotorBase::setJointPositionLimit(double max_joint_limit, double min_joint_limit)
{
  const auto address_max_joint_limit = address_table_->getAddress(Operation::MAX_POSITION_LIMIT);
  const auto address_min_joint_limit = address_table_->getAddress(Operation::MIN_POSITION_LIMIT);

  if (!address_max_joint_limit.exists()) {
    return Result(
      "MAX_POSITION_LIMIT operation does not support in " + toString(motor_type), false);
  }
  if (!address_min_joint_limit.exists()) {
    return Result(
      "MIN_POSITION_LIMIT operation does not support in " + toString(motor_type), false);
  }
  uint8_t error = 0;
  if (address_max_joint_limit.byte_size == PacketByteSize::FOUR_BYTE) {
    const auto result_max = packet_handler_->write4ByteTxRx(
      port_handler_.get(), id, address_max_joint_limit.address,
      radianToPosition<uint32_t>(max_joint_limit), &error);
    Result result = getResult(result_max, error);
    if (!result.success) {
      return result;
    }
    const auto result_min = packet_handler_->write4ByteTxRx(
      port_handler_.get(), id, address_min_joint_limit.address,
      radianToPosition<uint32_t>(min_joint_limit), &error);
    return getResult(result_min, error);
  }
  return Result("Invalid packet size", false);
}

Result MotorBase::torqueEnable(bool enable)
{
  const auto address = address_table_->getAddress(Operation::TORQUE_ENABLE);
  if (!address.exists()) {
    return Result("TORQUE_ENABLE operation does not support in " + toString(motor_type), false);
  }
  uint8_t error = 0;
  const auto result =
    packet_handler_->write1ByteTxRx(port_handler_.get(), id, address.address, enable, &error);
  return getResult(result, error);
}

Result MotorBase::setGoalPosition(double goal_position)
{
  goal_position_ = goal_position;
  const auto address = address_table_->getAddress(Operation::GOAL_POSITION);
  if (!address.exists()) {
    return Result("TORQUE_ENABLE operation does not support in " + toString(motor_type), false);
  }
  if (enable_dummy) {
    joint_position_ = goal_position_;
    return Result("", true);
  } else {
    uint8_t error = 0;
    if (address.byte_size == PacketByteSize::ONE_BYTE) {
      const auto result = packet_handler_->write1ByteTxRx(
        port_handler_.get(), id, address.address, radianToPosition<uint8_t>(goal_position_),
        &error);
      return getResult(result, error);
    }
    if (address.byte_size == PacketByteSize::TWO_BYTE) {
      const auto result = packet_handler_->write2ByteTxRx(
        port_handler_.get(), id, address.address, radianToPosition<uint16_t>(goal_position_),
        &error);
      return getResult(result, error);
    }
    if (address.byte_size == PacketByteSize::FOUR_BYTE) {
      const auto result = packet_handler_->write4ByteTxRx(
        port_handler_.get(), id, address.address, radianToPosition<uint32_t>(goal_position_),
        &error);
      return getResult(result, error);
    }
    return Result("Invalid packet size", false);
  }
}

Result MotorBase::updateJointVelocity()
{
  const auto address = address_table_->getAddress(Operation::PRESENT_SPEED);
  if (!address.exists()) {
    return Result("PRESENT_SPEED operation does not support in " + toString(motor_type), false);
  }
  if (enable_dummy) {
    joint_velocity_ = goal_velocity_;
    return Result("", true);
  } else {
    uint8_t error = 0;
    if (address.byte_size == PacketByteSize::ONE_BYTE) {
      uint8_t present_speed = 0;
      const auto result = packet_handler_->read1ByteTxRx(
        port_handler_.get(), id, address.address, &present_speed, &error);
      joint_velocity_ = valueToRpm(present_speed);
      return getResult(result, error);
    }
    if (address.byte_size == PacketByteSize::TWO_BYTE) {
      uint16_t present_speed = 0;
      const auto result = packet_handler_->read2ByteTxRx(
        port_handler_.get(), id, address.address, &present_speed, &error);
      joint_velocity_ = valueToRpm(present_speed);
      return getResult(result, error);
    }
    if (address.byte_size == PacketByteSize::FOUR_BYTE) {
      uint32_t present_speed = 0;
      const auto result = packet_handler_->read4ByteTxRx(
        port_handler_.get(), id, address.address, &present_speed, &error);
      joint_velocity_ = valueToRpm(present_speed);
      return getResult(result, error);
    }
    return Result("Invalid packet size", false);
  }
}

Result MotorBase::updateJointPosition()
{
  const auto address = address_table_->getAddress(Operation::PRESENT_POSITION);
  if (!address.exists()) {
    return Result("PRESENT_POSITION operation does not support in " + toString(motor_type), false);
  }
  if (enable_dummy) {
    joint_position_ = goal_position_;
    return Result("", true);
  } else {
    uint8_t error = 0;
    if (address.byte_size == PacketByteSize::ONE_BYTE) {
      uint8_t present_position = 0;
      const auto result = packet_handler_->read1ByteTxRx(
        port_handler_.get(), id, address.address, &present_position, &error);
      joint_position_ = positionToRadian(present_position);
      return getResult(result, error);
    }
    if (address.byte_size == PacketByteSize::TWO_BYTE) {
      uint16_t present_position = 0;
      const auto result = packet_handler_->read2ByteTxRx(
        port_handler_.get(), id, address.address, &present_position, &error);
      joint_position_ = positionToRadian(present_position);
      return getResult(result, error);
    }
    if (address.byte_size == PacketByteSize::FOUR_BYTE) {
      uint32_t present_position = 0;
      const auto result = packet_handler_->read4ByteTxRx(
        port_handler_.get(), id, address.address, &present_position, &error);
      joint_position_ = positionToRadian(present_position);
      return getResult(result, error);
    }
    return Result("Invalid packet size", false);
  }
}

Result MotorBase::updatePresentTemperature()
{
  const auto address = address_table_->getAddress(Operation::PRESENT_TEMPERATURE);
  if (!address.exists()) {
    return Result(
      "PRESENT_TEMPERATURE operation does not support in " + toString(motor_type), false);
  }
  if (enable_dummy) {
    present_temperature_ = 0;
    return Result("", true);
  } else {
    uint8_t error = 0;
    if (address.byte_size == PacketByteSize::ONE_BYTE) {
      uint8_t present_temperature = 0;
      const auto result = packet_handler_->read1ByteTxRx(
        port_handler_.get(), id, address.address, &present_temperature, &error);
      present_temperature_ = valueToTemperature(present_temperature);
      return getResult(result, error);
    }
    if (address.byte_size == PacketByteSize::TWO_BYTE) {
      uint16_t present_temperature = 0;
      const auto result = packet_handler_->read2ByteTxRx(
        port_handler_.get(), id, address.address, &present_temperature, &error);
      present_temperature_ = valueToTemperature(present_temperature);
      return getResult(result, error);
    }
    if (address.byte_size == PacketByteSize::FOUR_BYTE) {
      uint32_t present_temperature = 0;
      const auto result = packet_handler_->read4ByteTxRx(
        port_handler_.get(), id, address.address, &present_temperature, &error);
      present_temperature_ = valueToTemperature(present_temperature);
      return getResult(result, error);
    }
    return Result("Invalid packet size", false);
  }
}
}  //  namespace dynamixel_hardware_interface
