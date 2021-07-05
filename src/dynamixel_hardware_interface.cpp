/**
 * @file dynamixel_hardware_interface.cpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief Class implementation of the hardware interface for the Dynamixel motor.
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

#include <dynamixel_hardware_interface/dynamixel_hardware_interface.hpp>
#include <memory>
#include <string>
#include <vector>

namespace dynamixel_hardware_interface
{
hardware_interface::return_type DynamixelHardwareInterface::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != hardware_interface::return_type::OK) {
    return hardware_interface::return_type::ERROR;
  }
  RCLCPP_INFO(
    rclcpp::get_logger("dynamixel_hardware_interface"), "configure hardware " + info.name);
  for (const auto hardware_parameter : info_.hardware_parameters) {
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger("dynamixel_hardware_interface"),
      "hardware parameter : " << hardware_parameter.first << " = " << hardware_parameter.second);
  }
  port_name_ = getHardwareParameter<std::string>("port_name");
  baudrate_ = getHardwareParameter<int>("baudrate");
  RCLCPP_INFO(rclcpp::get_logger("dynamixel_hardware_interface"), "initialize port handler");
  port_handler_ = std::shared_ptr<dynamixel::PortHandler>(
    dynamixel::PortHandler::getPortHandler(port_name_.c_str()));
  RCLCPP_INFO(rclcpp::get_logger("dynamixel_hardware_interface"), "initialize packet handler");
  packet_handler_ = std::shared_ptr<dynamixel::PacketHandler>(
    dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION));
  if (!getHardwareParameter<bool>("enable_dummy")) {
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger("dynamixel_hardware_interface"),
      "serial port : " << port_handler_->getPortName());
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger("dynamixel_hardware_interface"),
      "baudrate : " << port_handler_->getBaudRate());
    if (port_handler_->openPort()) {
      RCLCPP_INFO(rclcpp::get_logger("dynamixel_hardware_interface"), "open serial port succeed");
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("dynamixel_hardware_interface"), "open serial port failed");
      return hardware_interface::return_type::ERROR;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("dynamixel_hardware_interface"), "configure each motors");
  for (const auto joint : info.joints) {
    std::shared_ptr<MotorBase> motor;
    try {
      motor = constructMotorInstance(joint);
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR(rclcpp::get_logger("dynamixel_hardware_interface"), e.what());
      return hardware_interface::return_type::ERROR;
    }
    const auto result = motor->configure();
    if (!result.success) {
      RCLCPP_ERROR(rclcpp::get_logger("dynamixel_hardware_interface"), result.description);
      return hardware_interface::return_type::ERROR;
    }
    motors_.emplace_back(motor);
  }
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
DynamixelHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces = {};
  for (const auto motor : motors_) {
    motor->appendStateInterfaces(state_interfaces);
  }
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("dynamixel_hardware_interface"), state_interfaces.size()
                                                          << " state interfaces exported.");
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DynamixelHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces = {};
  for (const auto motor : motors_) {
    motor->appendCommandInterfaces(command_interfaces);
  }
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("dynamixel_hardware_interface"), command_interfaces.size()
                                                          << " command interfaces exported.");
  return command_interfaces;
}

SupportedMotors DynamixelHardwareInterface::strToSupportMotorsEnum(
  const std::string & motor_type) const
{
  if (motor_type == "XM430-W350") {
    return SupportedMotors::XM430_W350;
  }
  else if (motor_type == "XW540-T260") {
    return SupportedMotors::XW540_T260;
  }
  return SupportedMotors::INVALID;
}

std::shared_ptr<MotorBase> DynamixelHardwareInterface::constructMotorInstance(
  const hardware_interface::ComponentInfo & info) const
{
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("dynamixel_hardware_interface"),
    "constructing motor instance : " << info.name);
  for (const auto parameter : info.parameters) {
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger("dynamixel_hardware_interface"),
      "parameter " << parameter.first << " : " << parameter.second);
  }
  if (info.type == "joint") {
    const auto motor_type = strToSupportMotorsEnum(getParameter<std::string>("motor_type", info));
    if (motor_type == SupportedMotors::INVALID) {
      throw std::runtime_error("failed to construct motor instance, motor type is invalid");
    }
    const auto id = static_cast<uint8_t>(getParameter<int>("id", info));
    switch (motor_type) {
      case SupportedMotors::XM430_W350:
        return std::make_shared<motors::XM430_W350>(
          info.name, getHardwareParameter<bool>("enable_dummy"), baudrate_, id, port_handler_,
          packet_handler_);
        break;
      case SupportedMotors::XW540_T260:
        return std::make_shared<motors::XW540_T260>(
          info.name, getHardwareParameter<bool>("enable_dummy"), baudrate_, id, port_handler_,
          packet_handler_);
        break;
      default:
        break;
    }
  }
  throw std::runtime_error("failed to construct motor instance");
}

hardware_interface::return_type DynamixelHardwareInterface::start()
{
  status_ = hardware_interface::status::STARTED;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DynamixelHardwareInterface::stop()
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DynamixelHardwareInterface::read()
{
  for (const auto motor : motors_) {
    if (motor->operationSupports(Operation::PRESENT_POSITION)) {
      const auto result = motor->updateJointPosition();
      if (!result.success) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("dynamixel_hardware_interface"), result.description);
        return hardware_interface::return_type::ERROR;
      }
    }
    if (motor->operationSupports(Operation::PRESENT_SPEED)) {
      const auto result = motor->updateJointVelocity();
      if (!result.success) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("dynamixel_hardware_interface"), result.description);
        return hardware_interface::return_type::ERROR;
      }
    }
    if (motor->operationSupports(Operation::PRESENT_TEMPERATURE)) {
      const auto result = motor->updatePresentTemperature();
      if (!result.success) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("dynamixel_hardware_interface"), result.description);
        return hardware_interface::return_type::ERROR;
      }
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DynamixelHardwareInterface::write()
{
  for (const auto motor : motors_) {
    if (motor->operationSupports(Operation::GOAL_POSITION)) {
      motor->setCurrentGoalPosition();
    }
  }
  return hardware_interface::return_type::OK;
}
}  // namespace dynamixel_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dynamixel_hardware_interface::DynamixelHardwareInterface, hardware_interface::SystemInterface)
