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

#include <dynamixel_hardware_interface/dynamixel_hardware_interface.hpp>

#include <memory>
#include <string>

namespace dynamixel_hardware_interface
{
hardware_interface::return_type DynamixelHardwareInterface::configure(
  const hardware_interface::HardwareInfo & info)
{
  port_name_ = info_.hardware_parameters.at("port_name");
  baudrate_ = std::stoi(info_.hardware_parameters.at("baudrate"));
  port_handler_ = std::shared_ptr<dynamixel::PortHandler>(
    dynamixel::PortHandler::getPortHandler(port_name_.c_str()));
  packet_handler_ = std::shared_ptr<dynamixel::PacketHandler>(
    dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION));
  for (const auto joint : info.joints) {
    const auto motor = constructMotorInstance(joint);
    const auto result = motor->configure();
    if (!result.success) {
      RCLCPP_ERROR(rclcpp::get_logger("dynamixel_hardware_interface"), result.description);
      return hardware_interface::return_type::ERROR;
    }
    motors_.emplace_back(motor);
  }
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> DynamixelHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces = {};
  for (const auto motor : motors_) {
    motor->appendStateInterfaces(state_interfaces);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DynamixelHardwareInterface::
export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces = {};
  for (const auto motor : motors_) {
    motor->appendCommandInterfaces(command_interfaces);
  }
  return command_interfaces;
}

SupportedMotors DynamixelHardwareInterface::strToSupportMotorsEnum(const std::string & motor_type)
const
{
  if (motor_type == "XW54-T260") {
    return SupportedMotors::XW54_T260;
  }
  throw std::runtime_error(motor_type + " does not supported yet.");
}

std::shared_ptr<MotorBase> DynamixelHardwareInterface::constructMotorInstance(
  const hardware_interface::ComponentInfo & info) const
{
  if (info.type == "joint") {
    const auto motor_type = strToSupportMotorsEnum(info.parameters.at("motor_type"));
    uint8_t id = static_cast<uint8_t>(std::stoi(info.parameters.at("id")));
    switch (motor_type) {
      case SupportedMotors::XW54_T260:
        return std::make_shared<MotorBase>(
          motors::XW54_T260(
            info.name,
            baudrate_,
            id,
            port_handler_,
            packet_handler_));
    }
  }
  throw std::runtime_error("failed to construct motor instance");
}

hardware_interface::return_type DynamixelHardwareInterface::start()
{
  status_ = hardware_interface::status::STARTED;
  return hardware_interface::return_type::OK;
}
}  // namespace dynamixel_hardware_interface
