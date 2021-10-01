/**
 * @file dynamixel_hardware_interface.hpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief Hardware interface class for dynamixel motor.
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

#ifndef DYNAMIXEL_HARDWARE_INTERFACE__DYNAMIXEL_HARDWARE_INTERFACE_HPP_
#define DYNAMIXEL_HARDWARE_INTERFACE__DYNAMIXEL_HARDWARE_INTERFACE_HPP_

#include <dynamixel_hardware_interface/visiblity_control.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

#include <dynamixel_hardware_interface/motors/motors.hpp>
#if GALACTIC
#include <hardware_interface/system_interface.hpp>
#else
#include <hardware_interface/base_interface.hpp>
#endif
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#if GALACTIC
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#else
#include <hardware_interface/types/hardware_interface_status_values.hpp>
#endif
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace dynamixel_hardware_interface
{
/**
 * @brief Hardware interface for the dynamixel motor.
 */
class DynamixelHardwareInterface
#if GALACTIC
: public hardware_interface::SystemInterface
#else
: public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
#endif
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DynamixelHardwareInterface)

  DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type start() override;

  DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type stop() override;

  DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read() override;

  DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write() override;

private:
  std::string port_name_;
  int baudrate_;
  SupportedMotors strToSupportMotorsEnum(const std::string & motor_type) const;

  template <typename T>
  T getParameter(const std::string key, const hardware_interface::ComponentInfo & info) const
  {
    T param;
    getParameter(key, info, param);
    return param;
  }
  void getParameter(
    const std::string & key, const hardware_interface::ComponentInfo & info,
    std::string & parameter) const
  {
    try {
      parameter = info.parameters.at(key);
    } catch (std::out_of_range & e) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("dynamixel_hardware_interface"),
        "parameter : " << key << " does not exist.");
    }
  }
  void getParameter(
    const std::string & key, const hardware_interface::ComponentInfo & info, int & parameter) const
  {
    std::string param_string;
    getParameter(key, info, param_string);
    parameter = std::stoi(param_string);
  }
  void getParameter(
    const std::string & key, const hardware_interface::ComponentInfo & info, bool & parameter) const
  {
    parameter = false;
    std::string param_string;
    getParameter(key, info, param_string);
    if (param_string == "true" || param_string == "True") {
      parameter = true;
    }
  }
  template <typename T>
  T getHardwareParameter(const std::string key) const
  {
    T param;
    getHardwareParameter(key, param);
    return param;
  }
  void getHardwareParameter(const std::string & key, std::string & parameter) const
  {
    try {
      parameter = info_.hardware_parameters.at(key);
    } catch (std::out_of_range & e) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("dynamixel_hardware_interface"),
        "hardware parameter : " << key << " does not exist.");
    }
  }
  void getHardwareParameter(const std::string & key, int & parameter) const
  {
    std::string param_string;
    getHardwareParameter(key, param_string);
    parameter = std::stoi(param_string);
  }
  void getHardwareParameter(const std::string & key, bool & parameter) const
  {
    parameter = false;
    std::string param_string;
    getHardwareParameter(key, param_string);
    if (param_string == "true" || param_string == "True") {
      parameter = true;
    }
  }
  std::shared_ptr<MotorBase> constructMotorInstance(
    const hardware_interface::ComponentInfo & info) const;
  std::vector<std::shared_ptr<MotorBase>> motors_;
  std::shared_ptr<dynamixel::PortHandler> port_handler_;
  std::shared_ptr<dynamixel::PacketHandler> packet_handler_;
};
}  // namespace dynamixel_hardware_interface

#endif  // DYNAMIXEL_HARDWARE_INTERFACE__DYNAMIXEL_HARDWARE_INTERFACE_HPP_
