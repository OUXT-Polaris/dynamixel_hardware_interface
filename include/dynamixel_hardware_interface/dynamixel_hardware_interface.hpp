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

#ifndef DYNAMIXEL_HARDWARE_INTERFACE__DYNAMIXEL_HARDWARE_INTERFACE_HPP_
#define DYNAMIXEL_HARDWARE_INTERFACE__DYNAMIXEL_HARDWARE_INTERFACE_HPP_

#include <dynamixel_hardware_interface/visiblity_control.h>
#include <dynamixel_hardware_interface/motors/motors.hpp>

#include <rclcpp/rclcpp.hpp>

#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_status_values.hpp>

#include <vector>

namespace dynamixel_hardware_interface
{
class DynamixelHardwareInterface
  : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
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
};
}  // namespace dynamixel_hardware_interface

#endif  // DYNAMIXEL_HARDWARE_INTERFACE__DYNAMIXEL_HARDWARE_INTERFACE_HPP_
