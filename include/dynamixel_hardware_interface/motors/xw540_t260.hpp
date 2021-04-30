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

#ifndef DYNAMIXEL_HARDWARE_INTERFACE__MOTORS__XW540_T260_HPP_
#define DYNAMIXEL_HARDWARE_INTERFACE__MOTORS__XW540_T260_HPP_

#include <cmath>
#include <dynamixel_hardware_interface/address_table_base.hpp>
#include <dynamixel_hardware_interface/motor_base.hpp>
#include <limits>
#include <memory>
#include <string>

namespace dynamixel_hardware_interface
{
namespace address_tables
{
class XW540_T260 : public AddressTableBase
{
public:
  XW540_T260() : AddressTableBase(64, 116, boost::none, 132, 128, boost::none, boost::none, 146) {}
};
}  // namespace address_tables

namespace motors
{
class XW540_T260 : public MotorBase
{
public:
  explicit XW540_T260(
    const std::string joint_name, int baudrate, uint8_t id,
    std::shared_ptr<dynamixel::PortHandler> port_handler,
    std::shared_ptr<dynamixel::PacketHandler> packet_handler)
  : MotorBase(
      SupportedMotors::XW540_T260, joint_name, address_tables::XW540_T260(), baudrate, id,
      port_handler, packet_handler)
  {
  }
};
}  // namespace motors
}  // namespace dynamixel_hardware_interface

#endif  // DYNAMIXEL_HARDWARE_INTERFACE__MOTORS__XW540_T260_HPP_
